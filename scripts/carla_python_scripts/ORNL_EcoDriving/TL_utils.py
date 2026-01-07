import pandas as pd
import time
from datetime import datetime
#from speed_control_algorithm import gen_desired_spd
from speed_control_implementation_ggg import gen_desired_spd, IntelligentDriverModel
from configparser import ConfigParser
import J2735_201603_combined_voices_mr_fix as J2735
import socket
import json
import csv
import binascii as ba
import math, sys
import numpy as np

from find_carla_egg import find_carla_egg
from gps2carla import gps_to_carla, GpsOrigin, CarlaTransform, distance_real_latlon, distances_to_heading
from mapOffset import GpsCarlaPair, calibrate_enu_to_carla

carla_egg_file = find_carla_egg()

sys.path.append(carla_egg_file)

import carla

delave_origin = GpsOrigin(
    lat_deg=0.00841723681631434,
    lon_deg=0.00773563408642701,
    h_m=0,
)
barPos_x, barPos_y = 53.33, -23.77

draw_lifetime = 1/60

_cached_map_state = {
    "decoded_map": None,
    "timestamp": None,
    "last_match": None
}

_cached_intersections = {}
_cached_intersections_latlon = {}

_cached_bsm_state = {
    "decoded_bsm": None,
    "timestamp": None,
    "last_leader": None
}

_cached_vehicles = {}

def process_SPaT(hex_data, signal_group):
    '''
    UDP_IP = "10.7.108.81"
    UDP_PORT = 5398

    sock = socket.socket(   socket.AF_INET, # Internet
                            socket.SOCK_DGRAM) # UDP
    sock.bind((UDP_IP, UDP_PORT))

    data, addr = sock.recvfrom(4096) # buffer size is 1024 bytes
    hex_data = data.hex()
    '''

    reference_timestamp = datetime.strptime('06:30:00', '%H:%M:%S')

    if hex_data.startswith("0013"):
        # print("=============Received SPaT=================")
        intersectionID, greenWin = getGreenWindow(hex_data, reference_timestamp, signal_group=signal_group, greenDuration=40, redDuration=30)
        return True, greenWin, intersectionID
    else:
        return False, {}, None
    
def decode_map(hex_data):
    ##############################################################
    # Decode MAP
    ##############################################################
    if hex_data.startswith("0012"):
        print("=============Received MAP=================")
        decoded_msg = J2735.DSRC.MessageFrame
        decoded_msg.from_uper(ba.unhexlify(hex_data))
        decoded_map = decoded_msg()
        print("Decoded MAP: ")
        print(str(decoded_msg.to_json()) + "\n")

def spat_data_process(j2735_tena):
    # place holder function
    try:
        # specify message type inside J2735.py
        decoded_msg = J2735.DSRC.MessageFrame
        # convert from hex using unhexlify then from uper using library
        decoded_msg.from_uper(ba.unhexlify(j2735_tena))
        # format data into json
        decoded_msg_json = decoded_msg.to_json()
        # print("Decoded SPaT: ")
        # print(str(decoded_msg_json)+"\n")
    except Exception as err:
        print(f"Unexpected {err}, {type(err)}")
        raise
    return decoded_msg


def getGreenWindow(j2735_tena, reference_timestamp, signal_group=2, greenDuration=40, redDuration=30):
    # covnert json msg into SPaT info
    """
    :param j2735_tena: SPaT data hex string
    :return: dict
        'status': current signal status: green, red, yellow, str
        # t1s, t1e: the start and end time of the closest green, float
        # t2s, t2e: the start and end time of the second green, float
        # r1s:      the start time of first red, float
    """
    decoded_msg = spat_data_process(j2735_tena)
    spatPhaseArray = [""] * 31
    intersectionID = decoded_msg()['value'][1]['intersections'][0]['id']['id']
    try:
        intersectionName = decoded_msg()['value'][1]['intersections'][0]['name']
    except:
        intersectionName = ""

    spatTimestamp = decoded_msg()['value'][1]['intersections'][0]['timeStamp']
    moy = decoded_msg()['value'][1]['intersections'][0]['moy']
    timeOfDayInMin = moy % 1440
    hour = timeOfDayInMin//60
    minute = timeOfDayInMin % 60
    seconds = spatTimestamp/1000
    currentTime = '{}:{}:{}'.format(hour, minute, seconds)
    currentTimestamp = datetime.strptime(currentTime, '%H:%M:%S.%f')
    currentTimeReference = (currentTimestamp.hour * 3600 + currentTimestamp.minute * 60 + currentTimestamp.second) - \
                           ((reference_timestamp.hour * 60 + reference_timestamp.minute) * 60 + reference_timestamp.second)
    instersectionPhaseArray = decoded_msg()['value'][1]['intersections'][0]['states']
    # phase2Status = [i for i in instersectionPhaseArray if i]

    for phase in range(len(instersectionPhaseArray)):
        currentPhase = decoded_msg()['value'][1]['intersections'][0]['states'][phase].get('signalGroup')
        currentState = str(decoded_msg()['value'][1]['intersections'][0]['states'][phase]['state-time-speed'][0]['eventState'])
        try:
            minEndTime = decoded_msg()['value'][1]['intersections'][0]['states'][phase]['state-time-speed'][0]['timing']['minEndTime']
        except:
            minEndTime = None

        try:
            maxEndTime = decoded_msg()['value'][1]['intersections'][0]['states'][phase]['state-time-speed'][0]['timing']['maxEndTime']
        except:
            maxEndTime = None

        phaseState = dict({'state': currentState, 'minEndTime': minEndTime, 'maxEndTime': maxEndTime})
        spatPhaseArray[currentPhase] = phaseState

        #print(phase, spatPhaseArray)

    # Assume we are approaching phase 2, otherwise, it need to be determined based on MAP data.
    # phase2State = spatPhaseArray[2]
    phase2State = spatPhaseArray[signal_group]
    phase2Status = phase2State['state']
    phaseStatusDict = {'protected-Movement-Allowed': 'green',
                       'permissive-Movement-Allowed': 'green',
                       'permissive-clearance': 'yellow',
                       'protected-clearance': 'yellow',
                       'caution-Conflicting-Traffic': 'yellow',
                       'stop-Then-Proceed': 'red',
                       'stop-And-Remain': 'red'}
    phase2Status = phaseStatusDict[phase2Status]

    minEndTimeSecond = float(phase2State['minEndTime'] / 10 - minute * 60)
    minEndTimeSecond = round(minEndTimeSecond, 3)
    if minEndTimeSecond < 60:
        # print('{}:{}:{}'.format(hour, minute, minEndTimeSecond))
        minEndTimeStamp = datetime.strptime('{}:{}:{:.3f}'.format(hour, minute, minEndTimeSecond), '%H:%M:%S.%f')
    else:
        minEndTimeStamp = datetime.strptime(
            '{}:{}:{:.3f}'.format(hour, (minute + int(minEndTimeSecond // 60)), minEndTimeSecond % 60), '%H:%M:%S.%f')

    if phase2Status == 'green' or phase2Status == 'yellow':
        t1e = (minEndTimeStamp.hour * 3600 + minEndTimeStamp.minute * 60 + minEndTimeStamp.second) - \
                           ((reference_timestamp.hour * 60 + reference_timestamp.minute) * 60 + reference_timestamp.second)
        t1s = currentTimeReference
        r1s = t1e
    else:
        t1s = (minEndTimeStamp.hour * 3600 + minEndTimeStamp.minute * 60 + minEndTimeStamp.second) - \
                           ((reference_timestamp.hour * 60 + reference_timestamp.minute) * 60 + reference_timestamp.second)
        t1e = t1s + greenDuration
        r1s = currentTimeReference

    t2s = t1e + redDuration
    t2e = t2s + greenDuration

    greenWindow = {'currentTime': currentTimeReference, 'status': phase2Status, 't1s': t1s, 't1e': t1e, 't2s': t2s, 't2e': t2e, 'r1s': r1s}
    # print(greenWindow)

    return intersectionID, greenWindow

def _heading_vector(location, override_heading=None):
    """Return unit vector that represents ego forward direction."""
    yaw_deg = None
    if override_heading is not None:
        yaw_deg = override_heading
    elif isinstance(location, carla.Transform):
        yaw_deg = location.rotation.yaw
    elif isinstance(location, dict):
        yaw_deg = location.get('yaw') or location.get('heading')
    if yaw_deg is None:
        return None
    yaw_rad = math.radians(yaw_deg)
    return np.array([math.cos(yaw_rad), math.sin(yaw_rad)])

def _latlon_to_local_xy(lat_deg, lon_deg, elevation_m=0.0):
    """ Convert lat/lon to local XY coordinates relative to MCITY origin.
    ## TODO: use Delave map origin to replace mcity_origin
    """
    #xyz = GeodeticToEcef(lat_deg, lon_deg, elevation_m)
    x, y, z = gps_to_carla(lat_deg, lon_deg, elevation_m)
    #return xyz['x'] - mcity_origin['x'], -xyz['y'] + mcity_origin['y']
    return x, y
    # return y, -x

def _extract_lat_lon(record):
    """ Extract lat, lon, elevation from a record.
    Note that 2735 stores lat/lon as signed integers in 1e‑7 degrees. 
    Valid latitude is within ±90°, longitude within ±180°. 
    If the raw value has a magnitude larger than that, it’s almost certainly the scaled integer, 
    so we divide by 1e7 to get degrees.
    """
    if record is None:
        return None, None, 0.0
    lat = record.get('lat') if isinstance(record, dict) else None
    lon = record.get('long') if isinstance(record, dict) else None
    elev = record.get('elevation') if isinstance(record, dict) else 0.0
    if lat is None or lon is None:
        return None, None, 0.0
    if abs(lat) > 90:
        lat = lat / 1e7
    if abs(lon) > 180:
        lon = lon / 1e7
    if elev is None:
        elev = 0.0
    elif abs(elev) > 10000:
        elev = elev / 10.0
    return lat, lon, elev

def _location_to_xy(location):
    """ Convert various location formats to local XY coordinates.
    Will call _latlon_to_local_xy internally.
    """
    if location is None:
        return None
    if isinstance(location, carla.Transform):
        return location.location.x, location.location.y
    if isinstance(location, carla.Location):
        return location.x, location.y
    if isinstance(location, tuple) or isinstance(location, list):
        return float(location[0]), float(location[1])
    if isinstance(location, dict):
        if 'x' in location and 'y' in location:
            return float(location['x']), float(location['y'])
        lat_key = 'lat' if 'lat' in location else 'latitude' if 'latitude' in location else None
        lon_key = 'long' if 'long' in location else 'lon' if 'lon' in location else 'longitude' if 'longitude' in location else None
        if lat_key and lon_key:
            lat = location[lat_key]
            lon = location[lon_key]
            if abs(lat) > 90:
                lat = lat / 1e7
            if abs(lon) > 180:
                lon = lon / 1e7
            elev = location.get('elevation') or location.get('elev') or 0.0
            if abs(elev) > 10000:
                elev = elev / 10.0
            return _latlon_to_local_xy(lat, lon, elev)
    return None

def determine_signal_phase_from_map_latlon(ego_latlon, map_message=None, ego_heading=None, max_search_distance=100.0):
    """Determine the MAP signal group using only lat/lon values (no CARLA XY conversion).

    Parameters
    ----------
    ego_latlon : tuple | list | dict
        Ego latitude/longitude in degrees (or 1e7 scaled ints). Elevation is optional.
    map_message : str | dict | callable | None
        MAP message in any format accepted by ``determine_signal_phase_from_map``.
    ego_heading : float | None
        Optional heading/yaw in degrees where 0° points east and 90° points north.
    max_search_distance : float
        Maximum allowable lateral distance (meters) from the lane centerline.

    Returns
    -------
    dict | None
        Same structure as ``determine_signal_phase_from_map`` but distances are computed
        purely in meters on the WGS‑84 sphere. Cached results are annotated with
        ``stale: True`` when MAP/ego data are missing.
    """
    R_EARTH = 6378137.0

    def _decode_map(payload):
        if payload is None:
            return None
        if isinstance(payload, dict):
            return payload
        if hasattr(payload, '__call__'):
            return payload()
        if isinstance(payload, str) and payload.startswith("0012"):
            try:
                decoded = J2735.DSRC.MessageFrame
                decoded.from_uper(ba.unhexlify(payload))
                # print("######### Get MAP", str(decoded.to_json()) + "\n")
                return decoded()
            except Exception:
                return None
        return None

    def _normalize_latlon(value):
        if value is None:
            return None, None
        if isinstance(value, (tuple, list)) and len(value) >= 2:
            lat, lon = value[0], value[1]
        elif isinstance(value, dict):
            lat, lon, _ = _extract_lat_lon(value)
        else:
            return None, None
        if abs(lat) > 90:
            lat = lat / 1e7
        if abs(lon) > 180:
            lon = lon / 1e7
        return float(lat), float(lon)

    def _offset_to_latlon(ref_lat, ref_lon, east_m, north_m):
        lat_rad = math.radians(ref_lat)
        dlat = north_m / R_EARTH
        dlon = east_m / (R_EARTH * math.cos(lat_rad))
        return math.degrees(math.radians(ref_lat) + dlat), math.degrees(math.radians(ref_lon) + dlon)

    def _latlon_to_local_en(lat_deg, lon_deg, lat_ref_deg, lon_ref_deg):
        lat_ref_rad = math.radians(lat_ref_deg)
        dlat = math.radians(lat_deg - lat_ref_deg)
        dlon = math.radians(lon_deg - lon_ref_deg)
        north = dlat * R_EARTH
        east = dlon * R_EARTH * math.cos(lat_ref_rad)
        return east, north

    def _node_delta_to_latlon(delta, ref_geo, last_point):
        delta_items = []
        if isinstance(delta, dict):
            delta_items = delta.items()
        elif isinstance(delta, tuple) and len(delta) == 2 and isinstance(delta[0], str):
            delta_items = [(delta[0], delta[1])]
        if not delta_items:
            return None, None
        for key, value in delta_items:
            if key.startswith('node-XY') and isinstance(value, dict):
                x_raw = value.get('x')
                y_raw = value.get('y')
                if x_raw is None or y_raw is None:
                    continue
                east = x_raw / 10.0
                north = y_raw / 10.0
                base_lat, base_lon = last_point if last_point is not None else (ref_geo[0], ref_geo[1])
                lat, lon = _offset_to_latlon(base_lat, base_lon, east, north)
                return (lat, lon), 'relative'
            if key == 'node-LatLon' and isinstance(value, dict):
                lat = value.get('lat')
                lon = value.get('lon') or value.get('long')
                if lat is None or lon is None:
                    continue
                if abs(lat) > 90:
                    lat = lat / 1e7
                if abs(lon) > 180:
                    lon = lon / 1e7
                return (lat, lon), 'absolute'
        return None, None

    def _build_lane_points_latlon(lane, ref_geo):
        #print('[DEBUG] lane type: ', type(lane))
        node_list = lane.get('nodeList') if isinstance(lane, dict) else None
        #print('[DEBUG] node_list: ', node_list)
        if not node_list:
            return []
        if isinstance(node_list, dict):
            nodes = node_list.get('nodes')
        elif isinstance(node_list, tuple) and len(node_list) == 2 and node_list[0] == 'nodes':
            nodes = node_list[1]
        else:
            nodes = None
        #print('[DEBUG] nodes type: ', type(nodes))
        if not nodes:
            return []
        points = []
        last_point = None
        for node in nodes:
            delta = node.get('delta') if isinstance(node, dict) else None
            latlon, mode = _node_delta_to_latlon(delta, ref_geo, last_point)
            if latlon is None:
                continue
            last_point = latlon
            points.append((float(last_point[0]), float(last_point[1])))
        return points

    def _point_to_polyline_distance_latlon(point_latlon, polyline):
        if point_latlon is None or not polyline:
            return float('inf')
        lat_p, lon_p = point_latlon
        best = float('inf')
        for idx in range(len(polyline) - 1):
            lat1, lon1 = polyline[idx]
            lat2, lon2 = polyline[idx + 1]
            e2, n2 = _latlon_to_local_en(lat2, lon2, lat1, lon1)
            ep, np_ = _latlon_to_local_en(lat_p, lon_p, lat1, lon1)
            if e2 == 0 and n2 == 0:
                dist = math.hypot(ep, np_)
            else:
                t = ((ep * e2 + np_ * n2) / (e2 * e2 + n2 * n2))
                t = max(0.0, min(1.0, t))
                proj_e = t * e2
                proj_n = t * n2
                dist = math.hypot(ep - proj_e, np_ - proj_n)
            if dist < best:
                best = dist
        return best

    decoded_map = _decode_map(map_message) if map_message is not None else None
    if decoded_map is not None:
        _cached_map_state['decoded_map'] = decoded_map
        _cached_map_state['timestamp'] = time.time()
    else:
        decoded_map = _cached_map_state.get('decoded_map')

    ego_lat, ego_lon = _normalize_latlon(ego_latlon)
    heading_vec = _heading_vector(None, ego_heading)

    if decoded_map is None or ego_lat is None or ego_lon is None:
        if _cached_map_state.get('last_match_latlon') is not None:
            stale = dict(_cached_map_state['last_match_latlon'])
            stale['stale'] = True
            return stale
        return None

    map_body = decoded_map.get('value') if isinstance(decoded_map, dict) else None
    # print('## [DEBUG] decoded MAP body: ', map_body)
    if isinstance(map_body, (list, tuple)) and len(map_body) > 1:
        map_body = map_body[1]
    if not isinstance(map_body, dict):
        if _cached_map_state.get('last_match_latlon') is not None:
            stale = dict(_cached_map_state['last_match_latlon'])
            stale['stale'] = True
            return stale
        return None

    intersections = map_body.get('intersections', [])
    print('## [DEBUG]', ' Intersections found in MAP message: ', len(intersections))
    if intersections:
        for intersection in intersections:
            ref_lat, ref_lon, ref_elev = _extract_lat_lon(intersection.get('refPoint'))
            intersection_id = intersection.get('id', {}).get('id')
            if ref_lat is None or ref_lon is None or intersection_id is None:
                continue
            _cached_intersections_latlon[intersection_id] = {
                'intersection': intersection,
                'geo': (ref_lat, ref_lon, ref_elev),
                'timestamp': time.time()
            }

    if not _cached_intersections_latlon:
        if _cached_map_state.get('last_match_latlon') is not None:
            stale = dict(_cached_map_state['last_match_latlon'])
            stale['stale'] = True
            return stale
        return None

    candidate_list = []
    for cached in _cached_intersections_latlon.values():
        intersection = cached['intersection']
        ref_geo = cached['geo']
        dist_to_intersection, ego_latlon_real = distance_real_latlon((ego_lat, ego_lon), (ref_geo[0], ref_geo[1]))
        print('## [DEBUG] Intersection loc info: ', ref_geo, dist_to_intersection, ego_latlon_real)
        if dist_to_intersection < 1.0:
            continue
        if heading_vec is not None:
            east, north = _latlon_to_local_en(ref_geo[0], ref_geo[1], ego_latlon_real[0], ego_latlon_real[1])
            forward_component = east * heading_vec[0] + north * heading_vec[1]
            if forward_component <= 0:
                continue
        candidate_list.append((dist_to_intersection, intersection, ref_geo, ego_latlon_real))

    print('## [DEBUG] Intersection candidates: ', len(candidate_list))

    if not candidate_list:
        if _cached_map_state.get('last_match_latlon') is not None:
            stale = dict(_cached_map_state['last_match_latlon'])
            stale['stale'] = True
            return stale
        return None

    candidate_list.sort(key=lambda item: item[0])
    target_distance, target_intersection, target_geo, target_ego_latlon_real = candidate_list[0]

    #print('## [DEBUG] Ego original lat lon: ', ego_lat, ego_lon, target_geo)
    print('## [DEBUG] Candidate metrics: ', target_distance, target_ego_latlon_real)

    best_match = None
    lane_set = target_intersection.get('laneSet', [])
    #print('## [DEBUG] Lane set size: ', len(lane_set))
    for lane in lane_set:
        signal_groups = []
        for connection in lane.get('connectsTo', []):
            sg = connection.get('signalGroup')
            if sg is None and isinstance(connection.get('connectingLane'), dict):
                sg = connection['connectingLane'].get('signalGroup')
            if sg is not None and sg not in signal_groups:
                signal_groups.append(sg)
        
        #print('## [DEBUG] Signal groups: ', len(signal_groups))
        if not signal_groups:
            continue
        lane_points = _build_lane_points_latlon(lane, target_geo)
        #print('## [DEBUG] Lane points: ', len(lane_points))
        if len(lane_points) < 2:
            continue
        distance = _point_to_polyline_distance_latlon(target_ego_latlon_real, lane_points)
        # print('## [DEBUG] Lateral distance: ', distance)
        if distance > max_search_distance:
            continue
        if best_match is None or distance < best_match['distance']:
            best_match = {
                'signal_group': signal_groups[0],
                'lane_id': lane.get('laneID'),
                'distance': distance,
                'intersection_id': target_intersection.get('id', {}).get('id'),
                'approach_id': lane.get('ingressApproach') or lane.get('egressApproach'),
                'intersection_distance': target_distance,
                'stale': False
            }

    if best_match is not None:
        _cached_map_state['last_match_latlon'] = dict(best_match)
        return best_match

    if _cached_map_state.get('last_match_latlon') is not None:
        stale = dict(_cached_map_state['last_match_latlon'])
        stale['stale'] = True
        return stale

    return None

gpsPairs = []

def determine_leader(ego_location, bsm_message=None, ego_heading=None, carla_info = None, max_search_distance=100.0):
    """Return the lead vehicle BSM that is closest to the ego vehicle position and travel direction.

    Parameters
    ----------
    ego_location : carla.Location | dict | tuple
        Current ego pose. Supports CARLA ``Location`` objects, dictionaries with
        ``x``/``y`` or ``lat``/``long`` keys, or 2-tuples of (x, y).
    bsm_message : list of str | list of dict | None
        List of BSM data as hex strings, already-decoded dictionaries.
        If ``None``, the most recently decoded BSMs will be reused.
    ego_heading : float | None
        Optional vehicle heading/yaw in degrees (CARLA convention). If omitted,
        the function attempts to infer yaw from a CARLA ``Transform`` or a
        dictionary with ``yaw``/``heading`` keys.
    max_search_distance : float, optional
        Maximum allowed distance (meters) between the ego position and the
        lead vehicle before giving up; defaults to 100 meters.

    Returns
    -------
    dict | None
        When a candidate lead vehicle is found a dictionary is returned with the keys
        ``bsm_id``, ``distance``, ``leader_speed``, and ``leader_heading``.
        ``None`` is returned if no viable lead vehicle can be determined.
    """
    ## BSM decoder
    def _decode_bsm(payload):
        """ Decode BSM message from various input formats.
        Parameters
        ----------
        payload : str | dict
            BSM data as a hex string or already-decoded dictionary.
        Returns
        -------
        dict | None
            Decoded BSM message as a dictionary, or ``None`` if decoding failed.
        """
        if payload is None:
            return None
        if isinstance(payload, dict):
            return payload

        try:
            return json.loads(payload)
        except (json.JSONDecodeError, TypeError):
            pass
        #stripped = payload.strip().lower()
        if payload.startswith("0014"):
            #stripped = stripped[2:]
            # print("=============Received BSM=================")
            try:
                decoded = J2735.DSRC.MessageFrame
                decoded.from_uper(ba.unhexlify(payload))
                #print(str(decoded.to_json()) + "\n")
                return decoded()
            except Exception:
                return None
        return None
    ## 1. decode BSM messages (or reuse cached) and convert ego location
    decoded_bsm = _decode_bsm(bsm_message) if bsm_message is not None else None
    if decoded_bsm is not None:
        _cached_bsm_state['decoded_bsm'] = decoded_bsm
        _cached_bsm_state['timestamp'] = time.time()
    else:
        decoded_bsm = _cached_bsm_state['decoded_bsm']
    
    ego_xy = _location_to_xy(ego_location)
    heading_vec = _heading_vector(ego_location, ego_heading)  # Implement heading vector calculation similar to MAP processing
    
    if decoded_bsm is None or ego_xy is None:
        if _cached_bsm_state['last_leader'] is not None:
            stale = dict(_cached_bsm_state['last_leader'])
            stale['stale'] = True
            return stale
        return None
    bsm_body = decoded_bsm.get('value')[1] if isinstance(decoded_bsm, dict) else None
    if not isinstance(bsm_body, dict):
        if _cached_bsm_state['last_leader'] is not None:
            stale = dict(_cached_bsm_state['last_leader'])
            stale['stale'] = True
            return stale
        return None
    
    ## 2. Implement logic to find lead vehicles based on ego position and heading
    core_data = bsm_body.get('coreData', {})
    if core_data:
        bsm_id = str(core_data.get('id').hex())
        #print("Processing BSM ID:", bsm_id)
        lat, lon, elev = core_data.get('lat'), core_data.get('long'), core_data.get('elev')  # Extract from core_data
        bsm_xy = _latlon_to_local_xy(lat/1e7, lon/1e7, elev/10)  # Convert lat/lon to local XY
        bsm_heading = 90 - core_data.get('heading') * 0.0125  # Extract heading
        # print("[DEBUG] BSM LatLonElev: ", lat, lon, elev)
        # print("[DEBUG] BSM Coordinate: ", bsm_xy)
        # print("[DEBUG] Carla Coordinate: ", carla_info['pos_ego'])
        bsm_speed = core_data.get('speed') * 0.02  # Extract speed in m/s
        _cached_vehicles[bsm_id] = {
            'position': bsm_xy,
            'heading': bsm_heading,
            'speed': bsm_speed,
            'timestamp': time.time()
        }
        ## debug use, use ego BSM to set ego position
        ego_xy = _cached_vehicles['f03ad658']['position'] if 'f03ad658' in _cached_vehicles.keys() else ego_xy
        ego_heading = _cached_vehicles['f03ad658']['heading'] if 'f03ad658' in _cached_vehicles.keys() else 20
        lead_heading = _cached_vehicles['f03ad628']['heading'] if 'f03ad628' in _cached_vehicles.keys() else 9999
        #print(f"Ego heading BSM: {ego_heading}, Ego heading Carla: {carla_info['heading']}")
        #print(f"Lead heading BSM: {lead_heading}, Ego heading Carla: {carla_info['heading_lead']}")
        heading_vec = _heading_vector(ego_location, ego_heading)
        
        # if bsm_id == 'f03ad658':
        #     gpsPairs.append(GpsCarlaPair(lat/1e7, lon/1e7, elev/10,  carla_info['pos_ego'][0],  carla_info['pos_ego'][1], carla_info['pos_ego'][2]))

        #     if len(gpsPairs) > 100:
        #         gpsPairs.pop(0)
        #     if len(gpsPairs) > 2:
        #         tf = calibrate_enu_to_carla(gpsPairs, delave_origin)
        #         print("Calibrated transform:")
        #         print(f"  yaw_offset_deg = {tf.yaw_offset_deg:.6f}")
        #         print(f"  offset_x       = {tf.offset_x:.6f} m")
        #         print(f"  offset_y       = {tf.offset_y:.6f} m")
        #         print(f"  offset_z       = {tf.offset_z:.6f} m")

    print('[DEBUG] BSM_INFO: ', _cached_vehicles)
    print('[DEBUG] CARLA INFO: ', carla_info)
    
    if not _cached_vehicles:
        if _cached_bsm_state['last_leader'] is not None:
            stale = dict(_cached_bsm_state['last_leader'])
            stale['stale'] = True
            return stale
        return None
    
    ego_xy_vec = np.array(ego_xy)
    candidate_list = []
    for vehicle, cached in _cached_vehicles.items():
        vehicle_xy = cached['position']
        vehicle_heading = cached['heading']
        vehicle_speed = cached['speed']
        vec_to_vehicle = np.array(vehicle_xy) - ego_xy_vec
        dist_to_vehicle = np.linalg.norm(vec_to_vehicle)
        _, _, abs_d_perp = distances_to_heading(ego_xy[0], ego_xy[1], ego_heading, vehicle_xy[0], vehicle_xy[1])
        if dist_to_vehicle < 1.0 or abs_d_perp >= 2.1:
            continue
        if heading_vec is not None:
            #print("Heading Vec:", heading_vec, "heading angle: ", ego_heading)
            forward_component = np.dot(vec_to_vehicle, heading_vec)
            if forward_component <= 0:
                continue
        candidate_list.append((vehicle, dist_to_vehicle, vehicle_speed, vehicle_heading))

    # print("Candidate lead vehicles found:", len(candidate_list)-1)
    if not candidate_list:
        if _cached_bsm_state['last_leader'] is not None:
            stale = dict(_cached_bsm_state['last_leader'])
            stale['stale'] = True
            return stale
        return None
    
    candidate_list.sort(key=lambda item: item[1])
    vehicle_bsmID, target_distance, target_speed, target_heading = candidate_list[0]

    best_match = None
    if target_distance <= max_search_distance:
        best_match = {
            'bsm_id': vehicle_bsmID,
            'distance': target_distance,
            'lead_speed': target_speed,
            'lead_heading': target_heading,
            'stale': False
        }
    
    if best_match is not None:
        _cached_bsm_state['last_leader'] = dict(best_match)
        return best_match

    if _cached_bsm_state['last_leader'] is not None:
        stale = dict(_cached_bsm_state['last_leader'])
        stale['stale'] = True
        return stale
            
    return None