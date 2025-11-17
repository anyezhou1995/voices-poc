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

carla_egg_file = find_carla_egg()

sys.path.append(carla_egg_file)

import carla

mcity_origin = { 
                "x": 518508.658, 
                "y": -4696054.02, 
                "z": 0
            }
barPos_x, barPos_y = 53.33, -23.77

draw_lifetime = 1/60

def draw_box(world, x, y, z):
    box_center = carla.Location(x=x, y=y, z=z)
 
    vru_box = carla.BoundingBox(box_center,carla.Vector3D(1.5,1.5,0))
 
    world.debug.draw_box(
        vru_box,
        carla.Rotation(0,0,0),
        0.2,
        # draw_shadow=False,
        color=carla.Color(r=255, g=0, b=0),
        life_time=draw_lifetime,
        persistent_lines=True)

def GeodeticToEcef( latitude, longitude,altitude):
    # WGS-84 geodetic constants
    a = 6378137.0        # WGS-84 Earth semimajor axis (m)

    b = 6356752.314245;     # Derived Earth semiminor axis (m)
    f = (a - b) / a          # Ellipsoid Flatness
    f_inv = 1.0 / f      # Inverse flattening
    a_sq = a * a
    b_sq = b * b
    e_sq = f * (2 - f)    # Square of Eccentricity

    # Convert to radians in notation consistent with the paper:
    lambdaa = latitude * (3.141592653589793 / 180.0)
    phi = longitude * (3.141592653589793 / 180.0)
    s = math.sin(lambdaa)
    N = a / math.sqrt(1 - e_sq * s * s)

    sin_lambda = math.sin(lambdaa)
    cos_lambda = math.cos(lambdaa)
    cos_phi = math.cos(phi)
    sin_phi = math.sin(phi)

    x = (altitude + N) * cos_lambda * cos_phi
    y = (altitude + N) * cos_lambda * sin_phi
    z = (altitude + (1 - e_sq) * N) * sin_lambda

    return { "x":x, "y": y, "z": z }

def lat_long_to_xyz_better(latitude, longitude, altitude):
    # WGS 84 parameters
    semi_major_axis = 6378137.0  # in meters
    flattening = 1 / 298.257223563

    # Convert latitude and longitude from degrees to radians
    lat_rad = latitude * (3.141592653589793 / 180.0)
    lon_rad = longitude * (3.141592653589793 / 180.0)

    # Calculate the radius of curvature in the prime vertical
    N = semi_major_axis / math.sqrt(1 - flattening * (2 - flattening) * math.sin(lat_rad)**2)

    # Calculate Cartesian coordinates
    x = (N + altitude) * math.cos(lat_rad) * math.cos(lon_rad)
    y = (N + altitude) * math.cos(lat_rad) * math.sin(lon_rad)
    z = ((1 - flattening)**2 * N + altitude) * math.sin(lat_rad)

    return { "x":x, "y": y, "z": z }

def lat_lon_alt_to_xyz(latitude, longitude, altitude):
    # Earth radius in meters (average value)
    earth_radius = 6371000.0

    # Convert latitude and longitude from degrees to radians
    lat_rad = math.radians(latitude)
    lon_rad = math.radians(longitude)

    # Calculate Cartesian coordinates
    x = (earth_radius + altitude) * math.cos(lat_rad) * math.cos(lon_rad)
    y = (earth_radius + altitude) * math.cos(lat_rad) * math.sin(lon_rad)
    z = (earth_radius + altitude) * math.sin(lat_rad)

    return { "x":x, "y": y, "z": z }

def process_SPaT(hex_data):
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
        print("Received SPaT")
        greenWin = getGreenWindow(hex_data, reference_timestamp, greenDuration=40, redDuration=30)
        return True, greenWin
    else:
        return False, {}

def process_BSM(hex_data):
    '''
    UDP_IP = "10.7.108.81"
    UDP_PORT = 5398

    sock = socket.socket(   socket.AF_INET, # Internet
                            socket.SOCK_DGRAM) # UDP
    sock.bind((UDP_IP, UDP_PORT))

    data, addr = sock.recvfrom(4096) # buffer size is 1024 bytes
    hex_data = data.hex()
    '''

    if hex_data.startswith("0014"):
        decoded_msg = J2735.DSRC.MessageFrame

        decoded_msg.from_uper(ba.unhexlify(hex_data))
        # decoded_bsm = decoded_msg.to_json()
        decoded_bsm = decoded_msg()
        #print(decoded_bsm)

        bsmId = decoded_bsm['value'][1]['coreData']['id']
        decoded_bsm['value'][1]['coreData']['id'] = str(bsmId.hex())
        if decoded_bsm['value'][1]['coreData']['id'] == "f03ad620":
        #if decoded_bsm['value'][1]['coreData']['id'] == "f03ad627":
            #print("Received BSM")
            lat= decoded_bsm['value'][1]['coreData']['lat']
            longstr = decoded_bsm['value'][1]['coreData']['long']
            speed = decoded_bsm['value'][1]['coreData']['speed']
            elevation = decoded_bsm['value'][1]['coreData']['elev']
            secMark = decoded_bsm['value'][1]['coreData']['secMark']
            heading = decoded_bsm['value'][1]['coreData']['heading']
            speed_converted = speed*0.02 #m/s
            accel_long = decoded_bsm['value'][1]['coreData']['accelSet']['long']
            accel_long_converted = accel_long*0.01 #m^s^2

            xyz = GeodeticToEcef(lat/10**7, longstr/10**7, elevation/10)
            #xyz = lat_long_to_xyz_better(lat/10**7, longstr/10**7, 0)
            x, y = xyz['x'] - mcity_origin['x'], -xyz['y'] + mcity_origin['y']

            #print('BSM position: ', lat, longstr, elevation)

            '''
            xyz1 = GeodeticToEcef(lat, longstr, elevation)
            xyz2 = lat_long_to_xyz_better(lat, longstr, elevation)
            print("x: " + str(xyz['x']-mcity_origin['x']) + "; y: " + str(xyz['y']-mcity_origin['y']) + \
                "; z: " + str(xyz['z']-mcity_origin['z']) + "; Speed: " + str(speed_converted) + \
                "; Accel: " + str(accel_long_converted))
            print('Original lat and Long: ', lat, longstr)
            print('XY: ', xyz['x'], xyz['y'])
            print('XY: ', xyz1['x'], xyz1['y'])
            #print(xyz2['x'], xyz2['y'])
            print('MCity origin: ', mcity_origin['x'], mcity_origin['y'])
            '''
            return True, x, y, speed_converted
    return False, 0, 0, 0

def decode_msg():
    ##############################################################
    # Decode msg
    ##############################################################
    numSpatPhases = 31 #use one more than desired phases

    UDP_IP = "10.7.108.81"
    UDP_PORT = 5398

    sock = socket.socket(   socket.AF_INET, # Internet
                            socket.SOCK_DGRAM) # UDP
    sock.bind((UDP_IP, UDP_PORT))

    data, addr = sock.recvfrom(4096) # buffer size is 1024 bytes
    hex_data = data.hex()
    # print("received message: %s" % hex_data)

    if hex_data.startswith("0013"):
        print("Received SPaT")
        decoded_msg = J2735.DSRC.MessageFrame
        decoded_msg.from_uper(ba.unhexlify(hex_data))
        # decoded_spat = decoded_msg.to_json()
        decoded_spat = decoded_msg()

        spatPhaseArray = [""] * numSpatPhases
        intersectionID = decoded_spat['value'][1]['intersections'][0]['id']['id']
        try:
            intersectionName = decoded_spat['value'][1]['intersections'][0]['name']
        except:
            intersectionName = ""
        spatTimestamp = decoded_spat['value'][1]['intersections'][0]['timeStamp']
        instersectionPhaseArray = decoded_spat['value'][1]['intersections'][0]['states']
        
        
        for phase in range(len(instersectionPhaseArray)):
            currentPhase = decoded_spat['value'][1]['intersections'][0]['states'][phase].get('signalGroup')
            currentState = str(decoded_spat['value'][1]['intersections'][0]['states'][phase]['state-time-speed'][0]['eventState'])
            spatPhaseArray[currentPhase] = currentState
        
        '''
        for phase in range(len(instersectionPhaseArray)):
            if decoded_spat['value'][1]['intersections'][0]['states'][phase]['signalGroup'] == 2:
                currentState = str(decoded_spat['value'][1]['intersections'][0]['states'][phase]['state-time-speed'][0]['eventState'])
                endTime = str(decoded_spat['value'][1]['intersections'][0]['states'][phase]['state-time-speed'][0]['timing']['minEndTime'])
                print(currentState, endTime)
        '''
        spatRowList = [str(spatTimestamp),str(intersectionID),intersectionName]
        for printPhase in range(1,numSpatPhases):
            spatRowList.append(spatPhaseArray[printPhase])

        #print("Decoded SPaT: ")
        # print(decoded_spat['value'][1]['intersections'][0]['name'])
        #print(str(spatRowList) + "\n")
        #print(decoded_msg.to_json())
    
    '''
    if hex_data.startswith("0014"):
        print("Received BSM")
        decoded_msg = J2735.DSRC.MessageFrame
        decoded_msg.from_uper(ba.unhexlify(hex_data))
        # decoded_bsm = decoded_msg.to_json()
        decoded_bsm = decoded_msg()

        bsmId = decoded_bsm['value'][1]['coreData']['id']
        lat= decoded_bsm['value'][1]['coreData']['lat']
        longstr = decoded_bsm['value'][1]['coreData']['long']
        speed = decoded_bsm['value'][1]['coreData']['speed']
        elevation = decoded_bsm['value'][1]['coreData']['elev']
        secMark = decoded_bsm['value'][1]['coreData']['secMark']
        heading = decoded_bsm['value'][1]['coreData']['heading']
        speed_converted = speed*0.02 #m/s
        accel_long = decoded_bsm['value'][1]['coreData']['accelSet']['long']
        accel_long_converted = accel_long*0.01 #m^s^2

        print("Decoded BSM: ")
        print("Lat: " + str(lat) + ", " + "Long: " + str(longstr) + ", " + "Speed: " + str(speed_converted) + ", " + \
            "Heading: " + str(heading) + ", " + "Accel: " + str(accel_long_converted) + "\n")
    '''

    if hex_data.startswith("0012"):
        print("Received MAP")
        decoded_msg = J2735.DSRC.MessageFrame
        decoded_msg.from_uper(ba.unhexlify(hex_data))
        decoded_map = decoded_msg()
        print("Decoded MAP: ")
        print(str(decoded_msg.to_json()) + "\n")

def configToDict(data):
    """
    Parses config contents into a dictionary.

    Parameters
    ----------
    data : ConfigParser.read() output
        content of config file in ConfigParser format

    Returns
    -------
    dict
        dictionary of config file
    """
    config = {}
    for section in data.sections():
        config[section] = {}
        for item in data[section]:
            config[section][item] = data[section][item]
    return config


def spat_data_process(j2735_tena):
    # place holder function
    try:
        # specify message type inside J2735.py
        decoded_msg = J2735.DSRC.MessageFrame
        # convert from hex using unhexlify then from uper using library
        decoded_msg.from_uper(ba.unhexlify(j2735_tena))
        # format data into json
        # decoded_msg_json = decoded_msg.to_json()

        print('')
        # print(decoded_msg_json)
    except Exception as err:
        print(f"Unexpected {err}, {type(err)}")
        raise
    return decoded_msg


def getGreenWindow(j2735_tena, reference_timestamp, greenDuration, redDuration):
    # covnert json msg into SPaT info
    """
    :param j2735_tena: SPaT data hex string
    :return: dict
        'status':
        't1s':
        't1e':
        't2s':
        't2e':
        'r1s':
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

    #assume we are approaching phase 2, otherwise, it need to be determined based on MAP data.
    phase2State = spatPhaseArray[2]
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
        print('{}:{}:{}'.format(hour, minute, minEndTimeSecond))
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
    #print(greenWindow)

    return greenWindow


def determine_signal_phase_from_map(ego_location, map_message, max_search_distance=100.0):
    """Return the traffic signal group (phase) that matches the ego vehicle location and travel direction.

    Parameters
    ----------
    ego_location : carla.Location | dict | tuple
        Current ego pose. Supports CARLA ``Location`` objects, dictionaries with
        ``x``/``y`` or ``lat``/``long`` keys, or 2-tuples of (x, y).
    map_message : str | dict | callable
        MAP data as a hex string, already-decoded dictionary, or the callable
        returned by ``J2735.DSRC.MessageFrame`` after ``from_uper`` is invoked.
    max_search_distance : float, optional
        Maximum allowed distance (meters) between the ego position and the
        projected lane centerline before giving up; defaults to 100 meters.

    Returns
    -------
    dict | None
        When a candidate lane is found a dictionary is returned with the keys
        ``signal_group``, ``lane_id``, ``distance``, ``intersection_id`` and
        ``approach_id``. ``None`` is returned if no viable signal group can be
        determined.
    """

    def _decode_map(payload):
        """ Decode MAP message from various input formats.
        Parameters
        ----------
        payload : str | dict | callable
            MAP data as a hex string, already-decoded dictionary, or the callable
            returned by ``J2735.DSRC.MessageFrame`` after ``from_uper`` is invoked.
        Returns
        -------
        dict | None
            Decoded MAP message as a dictionary, or ``None`` if decoding failed.
        """
        if payload is None:
            return None
        if isinstance(payload, dict):
            return payload
        if hasattr(payload, '__call__'):
            return payload()
        if isinstance(payload, str):
            stripped = payload.strip().lower()
            if stripped.startswith('0x'):
                stripped = stripped[2:]
            try:
                return json.loads(payload)
            except (json.JSONDecodeError, TypeError):
                pass
            try:
                decoded = J2735.DSRC.MessageFrame
                decoded.from_uper(ba.unhexlify(stripped))
                return decoded()
            except Exception:
                return None
        return None

    def _latlon_to_local_xy(lat_deg, lon_deg, elevation_m=0.0):
        """ Convert lat/lon to local XY coordinates relative to MCITY origin.
        ## TODO: use Delave map origin to replace mcity_origin
        """
        xyz = GeodeticToEcef(lat_deg, lon_deg, elevation_m)
        return xyz['x'] - mcity_origin['x'], -xyz['y'] + mcity_origin['y']

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

    def _node_delta_to_offset(delta, ref_geo):
        """ Convert a node delta to an (x, y) offset.
        Returns a tuple of (offset, mode), where offset is a numpy array
        of (x, y) coordinates, and mode is either 'absolute' or 'relative'.
        Parameters
        ----------
        delta : dict
            Delta dictionary from a lane node.
        ref_geo : tuple
            Reference (lat, lon, elevation) for the intersection reference point.
        Returns
        -------
        np.array | None, str | None
            Offset as a numpy array and mode string, or (None, None) if
            the delta could not be interpreted.
        """
        if not isinstance(delta, dict):
            return None, None
        for key, value in delta.items():
            ## Handle relative XY offsets: if the MAP stores x/y deltas in centimeters/decimeters 
            ## relative to the previous point (or to the intersection reference point for the first node)
            if key.startswith('node-XY') and isinstance(value, dict):
                x_raw = value.get('x')
                y_raw = value.get('y')
                if x_raw is None or y_raw is None:
                    continue
                return np.array([x_raw / 10.0, y_raw / 10.0]), 'relative'
            ## If MAPs periodically “reset” the running offset by specifying a full latitude/longitude 
            ## (plus optional elevation) instead of a delta
            if key == 'node-LatLon' and isinstance(value, dict):
                lat = value.get('lat')
                lon = value.get('lon') or value.get('long')
                elev = value.get('elevation') or value.get('elev') or ref_geo[2]
                if lat is None or lon is None:
                    continue
                if abs(lat) > 90:
                    lat = lat / 1e7
                if abs(lon) > 180:
                    lon = lon / 1e7
                if abs(elev) > 10000:
                    elev = elev / 10.0
                absolute_xy = _latlon_to_local_xy(lat, lon, elev)
                return np.array(absolute_xy), 'absolute'
        return None, None

    def _build_lane_points(lane, ref_xy, ref_geo):
        """ Build a list of (x, y) points for the given lane.
        Uses the lane's nodeList to construct the points, interpreting
        deltas as either absolute lat/lon or relative x/y offsets.
        Parameters
        ----------
        lane : dict
            Lane dictionary from the MAP message.
        ref_xy : tuple | None
            Reference (x, y) coordinates for the intersection reference point.
        ref_geo : tuple
            Reference (lat, lon, elevation) for the intersection reference point.
        Returns
        -------
        list of (x, y) tuples
            List of points defining the lane centerline.
        """
        node_list = lane.get('nodeList') if isinstance(lane, dict) else None
        if not node_list:
            return []
        nodes = node_list.get('nodes') if isinstance(node_list, dict) else None
        if not nodes:
            return []
        points = []
        last_point = None
        for node in nodes:
            delta = node.get('delta') if isinstance(node, dict) else None
            offset, mode = _node_delta_to_offset(delta, ref_geo)
            if offset is None:
                continue
            if mode == 'absolute':
                last_point = offset
            else:
                if last_point is None:
                    if ref_xy is None:
                        continue
                    last_point = np.array(ref_xy) + offset
                else:
                    last_point = last_point + offset
            points.append((float(last_point[0]), float(last_point[1])))
        return points

    def _point_to_polyline_distance(point, polyline):
        """ Compute the minimum distance from a point to a polyline.
        Parameters
        ----------
        point : tuple
            (x, y) coordinates of the point.
        polyline : list of tuples
            List of (x, y) coordinates defining the polyline.
        Returns
        -------
        float
            Minimum distance from the point to the polyline.
        """
        if point is None or not polyline:
            return float('inf')
        px, py = point
        best = float('inf')
        for idx in range(len(polyline) - 1):
            x1, y1 = polyline[idx]
            x2, y2 = polyline[idx + 1]
            dx = x2 - x1
            dy = y2 - y1
            if dx == 0 and dy == 0:
                dist = math.hypot(px - x1, py - y1)
            else:
                t = ((px - x1) * dx + (py - y1) * dy) / (dx * dx + dy * dy)
                t = max(0.0, min(1.0, t))
                proj_x = x1 + t * dx
                proj_y = y1 + t * dy
                dist = math.hypot(px - proj_x, py - proj_y)
            if dist < best:
                best = dist
        return best

    def _collect_signal_groups(lane):
        """ Put the signal groups of the lane and lanes connected to it into a list.
        Parameters
        ----------
        lane : dict
            Lane dictionary from the MAP message.
        Returns
        -------
        list of int
            List of unique signal groups associated with the lane.
        """
        groups = []
        for connection in lane.get('connectsTo', []):
            signal_group = connection.get('signalGroup')
            if signal_group is None and isinstance(connection.get('connectingLane'), dict):
                signal_group = connection['connectingLane'].get('signalGroup')
            if signal_group is not None and signal_group not in groups:
                groups.append(signal_group)
        return groups

    ## 1. Decode MAP message
    decoded_map = _decode_map(map_message)
    ego_xy = _location_to_xy(ego_location)
    if decoded_map is None or ego_xy is None:
        return None

    ## 2. Iterate through intersections and lanes to find best matching signal group
    map_body = decoded_map.get('value') if isinstance(decoded_map, dict) else None
    if isinstance(map_body, list) and len(map_body) > 1:
        map_body = map_body[1]
    if not isinstance(map_body, dict):
        return None
    intersections = map_body.get('intersections', [])
    if not intersections:
        return None

    best_match = None
    for intersection in intersections:
        # get intersection reference point
        ref_lat, ref_lon, ref_elev = _extract_lat_lon(intersection.get('refPoint'))
        if ref_lat is None or ref_lon is None:
            continue
        # get reference point in local XY
        ref_xy = _latlon_to_local_xy(ref_lat, ref_lon, ref_elev)
        # get lanes associated with the intersection
        lane_set = intersection.get('laneSet', [])
        for lane in lane_set:
            signal_groups = _collect_signal_groups(lane)
            if not signal_groups:
                continue
            # get lane centerline points
            lane_points = _build_lane_points(lane, ref_xy, (ref_lat, ref_lon, ref_elev))
            if len(lane_points) < 2:
                continue
            # compute distance from ego vehicle to lane centerline
            distance = _point_to_polyline_distance(ego_xy, lane_points)
            if distance > max_search_distance:
                continue
            # collect best match
            if best_match is None or distance < best_match['distance']:
                best_match = {
                    'signal_group': signal_groups[0],
                    'lane_id': lane.get('laneID'),
                    'distance': distance,
                    'intersection_id': intersection.get('id', {}).get('id'),
                    'approach_id': lane.get('ingressApproach') or lane.get('egressApproach')
                }

    return best_match


def get_advisory_speed(cav_spd, cav_acc, dist2Stop, precedSpeed, gapDist, reference_timestamp, SpatData):
    """
    :param cav_spd: vehicle current speed, mph
    :param cav_acc: vehicle current acceleration, m/s2
    :param dist2Stop: distance to the next traffic light, feet
    :param precedSpeed: speed of the lead vehicle, mph
    :param gapDist: the gap with the lead vehicle, feet
    :param j2735_tena: J2735 message received from TENA adapter in real time
    :return:
    """
    FlowData = {'flow_rate': 100, 'speed_agg': 30}

    example_coasting_profile = pd.read_csv('example_coasting_profile.csv', index_col=0)

    parser = ConfigParser()
    parser.read('shallowford.ini')
    config = configToDict(parser)

    A = float(config['Vehicle Coasting']['a'])
    B = float(config['Vehicle Coasting']['b'])
    C = float(config['Vehicle Coasting']['c'])
    M = float(config['Vehicle Coasting']['m'])

    orginal_desire_spd = float(config['Speed Limit']['orginal_desire_spd'])
    next_movement = config['Movement']['next_movement']

    if SpatData != 0:
        current_time = datetime.now()
        ########################################### This offset 5180s is only for local testing################################################
        current_time_rel = (current_time.hour * 3600 + current_time.minute * 60 + current_time.second) - \
                           ((reference_timestamp.hour * 60 + reference_timestamp.minute) * 60 + reference_timestamp.second) + 5180
        current_time_rel = SpatData['currentTime']

        data_toSave = [cav_spd, cav_acc,
                        current_time_rel, dist2Stop,
                        precedSpeed, gapDist, 2,
                        FlowData['flow_rate'], FlowData['speed_agg'],
                        SpatData['status'], SpatData['t1s'],
                        SpatData['t1e'], SpatData['t2s'],
                        SpatData['t2e'], SpatData['r1s']]
        
        with open('./data4Debug_2.txt', 'a') as t:
            t.write(json.dumps(data_toSave) + '\n')
        try:
            queue_length, instant_desired_speed, mode, a_out = gen_desired_spd(example_coasting_profile, A, B, C, M,
                                                                               orginal_desire_spd, next_movement, cav_spd, cav_acc,
                                                                               current_time_rel, dist2Stop,
                                                                               precedSpeed, gapDist, 2,
                                                                               FlowData['flow_rate'], FlowData['speed_agg'],
                                                                               SpatData['status'], SpatData['t1s'],
                                                                               SpatData['t1e'], SpatData['t2s'],
                                                                               SpatData['t2e'], SpatData['r1s'])
        except:
            return 0, data_toSave, 1

        #print('Before mod:', current_time_rel - 5180)
        print('current time: {}, \ncurrent speed: {}, \nqueue_length: {}, \ninstant_desired_speed: {}, \nmode: {}, \nfollow_distance: {}\n'.
              format(int(current_time_rel), int(cav_spd), int(queue_length), int(instant_desired_speed), mode, gapDist))

        print('\n----------------------------------------------------------------------------------------------------')

        data = {'cpu_time': time.time(), "x_ntl": dist2Stop, "clr_ntl": SpatData['status'],
                "v_advisory": instant_desired_speed, "a_advisory": a_out, 'mode': mode}

    else:
        instant_desired_speed = cav_spd
        a_out = cav_acc
        mode = 0

        data = {'cpu_time': time.time(), "x_ntl": dist2Stop, "clr_ntl": SpatData['status'],
                "v_advisory": instant_desired_speed, "a_advisory": a_out, 'mode': mode}

        print('SpatData is incomplete')

    #print(data)

    return data['v_advisory'], data_toSave, 0


def find_closest_waypoint(wp_list, ego_transform, speed, start_id):
    min_idx = 0
    min_dist = 1e6
    min_wp = wp_list[0]
    wp_l = wp_list[start_id:]
    for i, wp in enumerate(wp_l):
        dist = np.sqrt((ego_transform.location.x - wp[0])**2 + (ego_transform.location.y - wp[1])**2)
        if dist < min_dist:
            min_idx = i
            min_dist = dist
            min_wp = wp
    if min_idx == 0 and start_id != 0 and min_idx+start_id+1 < len(wp_list) and speed > 0.1:
        min_idx += 1

    return min_idx+start_id, min_wp

def search_target_index(cx, cy, veh_trans, desired_speed):

    def closest_index_on_trajectory():
        dx = cx - veh_trans.location.x
        dy = cy - veh_trans.location.y
        return np.argmin(np.hypot(dx, dy))

    def look_ahead_idx_from(closest_index):
        target_index = closest_index

        look_ahead_dis = 1.*desired_speed + 0
        #look_ahead_dis = 4.5
        while look_ahead_dis > np.hypot(cx[target_index]-veh_trans.location.x, cy[target_index]-veh_trans.location.y):
            if (target_index + 1) >= len(cx) or desired_speed < 0.1:
                break
            target_index += 1
        return target_index

    return look_ahead_idx_from(closest_index_on_trajectory())

def search_target_index_v2(cx, cy, veh_trans, desired_speed, distance, distance_traveled):
    """
    Parameters:
    cx: list of x coordinates of the waypoint breadcrumbs
    cy: list of y coordinates of the waypoint breadcrumbs
    veh_trans: currnet vehicle transform
    desired_speed: desired speed of the vehicle
    distance: list of cumulative distances along the waypoint breadcrumbs
    distance_traveled: current distance traveled by the ego vehicle

    Returns:
    target_index: index of the target waypoint ahead of the vehicle
    """
    def closest_index_on_trajectory():
        # complexity of log(n)
        idx = np.searchsorted(distance, distance_traveled)
        candidates = []
        for i in [-1, 0, 1]:
            candidate_idx = idx + i
            if 0 <= candidate_idx < len(cx):
                candidates.append(candidate_idx)
        dx = cx[candidates] - veh_trans.location.x
        dy = cy[candidates] - veh_trans.location.y
        return candidates[np.argmin(np.hypot(dx, dy))]

    def look_ahead_idx_from(closest_index):
        target_index = closest_index

        look_ahead_dis = 1.*desired_speed + 0
        #look_ahead_dis = 4.5
        while look_ahead_dis > np.hypot(cx[target_index]-veh_trans.location.x, cy[target_index]-veh_trans.location.y):
            if (target_index + 1) >= len(cx) or desired_speed < 0.1:
                break
            target_index += 1
        return target_index

    return look_ahead_idx_from(closest_index_on_trajectory())

def search_target_index_lookBack(cx, cy, veh_trans, desired_speed):

    def closest_index_on_trajectory():
        dx = cx - veh_trans.location.x
        dy = cy - veh_trans.location.y
        return np.argmin(np.hypot(dx, dy))

    def look_back_idx_from(closest_index):
        target_index = closest_index

        look_ahead_dis = 4
        while look_ahead_dis > np.hypot(cx[target_index]-veh_trans.location.x, cy[target_index]-veh_trans.location.y):
            if (target_index - 1) >= 0:
                break
            target_index -= 1
        return target_index

    return look_back_idx_from(closest_index_on_trajectory())


def EcoControl(vehicle, lead_vehicle, move_speed, mov_loc):
    # settign up PID controller
    args_lateral = {'K_P': 1.95, 'K_D': 0.2, 'K_I': 0.075, 'dt': save_step}
    args_longitudinal = {'K_P': 1.0, 'K_D': 0.02, 'K_I': 0.05, 'dt': save_step}

    PID = VehiclePIDController(vehicle, args_lateral, args_longitudinal)

    control = PID.run_step(move_speed, move_loc)

    return control
