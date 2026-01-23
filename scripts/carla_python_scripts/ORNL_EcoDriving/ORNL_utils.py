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

origin = GpsOrigin(
        lat_deg=36.123456,   # example
        lon_deg=-86.987654,  # example
        h_m=200.0            # meters (ellipsoidal height)
    )

carla_tf = CarlaTransform(
    offset_x=0.0,          # ENU origin coincides with CARLA (0,0,0)
    offset_y=0.0,
    offset_z=0.0
)

mcity_origin = { 
                "x": 6378136.5, 
                "y": 857.6, 
                "z": 935.9
            }

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

def process_SPaT(hex_data, greenDuration=24, redDuration=17, MAP_Record=None):
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
        intersectionID, greenWin = getGreenWindow(hex_data, reference_timestamp, greenDuration=greenDuration, redDuration=redDuration, MAP_Record=MAP_Record)
        return True, greenWin, intersectionID
    else:
        return False, {}, None

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
        # print("Decoded BSM: ")
        # print(str(decoded_msg.to_json()) + "\n")

        bsmId = decoded_bsm['value'][1]['coreData']['id']
        decoded_bsm['value'][1]['coreData']['id'] = str(bsmId.hex())
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
        #xyz = lat_long_to_xyz_better(lat/1e7, longstr/1e7, 0)
        
        #x, y = xyz['x'] - mcity_origin['x'], -xyz['y'] + mcity_origin['y']
        #x, y, z = xyz['y'], xyz['x'], xyz['z']
        x, y, z = gps_to_carla(lat/10**7, longstr/10**7, elevation/10)

        #if decoded_bsm['value'][1]['coreData']['id'] == "f03ad620":
        if decoded_bsm['value'][1]['coreData']['id'] == "f03ad658":
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
            # return True, x, y, speed_converted
            # return True, lat/1e7, longstr/1e7, speed_converted

            print('Ego BSM Coordinate no offset: ', x, y, z)
            return True, lat/1e7, longstr/1e7, speed_converted
        # elif decoded_bsm['value'][1]['coreData']['id'] == "f03ad658":
        else:
            #print('Ego BSM Coordinate: ', x-mcity_origin['x'], y-mcity_origin['y'], z)
            print('Lead BSM Coordinate: ', decoded_bsm['value'][1]['coreData']['id'], x, y, z)

    return False, 0, 0, 0

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
        decoded_msg_json = decoded_msg.to_json()
        # print("Decoded SPaT: ")
        # print(str(decoded_msg_json)+"\n")
    except Exception as err:
        print(f"Unexpected {err}, {type(err)}")
        raise
    return decoded_msg


def getGreenWindow(j2735_tena, reference_timestamp, greenDuration, redDuration, MAP_Record=None):
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
    cycleLength = 90
    timingPlan = {
        '1': {'green': 49, 'red': cycleLength - 49},
        '2': {'green': 50, 'red': cycleLength - 50},
        '3': {'green': 50, 'red': cycleLength - 50},
        '4': {'green': 64, 'red': cycleLength - 64},
        '5': {'green': 64, 'red': cycleLength - 64},
        '6': {'green': 64, 'red': cycleLength - 64},
        '7': {'green': 59, 'red': cycleLength - 59},
        '8': {'green': 49, 'red': cycleLength - 49}
    }
    decoded_msg = spat_data_process(j2735_tena)
    spatPhaseArray = [""] * 31
    intersectionID = decoded_msg()['value'][1]['intersections'][0]['id']['id']

    if MAP_Record:
        if str(intersectionID) in MAP_Record or intersectionID in MAP_Record:
            signalGroup = int(MAP_Record[str(intersectionID)])
            print("##############################Determined signal group from MAP_Record: ", signalGroup, ' ##############################')
        else:
            signalGroup = 2  # default to phase 2
    else:
        signalGroup = 2  # default to phase 2

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
    # phase2State = spatPhaseArray[2]
    phase2State = spatPhaseArray[signalGroup]
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
        # t1e = t1s + greenDuration
        t1e = t1s + timingPlan[str(intersectionID)]['green']
        r1s = currentTimeReference

    # t2s = t1e + redDuration
    t2s = t1e + timingPlan[str(intersectionID)]['red']
    # t2e = t2s + greenDuration
    t2e = t2s + timingPlan[str(intersectionID)]['green']

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


def determine_signal_phase_from_map(ego_location, ego_latlong, map_message=None, ego_heading=None, max_search_distance=150.0):
    """Return the traffic signal group (phase) that matches the ego vehicle location and travel direction.

    Parameters
    ----------
    ego_location : carla.Location | dict | tuple
        Current ego pose. Supports CARLA ``Location`` objects, dictionaries with
        ``x``/``y`` or ``lat``/``long`` keys, or 2-tuples of (x, y).
    map_message : str | dict | callable | None
        MAP data as a hex string, already-decoded dictionary, or the callable
        returned by ``J2735.DSRC.MessageFrame`` after ``from_uper`` is invoked.
        If ``None``, the most recently decoded MAP will be reused so that lane
        selection can continue between MAP broadcasts.
    ego_heading : float | None
        Optional vehicle heading/yaw in degrees (CARLA convention). If omitted,
        the function attempts to infer yaw from a CARLA ``Transform`` or a
        dictionary with ``yaw``/``heading`` keys.
    max_search_distance : float, optional
        Maximum allowed distance (meters) between the ego position and the
        projected lane centerline before giving up; defaults to 100 meters.

    Returns
    -------
    dict | None
        When a candidate lane is found a dictionary is returned with the keys
        ``signal_group``, ``lane_id``, ``distance``, ``intersection_id`` and
        ``approach_id``. ``None`` is returned if no viable signal group can be
        determined and no cached selection exists. Cached results are annotated
        with ``stale: True``.
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
        # if isinstance(payload, str):
        #     stripped = payload.strip().lower()
        #     if stripped.startswith('0x'):
        #         stripped = stripped[2:]
        #     try:
        #         return json.loads(payload)
        #     except (json.JSONDecodeError, TypeError):
        #         pass
        if payload.startswith("0012"):
            try:
                decoded = J2735.DSRC.MessageFrame
                decoded.from_uper(ba.unhexlify(payload))
                # print(str(decoded.to_json()) + "\n")
                return decoded()
            except Exception:
                return None
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
        delta_items = []
        if isinstance(delta, dict):
            delta_items = delta.items()
        elif isinstance(delta, tuple) and len(delta) == 2 and isinstance(delta[0], str):
            delta_items = [(delta[0], delta[1])]
        if not delta_items:
            return None, None
        for key, value in delta_items:
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
        if isinstance(node_list, dict):
            nodes = node_list.get('nodes')
        elif isinstance(node_list, tuple) and len(node_list) == 2 and node_list[0] == 'nodes':
            nodes = node_list[1]
        else:
            nodes = None
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

    ## 1. Decode MAP message (or reuse cached) and convert ego location
    decoded_map = _decode_map(map_message) if map_message is not None else None
    if decoded_map is not None:
        _cached_map_state['decoded_map'] = decoded_map
        _cached_map_state['timestamp'] = time.time()
    else:
        decoded_map = _cached_map_state['decoded_map']
    
    #print('## [DEBUG] decoded MAP: ', decoded_map)
    
    ego_xy = _location_to_xy(ego_location)
    heading_vec = _heading_vector(ego_location, ego_heading)
    if decoded_map is None or ego_xy is None:
        if _cached_map_state['last_match'] is not None:
            stale = dict(_cached_map_state['last_match'])
            stale['stale'] = True
            return stale
        return None

    ## 2. Iterate through intersections and lanes to find best matching signal group
    map_body = decoded_map.get('value') if isinstance(decoded_map, dict) else None
    # print('## [DEBUG] decoded MAP body: ', map_body)
    if isinstance(map_body, list) or isinstance(map_body, tuple) and len(map_body) > 1:
        map_body = map_body[1]
    # print('## [DEBUG] decoded MAP body: ', type(map_body))
    if not isinstance(map_body, dict):
        if _cached_map_state['last_match'] is not None:
            stale = dict(_cached_map_state['last_match'])
            stale['stale'] = True
            return stale
        return None
    intersections = map_body.get('intersections', [])
    # print('## [DEBUG]', ' Intersections found in MAP message: ', len(intersections))
    if intersections:
        for intersection in intersections:
            ref_lat, ref_lon, ref_elev = _extract_lat_lon(intersection.get('refPoint'))
            print('## [DEBUG] Intersection LatLongElev: ', ref_lat, ref_lon, ref_elev)
            intersection_id = intersection.get('id', {}).get('id')
            if ref_lat is None or ref_lon is None or intersection_id is None:
                continue
            ref_xy = _latlon_to_local_xy(ref_lat, ref_lon, ref_elev)
            print('## [DEBUG] Intersection coordinate: ', ref_xy)
            _cached_intersections[intersection_id] = {
                'intersection': intersection,
                'ref_xy': ref_xy,
                'geo': (ref_lat, ref_lon, ref_elev),
                'timestamp': time.time()
            }

    if not _cached_intersections:
        if _cached_map_state['last_match'] is not None:
            stale = dict(_cached_map_state['last_match'])
            stale['stale'] = True
            return stale
        return None

    # print('## [DEBUG] Intersections: ', _cached_intersections)
    ego_xy_vec = np.array(ego_xy)
    candidate_list = []
    for cached in _cached_intersections.values():
        intersection = cached['intersection']
        ref_xy = cached['ref_xy']
        ref_geo = cached['geo']
        #vec_to_intersection = np.array(ref_xy) - ego_xy_vec
        #dist_to_intersection = np.linalg.norm(vec_to_intersection)
        dist_to_intersection, ego_latlong_real = distance_real_latlon(ego_latlong, (ref_geo[0], ref_geo[1]))
        ref_ego_xy = _latlon_to_local_xy(ego_latlong_real[0], ego_latlong_real[1], ref_elev)
        vec_to_intersection = np.array(ref_xy) - np.array(ref_ego_xy)
        if dist_to_intersection < 1.0:
            continue
        if heading_vec is not None:
            forward_component = np.dot(vec_to_intersection, heading_vec)
            print('## [DEBUG] Intersection specs: ', forward_component, dist_to_intersection)
            if forward_component <= 0:
                continue
        candidate_list.append((dist_to_intersection, intersection, ref_xy, ref_geo))

    #print('## [DEBUG] Intersection candidates: ', candidate_list)

    if not candidate_list:
        if _cached_map_state['last_match'] is not None:
            stale = dict(_cached_map_state['last_match'])
            stale['stale'] = True
            return stale
        return None

    candidate_list.sort(key=lambda item: item[0])
    target_distance, target_intersection, target_ref_xy, target_geo = candidate_list[0]

    #print('## [DEBUG] Target: ', target_intersection)

    best_match = None
    lane_set = target_intersection.get('laneSet', [])
    for lane in lane_set:
        signal_groups = _collect_signal_groups(lane)
        print('## [DEBUG] Signal_groups: ', signal_groups)
        if not signal_groups:
            continue
        lane_points = _build_lane_points(lane, target_ref_xy, target_geo)
        if len(lane_points) < 2:
            continue
        distance = _point_to_polyline_distance(ref_ego_xy, lane_points)
        # if distance > max_search_distance:
        #     continue
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
    print('## [DEBUG] Target: ', best_match['signal_group'])
    if best_match is not None:
        _cached_map_state['last_match'] = dict(best_match)
        return best_match

    if _cached_map_state['last_match'] is not None:
        stale = dict(_cached_map_state['last_match'])
        stale['stale'] = True
        return stale

    return None

def determine_signal_phase_from_map_latlon(ego_latlon, map_message=None, ego_heading=None, max_search_distance=150.0):
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
        dist2StopLine, _ = distance_real_latlon(point_latlon, polyline[0])
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
        return best, dist2StopLine

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
    # print('## [DEBUG]', ' Intersections found in MAP message: ', len(intersections))
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
        # print(f'## [DEBUG] Intersection lat-lon: {ref_geo}, Dist to ego: {dist_to_intersection}, Ego lat-lon: {ego_latlon_real}')
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
    # print('## [DEBUG] Candidate metrics: ', target_distance, target_ego_latlon_real)

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
        distance, dist2StopLine = _point_to_polyline_distance_latlon(target_ego_latlon_real, lane_points)
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
                'dist2StopLine': dist2StopLine,
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

def determine_leader(ego_location, bsm_message=None, ego_heading=None, carla_info = None, max_search_distance=125.0):
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

    # print('[DEBUG] BSM_INFO: ', _cached_vehicles)
    # print('[DEBUG] CARLA INFO: ', carla_info)
    
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
        vehicle_fwd_vec = _heading_vector(cached['position'], cached['heading'])
        opposite = np.dot(vehicle_fwd_vec, heading_vec)
        if dist_to_vehicle < 0.5 or abs_d_perp >= 1.5 or opposite < 0:
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

def determine_leader_carla(ego_transform, nearby_vehicles, max_search_distance=100.0):
    """Return the lead vehicle that is closest to the ego vehicle position and travel direction.

    Parameters
    ----------
    ego_transform : carla.Transform
        Current ego pose.
    nearby_vehicles : list of carla.Vehicle
        List of nearby vehicles to consider as potential leaders.
    max_search_distance : float, optional
        Maximum allowed distance (meters) between the ego position and the
        lead vehicle before giving up; defaults to 100 meters.

    Returns
    -------
    carla.Vehicle | None
        When a candidate lead vehicle is found, the corresponding carla.Vehicle object is returned.
        ``None`` is returned if no viable lead vehicle can be determined.
    """
    if not nearby_vehicles:
        return None
    
    ego_xy = (ego_transform.location.x, ego_transform.location.y)
    heading_vec = _heading_vector(ego_transform.location, ego_transform.rotation.yaw)
    ego_fwd = ego_transform.get_forward_vector()

    candidate_list = []
    for vehicle in nearby_vehicles:
        vehicle_transform = vehicle.get_transform()
        vehicle_xy = (vehicle_transform.location.x, vehicle_transform.location.y)
        vec_to_vehicle = np.array(vehicle_xy) - np.array(ego_xy)
        dist_to_vehicle = np.linalg.norm(vec_to_vehicle)
        _, _, abs_d_perp = distances_to_heading(ego_xy[0], ego_xy[1], ego_transform.rotation.yaw, vehicle_xy[0], vehicle_xy[1])
        if dist_to_vehicle > max_search_distance or abs_d_perp >=1.5:
            continue
        if heading_vec is not None:
            forward_component = np.dot(vec_to_vehicle, heading_vec)
            oth_fwd = vehicle_transform.get_forward_vector()
            opposite = (ego_fwd.x * oth_fwd.x + ego_fwd.y * oth_fwd.y)
            if forward_component <= 0 or opposite < 0:
                continue
        candidate_list.append((vehicle, dist_to_vehicle))

    if not candidate_list:
        return None
    
    candidate_list.sort(key=lambda item: item[1])

    return candidate_list[0]

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

        # look_ahead_dis = 1.*desired_speed + 0
        look_ahead_dis = 4
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

import csv

class vehicle_logger(object):
    def __init__(self, outfile):
        self.csvout = open(outfile, 'w')
        self.csv_w = csv.writer(self.csvout)
        self.headers = ["TimeStamp", "x", "y", "Heading", "Speed", "Accel", "DesiredSpd", "Speed_Lead", "Spacing", "Dist2StopBar"]
        self.csv_w.writerow(self.headers)
        self.time = time.time()

    def record(self, data):
        currentRow = []
        next_t = time.time()
        currentRow.append('%.3f'%(next_t-self.time))
        #self.time = next_t
        for i in data:
            currentRow.append('%.3f'%(i))
        
        self.csv_w.writerow(currentRow)

INTERSECTION_XY = {
    '1': (-633.43, 774.79),
    '2': (-487.62, 771.05),
    '3': (-381.11, 760.60),
    '4': (-133.25, 734.62),
    '5': (53.89, 713.76),
    '6': (231.28, 709.92),
    '7': (360.30, 720.24),
    # '8': (475.90, 802.15),
    '8': (530.47, 843.83)
}

def get_closest_intersection_carla(ego_location, ego_heading=None):
    """ Get the closest intersection in CARLA coordinates.
    Parameters
    ----------
    ego_location : carla.Location | dict | tuple
        Current ego pose. Supports CARLA ``Location`` objects, dictionaries with
        ``x``/``y`` or ``lat``/``long`` keys, or 2-tuples of (x, y).
    Returns
    -------
    tuple | None
        (x, y) coordinates of the closest intersection in CARLA coordinates,
        or ``None`` if the ego location could not be interpreted.
    """
    ego_xy = _location_to_xy(ego_location)
    heading_vec = _heading_vector(ego_location)
    if ego_xy is None:
        return list(INTERSECTION_XY.keys())[-1], None
    min_dist = float('inf')
    closest_id, closest_xy = list(INTERSECTION_XY.keys())[-1], None
    for inter_id, inter_xy in INTERSECTION_XY.items():
        vec_to_vehicle = np.array(inter_xy) - np.array(ego_xy)
        forward_component = np.dot(vec_to_vehicle, heading_vec)
        if forward_component <= 0:
            continue
        dist = math.hypot(ego_xy[0] - inter_xy[0], ego_xy[1] - inter_xy[1])
        if dist < min_dist:
            min_dist = dist
            closest_xy = inter_xy
            closest_id = inter_id
    #print(f"Closest intersection: ID={closest_id}, Location={closest_xy}, Distance={min_dist:.2f} m")
    return closest_id, min_dist
