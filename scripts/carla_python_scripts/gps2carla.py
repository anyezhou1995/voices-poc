import math
from dataclasses import dataclass
from typing import Tuple

carla_origin = { 
                "x": 6378136.5, 
                "y": 857.6, 
                "z": 935.9
            }

# ==============================
#  WGS-84 CONSTANTS
# ==============================
WGS84_A = 6378137.0              # Semi-major axis (m)
WGS84_F = 1 / 298.257223563      # Flattening
WGS84_E2 = WGS84_F * (2 - WGS84_F)  # First eccentricity squared


# ==============================
#  DATA CLASSES
# ==============================
@dataclass
class GpsOrigin:
    lat_deg: float   # reference latitude  in degrees
    lon_deg: float   # reference longitude in degrees
    h_m: float       # reference ellipsoidal height in meters


@dataclass
class CarlaTransform:
    """
    Defines how to go from ENU (around GpsOrigin) to CARLA coordinates.
    - offset_x/y/z:   translation from ENU origin to CARLA origin (meters)
                      i.e., CARLA = Rz(yaw) * ENU + offset
    """
    offset_x: float
    offset_y: float
    offset_z: float


# ==============================
#  STEP 1: GEODETIC -> ECEF
# ==============================
def geodetic_to_ecef(lat_deg: float, lon_deg: float, h_m: float) -> Tuple[float, float, float]:
    """
    Convert geodetic coordinates (lat, lon, h) in WGS-84 to ECEF (X, Y, Z) in meters.
    lat_deg, lon_deg in degrees; h_m in meters (height above ellipsoid).
    """
    lat = math.radians(lat_deg)
    lon = math.radians(lon_deg)

    sin_lat = math.sin(lat)
    cos_lat = math.cos(lat)
    sin_lon = math.sin(lon)
    cos_lon = math.cos(lon)

    # Radius of curvature in the prime vertical
    N = WGS84_A / math.sqrt(1.0 - WGS84_E2 * sin_lat * sin_lat)

    X = (N + h_m) * cos_lat * cos_lon
    Y = (N + h_m) * cos_lat * sin_lon
    Z = (N * (1.0 - WGS84_E2) + h_m) * sin_lat

    return X, Y, Z


# ==============================
#  STEP 2: ECEF -> ENU
# ==============================
def ecef_to_enu(X: float, Y: float, Z: float,
                origin: GpsOrigin) -> Tuple[float, float, float]:
    """
    Convert ECEF (X, Y, Z) to local ENU (east, north, up) around 'origin'.
    origin is given in geodetic coordinates (lat, lon, h).
    """
    lat0 = math.radians(origin.lat_deg)
    lon0 = math.radians(origin.lon_deg)
    h0 = origin.h_m

    # ECEF of the origin
    X0, Y0, Z0 = geodetic_to_ecef(origin.lat_deg, origin.lon_deg, h0)

    dx = X - X0
    dy = Y - Y0
    dz = Z - Z0

    sin_lat0 = math.sin(lat0)
    cos_lat0 = math.cos(lat0)
    sin_lon0 = math.sin(lon0)
    cos_lon0 = math.cos(lon0)

    # Rotation matrix from ECEF to ENU
    # [ e ]   [ -sin(lon0)              cos(lon0)               0 ] [dx]
    # [ n ] = [ -sin(lat0)*cos(lon0)   -sin(lat0)*sin(lon0)    cos(lat0)] [dy]
    # [ u ]   [  cos(lat0)*cos(lon0)    cos(lat0)*sin(lon0)    sin(lat0)] [dz]
    e = -sin_lon0 * dx + cos_lon0 * dy
    n = (-sin_lat0 * cos_lon0) * dx + (-sin_lat0 * sin_lon0) * dy + cos_lat0 * dz
    u = (cos_lat0 * cos_lon0) * dx + (cos_lat0 * sin_lon0) * dy + sin_lat0 * dz

    return e, n, u


# ==============================
#  STEP 3: ENU -> CARLA
# ==============================
def enu_to_carla(e: float, n: float, u: float,
                 transform: CarlaTransform) -> Tuple[float, float, float]:
    """
    Apply the CARLA left-handed mapping described in the TENA→CARLA notes:
        +TENA x (East)  -> +CARLA y
        +TENA y (North) -> +CARLA x
        +TENA z (Up)    -> +CARLA z
    Then add the translation offsets that anchor the ENU origin in CARLA space.
    """
    carla_x = transform.offset_x + n     # north  → forward
    carla_y = transform.offset_y + e     # east   → right
    carla_z = transform.offset_z + u     # up     → up
    return carla_x, carla_y, carla_z

# ================================
# ENU Orientation to CARLA
# ================================
def enu_orientation_to_carla(roll_deg: float, pitch_deg: float, yaw_deg: float) -> Tuple[float, float, float]:
    carla_roll  = -roll_deg               # roll flips
    carla_pitch =  pitch_deg              # pitch stays
    carla_yaw   = 90.0 - yaw_deg          # yaw becomes 90° - ENU yaw
    return carla_roll, carla_pitch, carla_yaw

# ==============================
#  MAIN: GPS -> CARLA
# ==============================
ORIGIN = GpsOrigin(0.00841723681631434, 0.00773563408642701, 0)
# TRANSFORM = CarlaTransform(carla_origin['x'], carla_origin['y'], carla_origin['z'])
TRANSFORM = CarlaTransform(0, 0, 0)

def gps_to_carla(lat_deg: float,
                 lon_deg: float,
                 h_m: float) -> Tuple[float, float, float]:
    """
    Full pipeline: GPS (lat, lon, h) -> ECEF -> ENU -> CARLA.
    """
    X, Y, Z = geodetic_to_ecef(lat_deg, lon_deg, h_m)
    e, n, u = ecef_to_enu(X, Y, Z, ORIGIN)
    return enu_to_carla(e, n, u, TRANSFORM)


R = 6378137.0  # Earth radius [m]

def bsm_to_enu(lat_bsm, lon_bsm, lat_bsm_ref, lon_bsm_ref):
    # degrees -> radians
    lat  = math.radians(lat_bsm)
    lon  = math.radians(lon_bsm)
    lat0 = math.radians(lat_bsm_ref)
    lon0 = math.radians(lon_bsm_ref)

    dlat = lat - lat0
    dlon = lon - lon0

    # Local ENU approximation around the BSM frame origin
    north = dlat * R
    east  = dlon * R * math.cos(lat0)

    return east, north

def enu_to_map_latlon(east, north, lat_map_ref, lon_map_ref):
    R = 6378137.0
    lat0 = math.radians(lat_map_ref)
    lon0 = math.radians(lon_map_ref)

    dlat = north / R
    dlon = east  / (R * math.cos(lat0))

    lat = lat0 + dlat
    lon = lon0 + dlon

    return math.degrees(lat), math.degrees(lon)

def bsm_to_real_latlon(lat_bsm, lon_bsm,
                       lat_bsm_ref, lon_bsm_ref,
                       lat_map_ref, lon_map_ref):
    east, north = bsm_to_enu(lat_bsm, lon_bsm,
                             lat_bsm_ref, lon_bsm_ref)
    lat_real, lon_real = enu_to_map_latlon(east, north,
                                           lat_map_ref, lon_map_ref)
    return lat_real, lon_real

def haversine(lat1, lon1, lat2, lon2):
    # convert degrees to radians
    lat1, lon1, lat2, lon2 = map(math.radians, [lat1, lon1, lat2, lon2])

    dlat = lat2 - lat1
    dlon = lon2 - lon1

    a = math.sin(dlat/2)**2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon/2)**2
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))

    return R * c

def _normalize_latlon(value, is_lat=True):
    """Normalize latitude/longitude to degrees from various integer scalings."""
    if value is None:
        return None
    try:
        v = float(value)
    except (TypeError, ValueError):
        return None
    limit = 90.0 if is_lat else 180.0
    if abs(v) <= limit:
        return v
    for scale in (1e7, 1e6, 1e5, 1e4, 1e3, 10.0):
        candidate = v / scale
        if abs(candidate) <= limit:
            return candidate
    return v

def distance_real_latlon(latlongBSM, latlongMAP):
    lat1, lon1 = latlongBSM
    lat2, lon2 = latlongMAP
    lat1 = _normalize_latlon(lat1, is_lat=True)
    lon1 = _normalize_latlon(lon1, is_lat=False)
    lat2 = _normalize_latlon(lat2, is_lat=True)
    lon2 = _normalize_latlon(lon2, is_lat=False)
    lat_real1, lon_real1 = bsm_to_real_latlon(lat1, lon1,
                                               ORIGIN.lat_deg, ORIGIN.lon_deg,
                                               lat2, lon2)
    return haversine(lat_real1, lon_real1, lat2, lon2), (lat_real1, lon_real1)
