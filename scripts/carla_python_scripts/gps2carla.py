import math
from dataclasses import dataclass
from typing import Tuple


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
    - yaw_offset_deg: rotation around Up axis (ENU 'U' / CARLA 'Z')
                      from ENU East axis to CARLA X axis (CCW, degrees)
    - offset_x/y/z:   translation from ENU origin to CARLA origin (meters)
                      i.e., CARLA = Rz(yaw) * ENU + offset
    """
    yaw_offset_deg: float
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
    Convert ENU coordinates to CARLA coordinates with a yaw rotation +
    translation.

    yaw_offset_deg: angle from ENU East axis to CARLA X axis (CCW, degrees).
    """
    theta = math.radians(transform.yaw_offset_deg)
    cos_t = math.cos(theta)
    sin_t = math.sin(theta)

    # Rotation about Z (Up) axis
    # [x_c]   [ cosθ  -sinθ  0 ] [e]
    # [y_c] = [ sinθ   cosθ  0 ] [n]
    # [z_c]   [  0      0    1 ] [u]
    x_c = cos_t * e - sin_t * n
    y_c = sin_t * e + cos_t * n
    z_c = u  # still 'Up'; you can also scale/offset if needed

    # Translation to CARLA origin
    x_carla = x_c + transform.offset_x
    y_carla = y_c + transform.offset_y
    z_carla = z_c + transform.offset_z

    return x_carla, y_carla, z_carla


# ==============================
#  MAIN: GPS -> CARLA
# ==============================
def gps_to_carla(lat_deg: float,
                 lon_deg: float,
                 h_m: float,
                 origin: GpsOrigin,
                 transform: CarlaTransform) -> Tuple[float, float, float]:
    """
    Full pipeline: GPS (lat, lon, h) -> ECEF -> ENU -> CARLA.
    """
    X, Y, Z = geodetic_to_ecef(lat_deg, lon_deg, h_m)
    e, n, u = ecef_to_enu(X, Y, Z, origin)
    return enu_to_carla(e, n, u, transform)