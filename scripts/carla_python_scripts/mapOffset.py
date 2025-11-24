import math
from dataclasses import dataclass
from typing import List, Tuple

import numpy as np


# ==============================
#  WGS-84 CONSTANTS
# ==============================
WGS84_A = 6378137.0              # Semi-major axis (m)
WGS84_F = 1 / 298.257223563      # Flattening
WGS84_E2 = WGS84_F * (2 - WGS84_F)


# ==============================
#  DATA CLASSES
# ==============================
@dataclass
class GpsOrigin:
    lat_deg: float
    lon_deg: float
    h_m: float


@dataclass
class CarlaTransform:
    yaw_offset_deg: float   # ENU East -> CARLA X (CCW, degrees)
    offset_x: float         # translation x (m)
    offset_y: float         # translation y (m)
    offset_z: float         # translation z (m)


# ==============================
#  GEO FUNCTIONS
# ==============================
def geodetic_to_ecef(lat_deg: float, lon_deg: float, h_m: float) -> Tuple[float, float, float]:
    lat = math.radians(lat_deg)
    lon = math.radians(lon_deg)

    sin_lat = math.sin(lat)
    cos_lat = math.cos(lat)
    sin_lon = math.sin(lon)
    cos_lon = math.cos(lon)

    N = WGS84_A / math.sqrt(1.0 - WGS84_E2 * sin_lat * sin_lat)

    X = (N + h_m) * cos_lat * cos_lon
    Y = (N + h_m) * cos_lat * sin_lon
    Z = (N * (1.0 - WGS84_E2) + h_m) * sin_lat
    return X, Y, Z


def ecef_to_enu(X: float, Y: float, Z: float, origin: GpsOrigin) -> Tuple[float, float, float]:
    lat0 = math.radians(origin.lat_deg)
    lon0 = math.radians(origin.lon_deg)
    h0   = origin.h_m

    X0, Y0, Z0 = geodetic_to_ecef(origin.lat_deg, origin.lon_deg, h0)

    dx = X - X0
    dy = Y - Y0
    dz = Z - Z0

    sin_lat0 = math.sin(lat0)
    cos_lat0 = math.cos(lat0)
    sin_lon0 = math.sin(lon0)
    cos_lon0 = math.cos(lon0)

    e = -sin_lon0 * dx + cos_lon0 * dy
    n = (-sin_lat0 * cos_lon0) * dx + (-sin_lat0 * sin_lon0) * dy + cos_lat0 * dz
    u = (cos_lat0 * cos_lon0) * dx + (cos_lat0 * sin_lon0) * dy + sin_lat0 * dz

    return e, n, u


# ==============================
#  CALIBRATION
# ==============================
@dataclass
class GpsCarlaPair:
    lat_deg: float
    lon_deg: float
    h_m: float
    x_carla: float
    y_carla: float
    z_carla: float


def calibrate_enu_to_carla(
    gps_carla_pairs: List[GpsCarlaPair],
    origin: GpsOrigin,
) -> CarlaTransform:
    """
    Given multiple GPS ↔ CARLA correspondences, solve for the optimal:
      - yaw_offset_deg (rotation about Z)
      - offset_x, offset_y, offset_z (translation)

    Assumes:
      - ENU East/North/Up vs CARLA X/Y/Z differ only by yaw + translation.
      - All GPS are near the chosen origin (ENU is locally valid).
    """

    if len(gps_carla_pairs) < 2:
        raise ValueError("Need at least 2 pairs to estimate yaw and translation.")

    enu_list = []
    carla_xy_list = []
    enu_u_list = []
    carla_z_list = []

    # 1) Convert all GPS to ENU, collect ENU and CARLA
    for p in gps_carla_pairs:
        X, Y, Z = geodetic_to_ecef(p.lat_deg, p.lon_deg, p.h_m)
        e, n, u = ecef_to_enu(X, Y, Z, origin)

        enu_list.append([e, n])
        carla_xy_list.append([p.x_carla, p.y_carla])
        enu_u_list.append(u)
        carla_z_list.append(p.z_carla)

    E = np.array(enu_list)        # shape (N,2)
    C = np.array(carla_xy_list)   # shape (N,2)
    U = np.array(enu_u_list)      # shape (N,)
    Z = np.array(carla_z_list)    # shape (N,)

    # 2) Compute centroids
    E_mean = E.mean(axis=0)
    C_mean = C.mean(axis=0)

    E_centered = E - E_mean
    C_centered = C - C_mean

    # 3) 2D orthogonal Procrustes: find R (rotation) minimizing ||R E - C||
    #    H = C_centered^T * E_centered
    H = C_centered.T @ E_centered  # shape (2,2)

    U_svd, S_svd, Vt_svd = np.linalg.svd(H)
    R = U_svd @ Vt_svd  # rotation matrix

    # Enforce proper rotation (det(R)=+1)
    if np.linalg.det(R) < 0:
        U_svd[:, -1] *= -1
        R = U_svd @ Vt_svd

    # 4) Extract yaw from R (since R = [[cosθ, -sinθ],[sinθ, cosθ]])
    yaw_rad = math.atan2(R[1, 0], R[0, 0])
    yaw_deg = math.degrees(yaw_rad)

    # 5) Translation in XY: C ≈ R * E + t  =>  t = C_mean - R * E_mean
    t_xy = C_mean - R @ E_mean
    offset_x, offset_y = t_xy.tolist()

    # 6) Translation in Z: assume z_carla = u + tz (no pitch/roll)
    offset_z = float((Z - U).mean())

    return CarlaTransform(
        yaw_offset_deg=yaw_deg,
        offset_x=offset_x,
        offset_y=offset_y,
        offset_z=offset_z,
    )


if __name__ == "__main__":
    # 1. Choose an ENU origin (e.g., near center of your scene)
    origin = GpsOrigin(
        lat_deg=36.123456,
        lon_deg=-86.987654,
        h_m=200.0,
    )

    # 2. Collect multiple correspondences:
    #    (GPS lat, lon, h) <-> (CARLA x, y, z) for the same physical points
    #    Example with dummy numbers:
    pairs = [
        GpsCarlaPair(36.123500, -86.987600, 201.0,  10.0,  5.0, 0.5),
        GpsCarlaPair(36.123550, -86.987550, 201.2,  30.0,  6.0, 0.6),
        GpsCarlaPair(36.123600, -86.987500, 200.8,  50.0,  7.0, 0.4),
        # add as many as you can, ideally well spread in map
    ]

    tf = calibrate_enu_to_carla(pairs, origin)

    print("Calibrated transform:")
    print(f"  yaw_offset_deg = {tf.yaw_offset_deg:.6f}")
    print(f"  offset_x       = {tf.offset_x:.6f} m")
    print(f"  offset_y       = {tf.offset_y:.6f} m")
    print(f"  offset_z       = {tf.offset_z:.6f} m")
