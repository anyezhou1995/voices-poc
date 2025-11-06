import os
import sys
import re
import argparse
import json
import math
import time

from find_carla_egg import find_carla_egg

carla_egg_file = find_carla_egg()

sys.path.append(carla_egg_file)

import carla

argparser = argparse.ArgumentParser(
    description=__doc__)
argparser.add_argument(
    '--host',
    metavar='H',
    default='127.0.0.1',
    help='IP of the host server (default: 127.0.0.1)')
argparser.add_argument(
    '-p', '--port',
    metavar='P',
    default=2000,
    type=int,
    help='TCP port to listen to (default: 2000)')
argparser.add_argument(
    '-f', '--file',
    metavar='f',
    type=str,
    help='Import file to read crossing data')
args = argparser.parse_args()


def list_json_file():
    json_files = [file for file in os.listdir() if file.endswith('.json')]
    if not json_files:
        print("No .json files found in the current directory.")
        return None
    else:
        print("\nAvailable .json files:\n")
        for i, file in enumerate(json_files, 1):
            print(f"\t{i}. {file}")
        return json_files

def load_selected_json_file(selected_file):
    try:
        with open(selected_file, 'r') as file:
            data = json.load(file)
        return data
    except FileNotFoundError:
        print(f"Error: File not found at '{selected_file}'")
        return None
    except json.JSONDecodeError:
        print(f"Error: Unable to decode JSON from file '{selected_file}'")
        return None

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

def draw_world_axes(world,
                    origin=carla.Location(0.0, 0.0, 0.0),
                    length=10.0,
                    thickness=0.2,
                    arrow_size=0.8,
                    life_time=0.0,
                    persistent=False):
    """
    Draw arrows from 'origin' along +X, +Y, +Z in CARLA.
    Axis colors: X=red, Y=green, Z=blue.
    """
    dbg = world.debug

    end_x = carla.Location(origin.x + length, origin.y, origin.z)  # +X (North)
    end_y = carla.Location(origin.x, origin.y + length, origin.z)  # +Y (East)
    end_z = carla.Location(origin.x, origin.y, origin.z + length)  # +Z (Up)

    # Signature: draw_arrow(begin, end, thickness, arrow_size, color, life_time=-1.0, persistent_lines=True)
    dbg.draw_arrow(origin, end_x, thickness, arrow_size, carla.Color(255, 0, 0), float(life_time), persistent)  # X (red)
    dbg.draw_arrow(origin, end_y, thickness, arrow_size, carla.Color(0, 255, 0), float(life_time), persistent)  # Y (green)
    dbg.draw_arrow(origin, end_z, thickness, arrow_size, carla.Color(0, 0, 255), float(life_time), persistent)  # Z (blue)

    # draw_string signature varies slightly by version; this form is widely compatible:
    # draw_string(location, text, draw_shadow=False, color=Color(), life_time=-1.0, persistent_lines=True)
    dbg.draw_string(end_x, " +X", False, carla.Color(255, 0, 0), float(life_time), persistent)
    dbg.draw_string(end_y, " +Y", False, carla.Color(0, 255, 0), float(life_time), persistent)
    dbg.draw_string(end_z, " +Z", False, carla.Color(0, 0, 255), float(life_time), persistent)

def follow_vehicle_axes(world, role_name, length=5.0, life_time=0.5, offset_z=2.0):
    """
    Continuously draw X/Y/Z axes at the vehicle's position.
    The draw frequency matches the life_time, so arrows refresh smoothly.

    Args:
        world: carla.World
        role_name: vehicle's role_name attribute to track
        length: arrow shaft length in meters
        life_time: seconds each arrow stays visible (also the update rate)
        offset_z: vertical offset above the vehicle roof
    """
    dbg = world.debug
    actors = world.get_actors().filter('vehicle.*')
    vehicle = next((a for a in actors if a.attributes.get('role_name') == role_name), None)
    if vehicle is None:
        print(f"Vehicle with role_name '{role_name}' not found.")
        return

    print(f"Tracking vehicle '{role_name}'... Press Ctrl+C to stop.")
    try:
        while True:
            transform = vehicle.get_transform()
            origin = transform.location + carla.Location(z=offset_z)

            # use your existing draw_world_axes()
            draw_world_axes(world,
                            origin=origin,
                            length=length,
                            life_time=life_time,
                            persistent=False)

            time.sleep(life_time)
    except KeyboardInterrupt:
        print("\nStopped following vehicle axes.")

try:
    client = carla.Client(args.host, args.port)
    client.set_timeout(5.0)
    world = client.get_world()
    # map = world.get_map()

    draw_z_height = 237
    draw_lifetime = 30

    # mcity_origin = GeodeticToEcef(42.30059341574939,-83.69928318881136,0)

    mcity_origin = { 
                "x": 518508.658, 
                "y": -4696054.02, 
                "z": 0
            }
    
    draw_world_axes(world, life_time=30)
    follow_vehicle_axes(world, role_name="FHWA-JSON-3", length=8.0, life_time=0.1)

finally:
    print('\nDone!')
