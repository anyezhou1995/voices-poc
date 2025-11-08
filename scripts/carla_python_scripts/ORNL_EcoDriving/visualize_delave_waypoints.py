#!/usr/bin/env python3

"""Utility to visualize waypoints from a CSV file inside CARLA."""

import argparse
import math
import os
import sys
import time

import pandas as pd

from find_carla_egg import find_carla_egg

carla_egg_file = find_carla_egg()
if carla_egg_file not in sys.path:
    sys.path.append(carla_egg_file)

import carla  # noqa: E402


SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
DEFAULT_CSV = os.path.abspath(
    os.path.join(SCRIPT_DIR, os.pardir, os.pardir, 'json_scripts', 'delave_waypoints.csv'))


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--host', default='127.0.0.1', help='CARLA host (default: 127.0.0.1)')
    parser.add_argument('-p', '--port', default=2000, type=int, help='CARLA port (default: 2000)')
    parser.add_argument('--csv', default=DEFAULT_CSV, help='Path to waypoint CSV file')
    parser.add_argument('--lifetime', type=float, default=80.0, help='Seconds debug marks stay alive')
    parser.add_argument('--z-offset', type=float, default=0.5, help='Meters to lift drawings above ground')
    parser.add_argument('--arrow-length', type=float, default=2.0, help='Meters for arrow length')
    parser.add_argument('--arrow-size', type=float, default=0.5, help='Arrow head size')
    parser.add_argument('--arrow-thickness', type=float, default=0.3, help='Arrow shaft thickness')
    parser.add_argument('--point-size', type=float, default=0.5, help='Point marker size when yaw missing')
    parser.add_argument('--wait', type=float, default=120.0, help='Keep script alive for this many seconds')
    return parser.parse_args()


def load_waypoints(csv_path):
    if not os.path.isfile(csv_path):
        raise FileNotFoundError(f'Waypoint CSV not found: {csv_path}')
    df = pd.read_csv(csv_path)
    required = {'x', 'y', 'z'}
    missing = required - set(df.columns)
    if missing:
        raise ValueError(f'missing required columns in CSV: {sorted(missing)}')
    return df


def draw_waypoints(world, df, args):
    debug = world.debug
    color = carla.Color(r=0, g=255, b=255)

    for row in df.itertuples(index=False):
        location = carla.Location(x=float(row.y), y=float(row.x), z=float(row.z + args.z_offset))
        yaw = getattr(row, 'yaw', None)

        if yaw is not None and not math.isnan(yaw):
            rotation = carla.Rotation(yaw=float(yaw))
            forward = rotation.get_forward_vector()
            end_location = carla.Location(
                x=location.x + forward.x * args.arrow_length,
                y=location.y + forward.y * args.arrow_length,
                z=location.z + forward.z * args.arrow_length)
            debug.draw_arrow(
                location,
                end_location,
                thickness=args.arrow_thickness,
                arrow_size=args.arrow_size,
                color=color,
                life_time=args.lifetime)
        else:
            debug.draw_point(location, size=args.point_size, color=color, life_time=args.lifetime)


def main():
    args = parse_args()
    df = load_waypoints(args.csv)
    print(f'Loaded {len(df)} waypoints from {args.csv}')

    client = carla.Client(args.host, args.port)
    client.set_timeout(5.0)
    world = client.get_world()

    draw_waypoints(world, df, args)
    print('Waypoints drawn. Use Ctrl+C to exit early.')

    try:
        time.sleep(args.wait)
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
