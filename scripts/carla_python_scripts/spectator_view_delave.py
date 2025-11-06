#!/usr/bin/env python3

# Copyright (c) 2019 Computer Vision Center (CVC) at the Universitat Autonoma de
# Barcelona (UAB).
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

"""Spawn NPCs into the simulation"""

import glob
import os
import sys
import time

from find_carla_egg import find_carla_egg

carla_egg_file = find_carla_egg()

sys.path.append(carla_egg_file)

import carla

import argparse
import logging
from numpy import random

def main():
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
        '-n', '--number-of-vehicles',
        metavar='N',
        default=10,
        type=int,
        help='number of vehicles (default: 10)')
    argparser.add_argument(
        '-w', '--number-of-walkers',
        metavar='W',
        default=50,
        type=int,
        help='number of walkers (default: 50)')
    argparser.add_argument(
        '--safe',
        action='store_true',
        help='avoid spawning vehicles prone to accidents')
    argparser.add_argument(
        '--filterv',
        metavar='PATTERN',
        default='vehicle.*',
        help='vehicles filter (default: "vehicle.*")')
    argparser.add_argument(
        '--filterw',
        metavar='PATTERN',
        default='walker.pedestrian.*',
        help='pedestrians filter (default: "walker.pedestrian.*")')
    argparser.add_argument(
        '--tm-port',
        metavar='P',
        default=8000,
        type=int,
        help='port to communicate with TM (default: 8000)')
    argparser.add_argument(
        '--sync',
        action='store_true',
        help='Synchronous mode execution')
    argparser.add_argument(
        '--hybrid',
        action='store_true',
        help='Enanble')
    argparser.add_argument(
        '-s', '--seed',
        metavar='S',
        type=int,
        help='Random device seed')
    argparser.add_argument(
        '--car-lights-on',
        action='store_true',
        default=False,
        help='Enanble car lights')
    args = argparser.parse_args()

    logging.basicConfig(format='%(levelname)s: %(message)s', level=logging.INFO)

    vehicles_list = []
    walkers_list = []
    all_id = []
    client = carla.Client(args.host, args.port)
    client.set_timeout(10.0)
    synchronous_master = False
    random.seed(args.seed if args.seed is not None else int(time.time()))

    try:
        world = client.get_world()
        
        # Retrieve the spectator object
        spectator = world.get_spectator()

        # Get the location and rotation of the spectator through its transform
        # spec_transform = spectator.get_transform()

        # print(str(spec_transform))
        spec_location = carla.Location(x=-105.437805, y=726.073425, z=926.507080)
        spec_rotation = carla.Rotation(pitch=-89, yaw=-90, roll=0.000)

        # # Set the spectator with an empty transform
        spectator.set_transform(carla.Transform(spec_location,spec_rotation))

    finally:
        print('\n----- SUCCESSFULLY SET SPECTATOR VIEW -----\n')
        time.sleep(0.5)

if __name__ == '__main__':

    try:
        main()
    except KeyboardInterrupt:
        pass
