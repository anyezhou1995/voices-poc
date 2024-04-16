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
import pygame

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
    synchronous_master = True
    random.seed(args.seed if args.seed is not None else int(time.time()))

    try:
        world = client.get_world()

        ### Simulation time that goes by between simulation steps ###
        settings = world.get_settings()
        print("\n----- SETTING TIME MODE -----")
        print("fixed_delta_seconds before: " + str(settings.fixed_delta_seconds))
        print("synchronous_mode before: " + str(settings.synchronous_mode))
        settings.synchronous_mode = False # True
        settings.fixed_delta_seconds = None #0.025 #0.035 #0.025 #None # 0
        world.apply_settings(settings)
        ### Simulation time that goes by between simulation steps ###

        settings = world.get_settings()
        print("fixed_delta_seconds after: " + str(settings.fixed_delta_seconds))
        print("synchronous_mode after: " + str(settings.synchronous_mode))

        print('\n----- SUCCESSFULLY SET TIME MODE, CONTINUOUSLY TICKING WORLD -----\n')
        
        # # t_diff = 0.0417

        # while True:
        #     #goal_step = 0.05 #0.025 

        #     # settings = world.get_settings()
        #     # settings.fixed_delta_seconds = t_diff
        #     # world.apply_settings(settings)

        #     # t_prev = world.get_snapshot().timestamp.elapsed_seconds

        #     world.tick()

        #     # t_curr = world.get_snapshot().timestamp.elapsed_seconds
        #     # t_diff = t_curr - t_prev

        #     # print("t_diff: " + str(t_diff))
            
            
        #     #additional_sleep = max(0.0, goal_step - t_diff)

        #     #print("need to sleep: " + str(additional_sleep))
        #     #time.sleep(additional_sleep)
        #     time.sleep(3)

            
        

    finally:

        if synchronous_master:
            print('\nENDING SYNCHRONOUS MODE')
            settings = world.get_settings()
            settings.synchronous_mode = False
            settings.fixed_delta_seconds = None
            world.apply_settings(settings)


        time.sleep(0.5)

if __name__ == '__main__':

    try:
        main()
    except KeyboardInterrupt:
        pass
