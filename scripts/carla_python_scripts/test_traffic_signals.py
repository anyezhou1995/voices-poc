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

def get_dist_3d(start_point,end_point):
    return ((end_point[0]-start_point[0])**2 + (end_point[1]-start_point[1])**2 + (end_point[2]-start_point[2])**2) ** 0.5
    

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

    vehicles_list = []
    walkers_list = []
    all_id = []
    client = carla.Client(args.host, args.port)
    client.set_timeout(10.0)

    try:
        world = client.get_world()

        # traffic_lights_to_blank_locs = [
        #     [249.8,-163.4,0],
        #     [251.7,-180.3,0],
        #     [262.2,-165.6,0],
        #     [262.5,-175.9,0],

        # ]
        tsc_actors_to_set = []
        carla_actors = world.get_actors().filter("traffic.traffic_light")
        print(f'Setting Signals: {carla_actors}')
        for actor in carla_actors:
            print(f'Setting Signal: {actor}')
            actor_loc = actor.get_location()
            actor_loc_array = [actor_loc.x, actor_loc.y, actor_loc.z]
            
            # -- This for loop can be used to only blank specific signals in the array above
            # -- locations are specified as actor IDs are reset when carla and OpenCDA join CARLA
            # for tl_loc in traffic_lights_to_blank_locs:
                
                # dist_from_target_loc = get_dist_3d(actor_loc_array,tl_loc)
                # if dist_from_target_loc < 5:
            if tsc_actors_to_set == [] or actor.id in tsc_actors_to_set:
                
                actor.set_green_time(5)
                actor.set_yellow_time(2)
                actor.set_red_time(5)
                actor.set_state(carla.TrafficLightState.Green)
            else:
                actor.set_green_time(99999999999)
                actor.set_yellow_time(99999999999)
                actor.set_red_time(99999999999)
                actor.set_state(carla.TrafficLightState.Off)
            
        

    finally:

        print('\n----- SUCCESSFULLY SET SIGNALS -----\n')
        time.sleep(0.5)

if __name__ == '__main__':

    try:
        main()
    except KeyboardInterrupt:
        pass
