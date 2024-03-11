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

from carla import VehicleLightState as vls

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
        '-e', '--exclude',
        metavar='VEHICLE_NAME',
        type=str,
        help='Exclude these vehicles when deleting (comma separated, Ex: VW-MAN-1,ECON-MAN-1)')
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
        
        #world = client.load_world('/Game/Carla/Maps/Carla_v14_10_1_2021')
        #world = client.get_world()
        
        #client = carla.Client()
        #client.set_timeout(10.0)
        already_stopped_once = []

        if args.exclude:
            vehicles_to_exclude = (args.exclude).split(",")
        else:
            vehicles_to_exclude = []

        max_checks = 60


        for i in range(max_checks):

            vehicles = world.get_actors().filter('vehicle.*')

            stopped_vehicles = []

            print(f"Checking for new vehicles to stop [{max_checks - i}]")

            for vehicle in vehicles:
                
                if vehicle.attributes["role_name"] in vehicles_to_exclude:
                    print("\tSkipping: " + str(vehicle.attributes["role_name"]))
                    continue
                elif vehicle.attributes["role_name"] in already_stopped_once:
                    print("\tAlready stopped: " + str(vehicle.attributes["role_name"]))
                    continue
                
                print(f'\tStopping vehicle: {vehicle.attributes["role_name"]}')
                # print("attributes: " + str(vehicle.attributes))
                # print("location: " + str(vehicle.get_location()))
                veh_control = carla.VehicleControl()
                veh_control.hand_brake = True
                vehicle.apply_control(veh_control)

                stopped_vehicles.append(vehicle)
                already_stopped_once.append(vehicle.attributes["role_name"])


            if len(stopped_vehicles) > 0:
                time.sleep(5)

                for vehicle in stopped_vehicles:

                    veh_control = carla.VehicleControl()
                    veh_control.hand_brake = False
                    vehicle.apply_control(veh_control)

            time.sleep(1)

    finally:

        if args.sync and synchronous_master:
            settings = world.get_settings()
            settings.synchronous_mode = False
            settings.fixed_delta_seconds = None
            world.apply_settings(settings)

        print('\nENDING')


        time.sleep(0.5)

if __name__ == '__main__':

    try:
        main()
    except KeyboardInterrupt:
        pass
    finally:
        print('\ndone.')
