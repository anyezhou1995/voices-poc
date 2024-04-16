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
        '-e', '--exclude',
        metavar='VEHICLE_NAME',
        type=str,
        help='Exclude these vehicles when deleting (comma separated, Ex: VW-MAN-1,ECON-MAN-1)')
    argparser.add_argument(
        '-i', '--include',
        metavar='VEHICLE_NAME',
        type=str,
        help='Only delete these vehicles (comma separated, Ex: VW-MAN-1,ECON-MAN-1)')
    args = argparser.parse_args()

    logging.basicConfig(format='%(levelname)s: %(message)s', level=logging.INFO)

    vehicles_list = []
    walkers_list = []
    all_id = []
    client = carla.Client(args.host, args.port)
    client.set_timeout(10.0)

    try:
        world = client.get_world()
        print("Available Maps: " + ', '.join(client.get_available_maps()))
        
        #world = client.load_world('/Game/Carla/Maps/Carla_v14_10_1_2021')
        #world = client.get_world()
        
        #client = carla.Client()
        #client.set_timeout(10.0)
        
        vehicles = world.get_actors().filter('vehicle.*')

        if args.include:
            vehicles_to_include = (args.include).replace(" ","").split(",")
            included_vehicles = []

            for vehicle in vehicles:
                if vehicle.attributes["role_name"] in vehicles_to_include:
                    included_vehicles.append(vehicle)
            
            vehicles = included_vehicles


        if args.exclude:
            vehicles_to_exclude = (args.exclude).replace(" ","").split(",")
        else:
            vehicles_to_exclude = []
        for vehicle in vehicles:
            print(vehicle)
            print("attributes: " + str(vehicle.attributes))
            print("location: " + str(vehicle.get_location()))
            
            if vehicle.attributes["role_name"] in vehicles_to_exclude:
                print("Skipping: " + str(vehicle.attributes["role_name"]))
                continue
            # vehicle.attributes["color"] = "255,255,255"
            vehicle.destroy()


        
        #print(world.get_actors().find(102))
        #print("TrafficLights:" + ', '.join(trafficlights))
        #print(dir(trafficlights[0]))
        #for light in trafficlights:
            #print(light.get_group_traffic_lights())
            
            
        # blueprints = [bp for bp in world.get_blueprint_library().filter('vehicle.*')]
        # for blueprint in blueprints:
        #     print(blueprint.id)
        #     for attr in blueprint:
        #         print('  - {}'.format(attr))

    finally:

        print('\nENDING')


        time.sleep(0.5)

if __name__ == '__main__':

    try:
        main()
    except KeyboardInterrupt:
        pass
    finally:
        print('\ndone.')
