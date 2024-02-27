import glob
import os
import sys
import re
import argparse

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
    help='Import file to read SignID output')
args = argparser.parse_args()

try:
    client = carla.Client(args.host, args.port)
    client.set_timeout(5.0)
    world = client.get_world()
    # map = world.get_map()

    # Get actor information (Traffic Lights)
    traffic_light_list = world.get_actors().filter('traffic.*')
    print(f'Found {len(traffic_light_list)} Traffic Lights')

    if args.file:
        print("Reading Actor ID and SignID association from: " + str(args.file))        
        signid_list = {}
        with open(args.file) as signid_file:
            for line in signid_file:
                # print(line)
                if line.startswith('Sign ID: '):
                    signid_search = re.search('Sign ID: (.*) hex:',line)
                    signid_search_result = signid_search.group(1)
                    # print(str(signid_search_result))
                elif line.startswith('Actor ID: '):
                    actorid_search = re.search('Actor ID: (.*)',line)
                    actorid_search_result = actorid_search.group(1)
                    # print(str(actorid_search_result))
                    signid_list[actorid_search_result] = signid_search_result

        print("{Actor ID : SignID ...}: " + str(signid_list))
        
        for index, light in enumerate(traffic_light_list, start=1):
            print(f'{light.id} ({signid_list[str(light.id)]})')
            world.debug.draw_string(
                light.get_location(), 
                f'Actor: {light.id}, sign_id: {signid_list[str(light.id)]}', 
                draw_shadow=False,
                color=carla.Color(r=255, g=0, b=0), life_time=200,
                persistent_lines=True)
    else:



        # Print all index corresponding to all traffic lights in scene (CarlaUE4)
        for index, light in enumerate(traffic_light_list, start=1):
            print(f'Drawing TL: {light}')
            world.debug.draw_string(
                light.get_location(), 
                f'Actor: {light.id}', 
                draw_shadow=False,
                color=carla.Color(r=255, g=0, b=0), life_time=200,
                persistent_lines=True)
        
    ################################################################################################
    # Once you see all index number, you can manually change its states and timimg.
    # Your signal control scripts.
finally:
    print('Cleaning up...')
