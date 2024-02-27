import os
import sys
import re
import argparse
import json
import math

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
args = argparser.parse_args()


try:
    client = carla.Client(args.host, args.port)
    client.set_timeout(5.0)
    world = client.get_world()
    map = world.get_map()

    draw_z_height = 237
    draw_lifetime = 200

    print("All current vehicle locations")
    landmarks = map.get_all_landmarks()
    for landmark in landmarks:
        print(landmark)
        print("attributes: " + str(landmark.id))
        print("vehicle transform: " + str(landmark.transform))

        world.debug.draw_string(
                landmark.transform.location, 
                f'Landmark: {landmark.id}', 
                draw_shadow=False,
                color=carla.Color(r=255, g=0, b=0), life_time=draw_lifetime,
                persistent_lines=True)
        # try:
        print(f'TrafficLight: {world.get_traffic_light(landmark)}')
        # except:
            # print(f'Error getting traffic light')



finally:
    print('\nDone!')
