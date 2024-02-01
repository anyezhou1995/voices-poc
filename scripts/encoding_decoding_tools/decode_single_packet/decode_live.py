#SAE J275 DECODER
import J2735_201603_combined_voices_mr_fix as J2735
import socket
import json
import csv
import binascii as ba
from time import sleep
import readline
import os
import sys
import re
import argparse
import math

from scripts.carla_python_scripts.find_carla_egg import find_carla_egg

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

try:
    client = carla.Client(args.host, args.port)
    client.set_timeout(5.0)
    world = client.get_world()
    # map = world.get_map()

    draw_z_height = 237.5
    draw_lifetime = 0.2

    mcity_origin = { 
                "x": 518508.658, 
                "y": -4696054.02, 
                "z": 0
            }
    
    print("mcity_origin: " + str(mcity_origin))

    UDP_IP = "192.168.1.90"
    UDP_PORT = 5399

    sock = socket.socket(   socket.AF_INET, # Internet
                            socket.SOCK_DGRAM) # UDP
    sock.bind((UDP_IP, UDP_PORT))

    while True:
        data, addr = sock.recvfrom(4096) # buffer size is 1024 bytes
        hex_data = data.hex()
        # print("received message: %s" % hex_data)

        if hex_data.startswith("0013"):
            print("Received SPaT")
            decoded_msg = J2735.DSRC.MessageFrame
            decoded_msg.from_uper(ba.unhexlify(hex_data))
            decoded_spat = decoded_msg.to_json()
            print("Decoded SDSM: \n")
            print(decoded_spat)

finally:
    print('\nDone!')