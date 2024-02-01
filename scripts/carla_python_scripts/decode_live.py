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
    
    # print("mcity_origin: " + str(mcity_origin))

    numSpatPhases = 31 #use one more than desired phases

    UDP_IP = "10.7.108.81"
    UDP_PORT = 5398

    sock = socket.socket(   socket.AF_INET, # Internet
                            socket.SOCK_DGRAM) # UDP
    sock.bind((UDP_IP, UDP_PORT))

    while True:
        data, addr = sock.recvfrom(4096) # buffer size is 1024 bytes
        hex_data = data.hex()
        # print("received message: %s" % hex_data)
        '''
        if hex_data.startswith("0013"):
            print("Received SPaT")
            decoded_msg = J2735.DSRC.MessageFrame
            decoded_msg.from_uper(ba.unhexlify(hex_data))
            # decoded_spat = decoded_msg.to_json()
            decoded_spat = decoded_msg()

            spatPhaseArray = [""] * numSpatPhases
            intersectionID = decoded_spat['value'][1]['intersections'][0]['id']['id']
            try:
                intersectionName = decoded_spat['value'][1]['intersections'][0]['name']
            except:
                intersectionName = ""
            spatTimestamp = decoded_spat['value'][1]['intersections'][0]['timeStamp']
            instersectionPhaseArray = decoded_spat['value'][1]['intersections'][0]['states']
            
            for phase in range(len(instersectionPhaseArray)):
                currentPhase = decoded_spat['value'][1]['intersections'][0]['states'][phase].get('signalGroup')
                currentState = str(decoded_spat['value'][1]['intersections'][0]['states'][phase]['state-time-speed'][0]['eventState'])
                spatPhaseArray[currentPhase] = currentState
            
            spatRowList = [str(spatTimestamp),str(intersectionID),intersectionName]
            for printPhase in range(1,numSpatPhases):
                spatRowList.append(spatPhaseArray[printPhase])

            print("Decoded SPaT: ")
            # print(decoded_spat['value'][1]['intersections'][0]['name'])
            #print(str(spatRowList) + "\n")
            print(decoded_msg.to_json())
        '''
        
        '''
        if hex_data.startswith("0014"):
            print("Received BSM")
            decoded_msg = J2735.DSRC.MessageFrame
            decoded_msg.from_uper(ba.unhexlify(hex_data))
            # decoded_bsm = decoded_msg.to_json()
            decoded_bsm = decoded_msg()

            bsmId = decoded_bsm['value'][1]['coreData']['id']
            lat= decoded_bsm['value'][1]['coreData']['lat']
            longstr = decoded_bsm['value'][1]['coreData']['long']
            speed = decoded_bsm['value'][1]['coreData']['speed']
            elevation = decoded_bsm['value'][1]['coreData']['elev']
            secMark = decoded_bsm['value'][1]['coreData']['secMark']
            heading = decoded_bsm['value'][1]['coreData']['heading']
            speed_converted = speed*0.02 #m/s
            accel_long = decoded_bsm['value'][1]['coreData']['accelSet']['long']
            accel_long_converted = accel_long*0.01 #m^s^2

            print("Decoded BSM: ")
            print("Lat: " + str(lat) + ", " + "Long: " + str(longstr) + ", " + "Speed: " + str(speed_converted) + ", " + \
                "Heading: " + str(heading) + ", " + "Accel: " + str(accel_long_converted) + "\n")

            print(decoded_msg.to_json())

        '''

        if hex_data.startswith("0012"):
            print("Received MAP")
            decoded_msg = J2735.DSRC.MessageFrame
            decoded_msg.from_uper(ba.unhexlify(hex_data))
            decoded_map = decoded_msg()
            print("Decoded MAP: ")
            print(decoded_msg.to_json())
        
finally:
    print('\nDone!')