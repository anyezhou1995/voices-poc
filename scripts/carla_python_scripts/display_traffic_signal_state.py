import glob
import os
import sys
import time

from find_carla_egg import find_carla_egg

carla_egg_file = find_carla_egg()

sys.path.append(carla_egg_file)

import carla

import argparse

argparser = argparse.ArgumentParser(
    description=__doc__)
argparser.add_argument(
    '--host',
    metavar='<hostname>',
    default='127.0.0.1',
    help='IP of the host server (default: 127.0.0.1)')
argparser.add_argument(
    '-p', '--port',
    metavar='<port>',
    default=2000,
    type=int,
    help='TCP port to listen to (default: 2000)')
argparser.add_argument(
    '--filterw',
    metavar='PATTERN',
    default='walker.pedestrian.*',
    help='pedestrians filter (default: "walker.pedestrian.*")')
argparser.add_argument(
    '--sync',
    action='store_true',
    help='Synchronous mode execution')
argparser.add_argument(
    '-d', '--duration',
    metavar='<duration in s>',
    default=10,
    type=int,
    help='duration to display signal state - use 0 for indefinite (default: 10)')
argparser.add_argument(
    '-v', '--verbose',
    default=False,
    action='store_true',
    help='display actor details each iteration (default: false)')

args = argparser.parse_args()


try:
    client = carla.Client(args.host, args.port)
    client.set_timeout(5.0)
    
    print('\n----- DISPLAYING TRAFFIC SIGNAL STATE -----\n')

    while (True):
        world = client.get_world()
        # Get actor information (Signals)
        signal_list = world.get_actors().filter('traffic.*')
        # Print all index corresponding to all traffic signals in scene (CarlaUE4)

        

        if args.duration == 0:
            label_duration = 0.5
        else:
            label_duration = args.duration
        
        
        if len(signal_list) == 0:

            if args.verbose:
                print("    NO TRAFFIC SIGNALS")

        else:
            if args.verbose:
                print("\nTRAFFIC SIGNALS: ")

            for index, signal in enumerate(signal_list, start=1):
                
                if args.verbose:
                    print("    " + str(signal.attributes))

                signal_state = signal.state
                signal_state_display = str(signal_state).upper()

                if signal_state_display == "OFF":
                    continue


                if signal_state == carla.TrafficLightState.Green:
                    signal_color = carla.Color(r=0, g=255, b=0)
                elif signal_state == carla.TrafficLightState.Red:
                    signal_color = carla.Color(r=255, g=0, b=0)
                elif signal_state == carla.TrafficLightState.Yellow:
                    signal_color = carla.Color(r=255, g=255, b=0)
                else:
                    signal_color = carla.Color(r=0, g=0, b=255)

                signal_draw_loc = signal.get_location() + carla.Location(x=0, y=0, z=10)

                
                
                # box_center = signal_draw_loc + carla.Location(x=0, y=0.5, z=0)

                # box = carla.BoundingBox(box_center,carla.Vector3D(0.1,0.2,0))

                # world.debug.draw_box(
                #     box, 
                #     carla.Rotation(0,0,0),
                #     0.5,
                #     # draw_shadow=False,
                #     color=carla.Color(r=0, g=0, b=0), 
                #     life_time=label_duration,
                #     persistent_lines=True)
                
                world.debug.draw_string(
                    signal_draw_loc, 
                    str(signal_state_display), 
                    draw_shadow=False,
                    color=signal_color, 
                    life_time=label_duration,
                    persistent_lines=True)
            
        if args.duration != 0:
            sys.exit()

        time.sleep(0.5)

except KeyboardInterrupt:
        print('\nCancelled by user. Bye!')

except Exception as err_msg:
    print(str(err_msg))
    print("\nERROR CONNECTING TO CARLA")



    ################################################################################################
    # Once you see all index number, you can manually change its states and timimg.
    # Your signal control scripts.
