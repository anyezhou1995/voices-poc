import os
import sys
import re
import argparse
import json
import math
import time
import pandas as pd



from find_carla_egg import find_carla_egg

carla_egg_file = find_carla_egg()

sys.path.append(carla_egg_file)

import carla

from agents.navigation.global_route_planner import GlobalRoutePlanner
from agents.navigation.global_route_planner_dao import GlobalRoutePlannerDAO

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
    '-l', '--lifetime',
    default=20,
    type=int,
    help='Number of seconds to display each route (default: 10)')
argparser.add_argument(
    '-e', '--export',
    action='store_true',
    help='export waypoints to file (creates waypoint_files dir)')
argparser.add_argument(
    '-o', '--overlay',
    action='store_true',
    help='overlay all routes on top of each other')
argparser.add_argument(
    '--follow_vehicle',
    help='Vehicle to be used for the follow cam (default: "TFHRC-MANUAL-1"')
args = argparser.parse_args()

def draw_waypoints(world, map, start_point, end_point, draw_arrows, veh_name):   

    sampling_resolution = 0.5
    dao = GlobalRoutePlannerDAO(map, sampling_resolution)
    grp = GlobalRoutePlanner(dao)
    grp.setup()
    
    print("\nStart Point XYZ: " + str(start_point))
    start_point_geo = map.transform_to_geolocation(start_point)
    print("Start Point Lat/Long: " + str(start_point_geo))
    print("End Point XYZ: " + str(end_point))
    end_point_geo = map.transform_to_geolocation(end_point)
    print("End Point Lat/Long: " + str(end_point_geo))
    
    route_waypoints = grp.trace_route(start_point, end_point) # there are other funcations can be used to generate a route in GlobalRoutePlanner.
    i = 0

    waypoint_data = {
        "index" : [],
        "x" : [],
        "y" : [],
        "z" : [],
        "latitude" : [],
        "longitude" : [],
        "altitude" : [],
    }
    
    draw_arrow_size = 0.2
    draw_arrow_thickness = 0.3
    draw_arrow_z_offset = carla.Location(0,0,1)

    for waypoint in route_waypoints:
        if draw_arrows:
            if i % 12 == 0:
                world.debug.draw_arrow(
                    waypoint[0].transform.location + draw_arrow_z_offset, 
                    waypoint[0].transform.location + waypoint[0].transform.get_forward_vector() + draw_arrow_z_offset,
                    thickness=draw_arrow_thickness, 
                    arrow_size=draw_arrow_size, 
                    color=carla.Color(r=0, g=0, b=255), 
                    life_time=drawing_lifetime)
                # world.debug.draw_string(waypoint[0].transform.location, 'O', draw_shadow=False,color=carla.Color(r=0, g=0, b=255), life_time=drawing_lifetime,persistent_lines=True)
            elif i % 3 == 0:
                world.debug.draw_arrow(
                    waypoint[0].transform.location + draw_arrow_z_offset, 
                    waypoint[0].transform.location + waypoint[0].transform.get_forward_vector() + draw_arrow_z_offset,
                    thickness=draw_arrow_thickness, 
                    arrow_size=draw_arrow_size, 
                    color=carla.Color(r=0, g=50, b=255), 
                    life_time=drawing_lifetime)
                # world.debug.draw_string(waypoint[0].transform.location, 'O', draw_shadow=False,color = carla.Color(r=0, g=50, b=255), life_time=drawing_lifetime,persistent_lines=True)

        waypoint_data["index"].append(i)
        waypoint_data["x"].append(waypoint[0].transform.location.x)
        waypoint_data["y"].append(waypoint[0].transform.location.y)
        waypoint_data["z"].append(waypoint[0].transform.location.z)

        w_geo = map.transform_to_geolocation(waypoint[0].transform.location)

        waypoint_data["latitude"].append(w_geo.latitude)
        waypoint_data["longitude"].append(w_geo.longitude)
        waypoint_data["altitude"].append(w_geo.altitude)
                
        i += 1

    car_length = 3
    car_width = 2
    
    if draw_arrows:

        start_box_center = route_waypoints[0][0].transform.location + draw_arrow_z_offset

        start_box = carla.BoundingBox(start_box_center,carla.Vector3D(car_length,car_width,0))

        world.debug.draw_box(
            start_box, 
            route_waypoints[0][0].transform.rotation,
            0.2,
            # draw_shadow=False,
            color=carla.Color(r=0, g=255, b=0), 
            life_time=drawing_lifetime,
            persistent_lines=True)
        
        end_box_center = route_waypoints[-1][0].transform.location + draw_arrow_z_offset

        end_box = carla.BoundingBox(end_box_center,carla.Vector3D(car_length,car_width,0))

        world.debug.draw_box(
            end_box, 
            route_waypoints[-1][0].transform.rotation,
            0.2,
            # draw_shadow=False,
            color=carla.Color(r=255, g=0, b=0), 
            life_time=drawing_lifetime,
            persistent_lines=True)
    
        world.debug.draw_string(start_point, "        " + veh_name + ' START', draw_shadow=False,color = carla.Color(r=0, g=255, b=0), life_time=drawing_lifetime,persistent_lines=True)
        world.debug.draw_string(end_point,"             " +  veh_name + ' END', draw_shadow=False,color = carla.Color(r=255, g=0, b=0), life_time=drawing_lifetime,persistent_lines=True)

    return waypoint_data

try:
    client = carla.Client(args.host, args.port)
    client.set_timeout(5.0)
    world = client.get_world()
    map = world.get_map()

    # event2_spawns = [
    #     {
    #         "name" : "FHWA",
    #         "line_order" : 1,
    #         "spawn_point" : carla.Location(x=59.296265, y=67.242165, z=235.812256),
    #         "end_point" : carla.Location(x=99.357674, y=-83.244278, z=242.828964),
    #     },
    #     {
    #         "name" : "ORNL",
    #         "line_order" : 2,
    #         "spawn_point" : carla.Location(x=57.231377, y=59.069706, z=236.117538),
    #         "end_point" : carla.Location(x=99.357674, y=-83.244278, z=242.828964),
    #     },
    #     {
    #         "name" : "ANL",
    #         "line_order" : 3,
    #         "spawn_point" : carla.Location(x=56.112572, y=52.548759, z=236.292847),
    #         "end_point" : carla.Location(x=99.357674, y=-83.244278, z=242.828964),
    #     },
    #     {
    #         "name" : "UCLA",
    #         "line_order" : 4,
    #         "spawn_point" : carla.Location(x=55.125175, y=45.391933, z=236.519577),
    #         "end_point" : carla.Location(x=99.357674, y=-83.244278, z=242.828964),
    #     },
    # ]

    event2_spawn = {
            "veh_in_order" : ["ORNL"],
            "wp_btwn_veh" : 5,
            "start_point" : carla.Location(x=57.231377, y=59.069706, z=236.117538),
            #"end_point" : carla.Location(x=99.357674, y=-83.244278, z=242.828964),
            "end_point" : carla.Location(x=208.813, y=-153.486, z=242.258),
            #"end_point" : carla.Location(x=146.522, y=-2.166, z=237.762)
    }

    drawing_lifetime = args.lifetime
    draw_loop_sleep = args.lifetime

    if args.overlay:
        draw_loop_sleep = 0


    
    vehicle_wp_spacing = 20


    if args.follow_vehicle:
        while True:
            world = client.get_world()
            map = world.get_map() 

            end_point = carla.Location(x=99.357674, y=-83.244278, z=242.828964)

            carlaVehicles = world.get_actors().filter('vehicle.*')
            for vehicle in carlaVehicles:
                currentAttributes = vehicle.attributes
                print("Checking vehicle: " + str(currentAttributes["role_name"]))
                if currentAttributes["role_name"] == args.follow_vehicle:
                    player = vehicle
            if not player:
                print("ERROR: Unable to find vehicle with rolename: " + args.follow_vehicle)
                sys.exit()
            
            try:
                draw_waypoints(world,map,player.get_location(),end_point,"")
            except Exception as errMsg:
                print("UNABLE TO FIND ROUTE")
                print(errMsg)

            time.sleep(draw_loop_sleep)
    else:
        
        if args.export:
            current_directory = os.getcwd()
            folder_path = os.path.join(current_directory, "waypoint_files")
            if not os.path.exists(folder_path):
                os.makedirs(folder_path)

        

        waypoint_data = draw_waypoints(world,map,event2_spawn["start_point"],event2_spawn["end_point"],False,"")
        num_waypoints = len(waypoint_data["x"])
        print("num_waypoints: " + str(num_waypoints))
        event2_spawns = []

        for i_v,veh_name in enumerate(event2_spawn["veh_in_order"]):
            
            start_waypoint_index = 0 + (vehicle_wp_spacing * i_v)
            end_waypoint_index = (num_waypoints -1) - (vehicle_wp_spacing * (len(event2_spawn["veh_in_order"]) - 1 - i_v))

            print(veh_name + " start_waypoint_index: " + str(start_waypoint_index))
            print(veh_name + " end_waypoint_index: " + str(end_waypoint_index))

            this_spawn = {
                "name" : veh_name,
                "line_order" : i_v,
                "spawn_point" : carla.Location(
                    x=waypoint_data["x"][start_waypoint_index], 
                    y=waypoint_data["y"][start_waypoint_index], 
                    z=waypoint_data["z"][start_waypoint_index]
                    ) ,
                "end_point" : carla.Location(
                    x=waypoint_data["x"][end_waypoint_index], 
                    y=waypoint_data["y"][end_waypoint_index], 
                    z=waypoint_data["z"][end_waypoint_index] 
                    ), 
            }

            event2_spawns.append(this_spawn)


        for test_spawn in event2_spawns:
            # world.debug.draw_string(test_spawn["spawn_point"], "o", draw_shadow=False,color = carla.Color(r=255, g=255, b=0), life_time=drawing_lifetime,persistent_lines=True)
            print("\nDrawing: " + test_spawn["name"])
            # world.debug.draw_string(test_spawn["spawn_point"], "     " + test_spawn["name"], draw_shadow=False,color = carla.Color(r=255, g=255, b=0), life_time=drawing_lifetime,persistent_lines=True)
            waypoint_data = draw_waypoints(world,map,test_spawn["spawn_point"],test_spawn["end_point"],True,test_spawn["name"])

            if args.export:
                df = pd.DataFrame(waypoint_data)
                df.to_csv("waypoint_files/" + test_spawn["name"] + '_waypoints_outloop.csv', index=False)

            time.sleep(draw_loop_sleep)

finally:
    print('\nDone!')
