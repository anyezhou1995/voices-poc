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
    default=10,
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

def draw_waypoints(world,map,waypoints,draw_arrows,veh_name):   

    sampling_resolution = 0.5
    dao = GlobalRoutePlannerDAO(map, sampling_resolution)
    grp = GlobalRoutePlanner(dao)
    grp.setup()

    route_waypoints = []
    segment_endpoints = []

    for i_sp in range(1,len(waypoints)):
        
        start_point = waypoints[i_sp-1]
        end_point = waypoints[i_sp]

        print(f"\nSegment {i_sp}")

        print("Start Point XYZ: " + str(start_point))
        start_point_geo = map.transform_to_geolocation(start_point)
        print("Start Point Lat/Long: " + str(start_point_geo))
        print("End Point XYZ: " + str(end_point))
        end_point_geo = map.transform_to_geolocation(end_point)
        print("End Point Lat/Long: " + str(end_point_geo))
        
        segment_waypoints = grp.trace_route(start_point, end_point) # there are other funcations can be used to generate a route in GlobalRoutePlanner.

        num_segment_waypoints = len(segment_waypoints)
        print(f"Added {num_segment_waypoints} points")

        route_waypoints = route_waypoints + segment_waypoints

        if i_sp != (len(waypoints) - 1):
            segment_endpoints.append(len(route_waypoints))
            # print(f'segment_endpoints: {segment_endpoints}')
        


    waypoint_data = {
        "index" : [],
        "x" : [],
        "y" : [],
        "z" : [],
        "heading" : [],
        "latitude" : [],
        "longitude" : [],
        "altitude" : [],
        "is_segment_endpoint" : [],
    }
    
    draw_arrow_size = 0.2
    draw_arrow_thickness = 0.2
    draw_arrow_z_offset = carla.Location(0,0,1)

    car_length = 3
    car_width = 2

    for i,waypoint in enumerate(route_waypoints):
        if draw_arrows:
            if i == 0:
                start_box_center = route_waypoints[i][0].transform.location + draw_arrow_z_offset

                start_box = carla.BoundingBox(start_box_center,carla.Vector3D(car_length,car_width,0))

                world.debug.draw_box(
                    start_box, 
                    route_waypoints[i][0].transform.rotation,
                    0.2,
                    # draw_shadow=False,
                    color=carla.Color(r=0, g=255, b=0), 
                    life_time=drawing_lifetime,
                    persistent_lines=True)
                
                world.debug.draw_string(start_box_center,"        " + veh_name + ' START', draw_shadow=False,color = carla.Color(r=0, g=255, b=0), life_time=drawing_lifetime,persistent_lines=True)

            elif i == (len(route_waypoints) -1):
                end_box_center = route_waypoints[i][0].transform.location + draw_arrow_z_offset

                end_box = carla.BoundingBox(end_box_center,carla.Vector3D(car_length,car_width,0))

                world.debug.draw_box(
                    end_box, 
                    route_waypoints[i][0].transform.rotation,
                    0.2,
                    # draw_shadow=False,
                    color=carla.Color(r=255, g=0, b=0), 
                    life_time=drawing_lifetime,
                    persistent_lines=True)
            
                world.debug.draw_string(end_box_center,"             " +  veh_name + ' END', draw_shadow=False,color = carla.Color(r=255, g=0, b=0), life_time=drawing_lifetime,persistent_lines=True)

            elif i in segment_endpoints:
                mid_box_center = route_waypoints[i][0].transform.location + draw_arrow_z_offset

                mid_box = carla.BoundingBox(mid_box_center,carla.Vector3D(car_length/2,car_width/2,0))
                this_color = carla.Color(r=255, g=50, b=0)

                world.debug.draw_box(
                    mid_box, 
                    route_waypoints[i][0].transform.rotation,
                    0.2,
                    # draw_shadow=False,
                    color=this_color, 
                    life_time=drawing_lifetime,
                    persistent_lines=True)
            
                world.debug.draw_string(mid_box_center,"             " + ' MID', draw_shadow=False,color=this_color, life_time=drawing_lifetime,persistent_lines=True)
            
            elif i % 12 == 0:
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

        waypoint_heading = waypoint[0].transform.rotation.yaw
        if waypoint_heading < 0:
            waypoint_heading = waypoint_heading + 360
            
        waypoint_data["heading"].append(waypoint_heading)

        w_geo = map.transform_to_geolocation(waypoint[0].transform.location)

        waypoint_data["latitude"].append(w_geo.latitude)
        waypoint_data["longitude"].append(w_geo.longitude)
        waypoint_data["altitude"].append(w_geo.altitude)


    return waypoint_data

try:
    client = carla.Client(args.host, args.port)
    client.set_timeout(5.0)
    world = client.get_world()
    map = world.get_map()

    event2_spawn = {
            "veh_in_order" : ["MCITY","FHWA", "ORNL", "ANL", "UCLA"],
            "wp_btwn_veh" : 5,
            "waypoints" : [
                carla.Location(x=26.685774, y=129.308929, z=232.633194), # start
                carla.Location(x=59.296265, y=67.242165, z=235.812256),
                carla.Location(x=99.357674, y=-83.244278, z=242.828964), # end
            ],
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

            
            # end_point = carla.Location(x=99.357674, y=-83.244278, z=242.828964)

            carlaVehicles = world.get_actors().filter('vehicle.*')
            for vehicle in carlaVehicles:
                currentAttributes = vehicle.attributes
                # print("Checking vehicle: " + str(currentAttributes["role_name"]))
                if currentAttributes["role_name"] == args.follow_vehicle:
                    player = vehicle
            if not player:
                print("ERROR: Unable to find vehicle with rolename: " + args.follow_vehicle)
                sys.exit()

            event2_spawn["waypoints"][0] = player.get_location()
            
            # try:
            draw_waypoints(world,map,event2_spawn["waypoints"],True,"")
            # except Exception as errMsg:
            #     print("UNABLE TO FIND ROUTE")
            #     print(errMsg)

            time.sleep(draw_loop_sleep)
    else:
        
        if args.export:
            current_directory = os.getcwd()
            folder_path = os.path.join(current_directory, "waypoint_files")
            if not os.path.exists(folder_path):
                os.makedirs(folder_path)

        

        waypoint_data = draw_waypoints(world,map,event2_spawn["waypoints"],True,"")
        num_waypoints = len(waypoint_data["x"])
        print("num_waypoints: " + str(num_waypoints))
        new_event2_spawns = []

        for i_v,veh_name in enumerate(event2_spawn["veh_in_order"]):
            
            start_waypoint_index = 0 + (vehicle_wp_spacing * i_v)
            end_waypoint_index = (num_waypoints -1) - (vehicle_wp_spacing * (len(event2_spawn["veh_in_order"]) - 1 - i_v))

            print(veh_name + " start_waypoint_index: " + str(start_waypoint_index))
            print(veh_name + " end_waypoint_index: " + str(end_waypoint_index))

            this_spawn = {
                "name" : veh_name,
                "line_order" : i_v,
                "waypoints" : []
            }

            for i_sp,this_waypoint in enumerate(event2_spawn["waypoints"]):
                if i_sp == 0:
                    new_waypoint = carla.Location(
                        x=waypoint_data["x"][start_waypoint_index], 
                        y=waypoint_data["y"][start_waypoint_index], 
                        z=waypoint_data["z"][start_waypoint_index]
                    )

                elif i_sp == (len(event2_spawn["waypoints"]) -1):
                    new_waypoint = carla.Location(
                        x=waypoint_data["x"][end_waypoint_index], 
                        y=waypoint_data["y"][end_waypoint_index], 
                        z=waypoint_data["z"][end_waypoint_index]
                    )
                
                else:
                    new_waypoint = this_waypoint

                this_spawn["waypoints"].append(new_waypoint)
                

            print(f'this_spawn: {this_spawn}')    


            

            new_event2_spawns.append(this_spawn)


        for test_spawn in new_event2_spawns:
            # world.debug.draw_string(test_spawn["spawn_point"], "o", draw_shadow=False,color = carla.Color(r=255, g=255, b=0), life_time=drawing_lifetime,persistent_lines=True)
            print("\nDrawing: " + test_spawn["name"])
            # world.debug.draw_string(test_spawn["spawn_point"], "     " + test_spawn["name"], draw_shadow=False,color = carla.Color(r=255, g=255, b=0), life_time=drawing_lifetime,persistent_lines=True)
            waypoint_data = draw_waypoints(world,map,test_spawn["waypoints"],True,test_spawn["name"])

            if args.export:
                df = pd.DataFrame(waypoint_data)
                df.to_csv("waypoint_files/" + test_spawn["name"] + '_waypoints.csv', index=False)

            time.sleep(draw_loop_sleep)

finally:
    print('\nDone!')
