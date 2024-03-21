import os
import sys
import re
import argparse
import json
import math
import time
import pandas as pd

from pynput import keyboard
from pynput.keyboard import Key



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

def on_press(key):

    world = client.get_world()
        
    # Retrieve the spectator object
    spectator = world.get_spectator()

    # Get the location and rotation of the spectator through its transform
    spec_transform = spectator.get_transform()

    try:
        key_char = key.char
    except Exception:
        key_char = None

    if key == Key.space:
        print("Adding waypoint")
        follow_vehicle = get_veh_with_name(args.follow_vehicle)

        event2_spawn["waypoints"].append(follow_vehicle.get_location())
    elif key == Key.delete:
        print("Adding waypoint")
        follow_vehicle = get_veh_with_name(args.follow_vehicle)

        if len(event2_spawn["waypoints"]) > 0:
            event2_spawn["waypoints"].pop()
        else:
            print("No waypoints to remove")

def get_veh_with_name(veh_rolename):
    player = None
    carlaVehicles = world.get_actors().filter('vehicle.*')
    for vehicle in carlaVehicles:
        currentAttributes = vehicle.attributes
        # print("Checking vehicle: " + str(currentAttributes["role_name"]))
        if currentAttributes["role_name"] == veh_rolename:
            player = vehicle
    if not player:
        print("ERROR: Unable to find vehicle with rolename: " + veh_rolename)
        sys.exit()
    
    return player

def get_road_grade(start_point,end_point,mid_point):

    print(f'mid_point: {mid_point[0].transform}')
    
    run = math.sqrt((end_point[0].transform.location.x - start_point[0].transform.location.x)**2 + (end_point[0].transform.location.y - start_point[0].transform.location.y)**2 )
    print(f'\nrun: {run}')
    rise = end_point[0].transform.location.z - start_point[0].transform.location.z
    print(f'rise: {rise}')

    if run == 0:
        grade = 0
    else:
        grade = rise/run*100

    draw_arrow_size = 0.2
    draw_arrow_thickness = 0.2
    draw_arrow_z_offset = carla.Location(0,0,0)

    # world.debug.draw_arrow(
    #     mid_point[0].transform.location + draw_arrow_z_offset, 
    #     mid_point[0].transform.location + mid_point[0].transform.get_forward_vector() + draw_arrow_z_offset,
    #     thickness=draw_arrow_thickness, 
    #     arrow_size=draw_arrow_size, 
    #     color=carla.Color(r=0, g=50, b=255), 
    #     life_time=drawing_lifetime)
    
    world.debug.draw_string(mid_point[0].transform.location,str(grade), draw_shadow=False,color = carla.Color(r=0, g=255, b=0), life_time=drawing_lifetime,persistent_lines=True)


    print(f'grade: {grade}')

def draw_waypoints(world,map,waypoints,draw_arrows,veh_name):   

    print("SETTING UP MAP")
    sampling_resolution = 2
    dao = GlobalRoutePlannerDAO(map, sampling_resolution)
    grp = GlobalRoutePlanner(dao)
    grp.setup()
    print("FINISHED SETTING UP MAP")

    route_waypoints = []
    segment_endpoints = []

    carma_route = []
    general_route = []

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
        
        if i_sp == 1:
            general_route.append(f'index,x,y,z,latitide,longitude,altitude')
            general_route.append(f'0,{start_point.x},{start_point.y},{start_point.z},{start_point_geo.latitude},{start_point_geo.longitude},{start_point_geo.altitude}')
            general_route.append(f'{i_sp},{end_point.x},{end_point.y},{end_point.z},{end_point_geo.latitude},{end_point_geo.longitude},{end_point_geo.altitude}')
        else:
            general_route.append(f'{i_sp},{end_point.x},{end_point.y},{end_point.z},{end_point_geo.latitude},{end_point_geo.longitude},{end_point_geo.altitude}')


        if i_sp == len(waypoints)-1:
            carma_route.append(f'{end_point_geo.longitude},{end_point_geo.latitude},0,{veh_name}_route')
        else:
            carma_route.append(f'{end_point_geo.longitude},{end_point_geo.latitude},0,{veh_name}_route_waypoint_{i_sp}')
        
        
        segment_waypoints = grp.trace_route(start_point, end_point) # there are other funcations can be used to generate a route in GlobalRoutePlanner.

        num_segment_waypoints = len(segment_waypoints)
        print(f"Added {num_segment_waypoints} points")

        route_waypoints = route_waypoints + segment_waypoints

        if i_sp != (len(waypoints) - 1):
            segment_endpoints.append(len(route_waypoints))
            # print(f'segment_endpoints: {segment_endpoints}')
    
    if args.export and veh_name:
        f_c = open(f'waypoint_files/{veh_name}_carma_route', "w")
    
        print("\nCARMA ROUTE:")
        for route_line in carma_route:
            print(route_line)
            f_c.write(f'{route_line}\n')
        
        f_c.close()

        f_g = open(f'waypoint_files/{veh_name}_waypoints.csv', "w")
    
        print("\nCARMA ROUTE:")
        for route_line in general_route:
            print(route_line)
            f_g.write(f'{route_line}\n')
        
        f_g.close()


    waypoint_data = {
        "index" : [],
        "x" : [],
        "y" : [],
        "z" : [],
        "carla_heading" : [],
        "geo_heading" : [],
        "latitude" : [],
        "longitude" : [],
        "altitude" : [],
        "road_grade" : [],
        # "is_segment_endpoint" : [],
    }
    
    draw_arrow_size = 0.2
    draw_arrow_thickness = 0.2
    draw_arrow_z_offset = carla.Location(0,0,1)

    car_length = 3
    car_width = 2

    midpoint_count = 0

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

                midpoint_count += 1
                world.debug.draw_string(mid_box_center,"" + 'MID_' + str(midpoint_count), draw_shadow=False,color=this_color, life_time=drawing_lifetime,persistent_lines=True)
            
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
                # world.debug.draw_string(waypoint[0].transform.location, str(i), draw_shadow=False,color = carla.Color(r=0, g=50, b=255), life_time=drawing_lifetime,persistent_lines=True)

        waypoint_data["index"].append(i)
        waypoint_data["x"].append(waypoint[0].transform.location.x)
        waypoint_data["y"].append(waypoint[0].transform.location.y)
        waypoint_data["z"].append(waypoint[0].transform.location.z)

        carla_heading = waypoint[0].transform.rotation.yaw
        while carla_heading < 0:
            carla_heading = carla_heading + 360

        while carla_heading > 360:
            carla_heading = carla_heading - 360
            
        waypoint_data["carla_heading"].append(carla_heading)
        
        geo_heading = carla_heading + 90
        while geo_heading < 0:
            geo_heading = geo_heading + 360

        while geo_heading > 360:
            geo_heading = geo_heading - 360

        waypoint_data["geo_heading"].append(geo_heading)

        w_geo = map.transform_to_geolocation(waypoint[0].transform.location)

        waypoint_data["latitude"].append(w_geo.latitude)
        waypoint_data["longitude"].append(w_geo.longitude)
        waypoint_data["altitude"].append(w_geo.altitude)

        waypoint_data["road_grade"].append(waypoint[0].transform.rotation.pitch)

        # if i > 0 and i < len(route_waypoints) - 1:
        #     get_road_grade(route_waypoints[i-1],route_waypoints[i+1],route_waypoints[i])




    return waypoint_data

try:
    client = carla.Client(args.host, args.port)
    client.set_timeout(5.0)
    world = client.get_world()
    map = world.get_map()

    # orig 
    # event2_spawn = {
    #         "veh_in_order" : ["MCITY","FHWA", "ORNL", "ANL", "UCLA"],
    #         "wp_btwn_veh" : 5,
    #         "waypoints" : [
    #             carla.Location(x=26.685774, y=129.308929, z=232.633194), # start
    #             carla.Location(x=59.296265, y=67.242165, z=235.812256),
    #             carla.Location(x=99.357674, y=-83.244278, z=242.828964), # end
    #         ],
    # }

    #loop
    event2_spawn = {
            "veh_in_order" : ["MCITY","FHWA", "ORNL", "ANL", "UCLA"],
            "wp_btwn_veh" : 5,
            "waypoints" : [
                carla.Location(x=149.985428, y=-228.668350, z=244),          # start
                carla.Location(x=109.453369, y=-60.563061, z=238.563339),   # mid 1
                carla.Location(x=100.363297, y=63.378155, z=236.299454),    # mid 2
                carla.Location(x=55.186951, y=42.167976, z=236.767197),     # mid 3
                carla.Location(x=56.355728, y=-10.193906, z=237.101028),    # mid 4
                carla.Location(x=57.891472, y=-66.892288, z=238.003296),    # mid 5
                # carla.Location(x=78.490326, y=-120.598251, z=240.014816),   # mid 6
                carla.Location(104.506683, y=-130.960526, z=241.668213),    # end
            ],
    }

    drawing_lifetime = args.lifetime
    draw_loop_sleep = args.lifetime

    if args.overlay:
        draw_loop_sleep = 0


    
    start_vehicle_wp_spacing = 5
    end_vehicle_wp_spacing = 7

    if args.export:
        current_directory = os.getcwd()
        folder_path = os.path.join(current_directory, "waypoint_files")
        if not os.path.exists(folder_path):
            os.makedirs(folder_path)

    if args.follow_vehicle:
        listener = keyboard.Listener(on_press=on_press)

        print("Starting keyboard listener")
        listener.start()
        print("Keyboard listener started")
        
        event2_spawn["waypoints"] = []

        while True:
            world = client.get_world()
            map = world.get_map() 

            follow_vehicle = get_veh_with_name(args.follow_vehicle)
            

            
            if len(event2_spawn["waypoints"]) > 0:
                # add vehicle as final dest
                event2_spawn["waypoints"].append(follow_vehicle.get_location())
                # try:
                draw_waypoints(world,map,event2_spawn["waypoints"],True,"")
                # except Exception as errMsg:
                #     print("UNABLE TO FIND ROUTE")
                #     print(errMsg)
                event2_spawn["waypoints"].pop()
            else:
                print("No waypoints added. Add a new waypoint by pressing SPACE")

            

            time.sleep(draw_loop_sleep)
    else:
        
        waypoint_data = draw_waypoints(world,map,event2_spawn["waypoints"],False,"")
        num_waypoints = len(waypoint_data["x"])
        print("num_waypoints: " + str(num_waypoints))
        new_event2_spawns = []

        for i_v,veh_name in enumerate(event2_spawn["veh_in_order"]):
            
            start_waypoint_index = 0 + (start_vehicle_wp_spacing * i_v)
            end_waypoint_index = (num_waypoints -1) - (end_vehicle_wp_spacing * (len(event2_spawn["veh_in_order"]) - 1 - i_v))

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
                df.to_csv("waypoint_files/" + test_spawn["name"] + '_breadcrumbs.csv', index=False)

            time.sleep(draw_loop_sleep)

finally:
    print('\nDone!')