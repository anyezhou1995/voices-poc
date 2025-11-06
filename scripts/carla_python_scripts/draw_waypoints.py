import os
import sys
import re
import argparse
import json
import math
import time
import pandas as pd
import numpy as np

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


# Colors
red    = carla.Color(255,   0,   0)
green  = carla.Color(  0, 255,   0)
blue   = carla.Color( 47, 210, 231)
cyan   = carla.Color(  0, 255, 255)
yellow = carla.Color(255, 255,   0)
orange = carla.Color(255, 162,   0)
white  = carla.Color(255, 255, 255)

waypoint_separation = 0.2

waypoint_size = 0.1
drawing_lifetime = args.lifetime

draw_arrow_size = 0.2
draw_arrow_thickness = 0.2
draw_arrow_z_offset = carla.Location(0,0,0)

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

        spawn_data["waypoints"].append(follow_vehicle.get_location())
    elif key == Key.delete:
        print("Removing waypoint")
        follow_vehicle = get_veh_with_name(args.follow_vehicle)

        if len(spawn_data["waypoints"]) > 0:
            spawn_data["waypoints"].pop()
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

    # world.debug.draw_arrow(
    #     mid_point[0].transform.location + draw_arrow_z_offset, 
    #     mid_point[0].transform.location + mid_point[0].transform.get_forward_vector() + draw_arrow_z_offset,
    #     thickness=draw_arrow_thickness, 
    #     arrow_size=draw_arrow_size, 
    #     color=carla.Color(r=0, g=50, b=255), 
    #     life_time=drawing_lifetime)
    
    world.debug.draw_string(mid_point[0].transform.location,str(grade), draw_shadow=False,color = carla.Color(r=0, g=255, b=0), life_time=drawing_lifetime,persistent_lines=True)


    print(f'grade: {grade}')

def draw_waypoint_info(debug, w, lt=drawing_lifetime, x_offset=0,draw_data=False):
    w_loc = w.transform.location
    # debug.draw_point(w_loc, waypoint_size, red, lt)
    world.debug.draw_arrow(
                    w.transform.location + draw_arrow_z_offset, 
                    w.transform.location + w.transform.get_forward_vector() + draw_arrow_z_offset,
                    thickness=draw_arrow_thickness, 
                    arrow_size=draw_arrow_size, 
                    color=red, 
                    life_time=drawing_lifetime)
    if draw_data:
        debug.draw_string(w_loc + carla.Location(x=x_offset,z=0.5),  f"lane: {w.lane_id}", False, yellow, lt)
        debug.draw_string(w_loc + carla.Location(x=x_offset,z=1.0),  f"road: {w.road_id}", False, cyan,   lt)
        debug.draw_string(w_loc + carla.Location(x=x_offset,z=1.5), f"lc: {w.lane_change}",    False, red,    lt)
        debug.draw_string(w_loc + carla.Location(x=x_offset,z=2), f"lt: {w.lane_type}",    False, red,    lt)

def draw_waypoint_union(debug, w0, w1, color=green, lt=drawing_lifetime):
    # debug.draw_line(
    #     w0.transform.location + carla.Location(z=0.25),
    #     w1.transform.location + carla.Location(z=0.25),
    #     thickness=0.1, color=color, life_time=lt, persistent_lines=False)
    debug.draw_point(w1.transform.location + carla.Location(z=1), 0.1, color, lt, False)

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
        
        try:
            segment_waypoints = grp.trace_route(start_point, end_point) # there are other funcations can be used to generate a route in GlobalRoutePlanner.
        except Exception as errMsg:
            print(f"Error generating route: {errMsg}")
            segment_waypoints = []

        num_segment_waypoints = len(segment_waypoints)
        print(f"Added {num_segment_waypoints} points")

        route_waypoints = route_waypoints + segment_waypoints

        if i_sp != (len(waypoints) - 1):
            segment_endpoints.append(len(route_waypoints))
            # print(f'segment_endpoints: {segment_endpoints}')
    
    print(f'\n ~~~~~~~~~FINDING SEGMENTS~~~~~~~~~')

    print(f'num route waypoints: {len(route_waypoints)}')

    segment_list = []
    first_waypoint,first_road_options = route_waypoints[0]
    first_segment_end_wp = first_waypoint.next_until_lane_end(0.001)[-1]
    segment_list.append(
        {
            "starting_waypoint": first_waypoint,
            "ending_waypoint": first_segment_end_wp,
            "road_id": first_waypoint.road_id,
            "section_id": first_waypoint.section_id,
            "lane_id": first_waypoint.lane_id,
        }
    )

    for waypoint,road_option in route_waypoints:
        # print(f'waypoint id: {waypoint.id} option: {road_option} road: {waypoint.road_id} section: {waypoint.section_id} lane_id: {waypoint.lane_id}' )
            
        if (    segment_list[-1]["road_id"] == waypoint.road_id and 
                segment_list[-1]["section_id"] == waypoint.section_id and 
                segment_list[-1]["lane_id"] == waypoint.lane_id  
        ):
            continue
        else:
            print(f'finished current segment, found first wp of next')
            current_segment_end_wp = waypoint.next_until_lane_end(0.1)[-1]
            print("adding segment: ")
            segment_list.append(
                {
                    "starting_waypoint": waypoint,
                    "ending_waypoint": current_segment_end_wp,
                    "road_id": waypoint.road_id,
                    "section_id": waypoint.section_id,
                    "lane_id": waypoint.lane_id,
                }
            )
    debug = world.debug

    final_segment_list = []
    roads_to_exclude = [131,281,382,328]
    for segment in segment_list:
        
        if segment["ending_waypoint"].road_id in roads_to_exclude:
            print("Skipping segment as road is excluded (usually bike or similar)")
            continue
        
        draw_waypoint_union(debug,segment["starting_waypoint"],segment["ending_waypoint"],green)
        draw_waypoint_info(debug,segment["starting_waypoint"])
        draw_waypoint_info(debug,segment["ending_waypoint"],x_offset=1)
        final_segment_list.append(segment)


    for segment in final_segment_list:
        print(f'start road: {segment["starting_waypoint"].road_id}')

    
    final_waypoints = []

    for i_seg, segment in enumerate(final_segment_list):
        final_waypoints.append(segment["starting_waypoint"])
        cur_wp = final_waypoints[-1]
        print(f'i_seg: {i_seg} road: {segment["starting_waypoint"].road_id}')

        reached_end = False
        while True:
            next_wp = list(cur_wp.next(waypoint_separation))
            if len(next_wp) == 0:
                print(f'no next wp found')
                reached_end = True
                break
            elif len(next_wp) == 1:
                next_wp = next_wp[0]
                # print(f'progressing down road: {next_wp.road_id}')
            else: 
                print(f'found fork...')
                for fork_wp in next_wp:
                    print(f'fork_wp.road_id: {fork_wp.road_id} final_segment_list road: {final_segment_list[i_seg + 1]["ending_waypoint"].road_id}')
                    if fork_wp.road_id == final_segment_list[i_seg + 1]["ending_waypoint"].road_id:
                        print(f'found next road in fork: {fork_wp.road_id}')
                        next_wp = fork_wp
                        break

            # print(f'next_wp: {next_wp.id}')
            if next_wp.road_id != cur_wp.road_id:
                print(f'reached end of road')
                break

            draw_waypoint_info(debug,cur_wp)
            final_waypoints.append(next_wp)
            cur_wp = next_wp

            if reached_end:
                print(f'reached end of route')
                break
            time.sleep(0.001)



    if args.export and veh_name:
        f_c = open(f'waypoint_files/{veh_name}_carma_route', "w")
    
        print("\nCARMA ROUTE:")
        for route_line in carma_route:
            print(route_line)
            f_c.write(f'{route_line}\n')
        
        f_c.close()

        f_g = open(f'waypoint_files/{veh_name}_waypoints.csv', "w")
    
        print("\nGENERAL ROUTE:")
        for route_line in general_route:
            print(route_line)
            f_g.write(f'{route_line}\n')
        
        f_g.close()


    waypoint_data = {
        "index" : [],
        "x" : [],
        "y" : [],
        "z" : [],
        "carla_yaw" : [],
        # "geo_heading" : [],
        "carla_bearing_yaw" : [],
        "roll" : [],
        "latitude" : [],
        "longitude" : [],
        "altitude" : [],
        "road_grade" : [],
        # "is_segment_endpoint" : [],
    }

    car_length = 3
    car_width = 2

    midpoint_count = 0

    for i,waypoint in enumerate(final_waypoints):
        if draw_arrows:
            if i == 0:
                start_box_center = final_waypoints[i].transform.location + draw_arrow_z_offset

                start_box = carla.BoundingBox(start_box_center,carla.Vector3D(car_length,car_width,0))

                world.debug.draw_box(
                    start_box, 
                    final_waypoints[i].transform.rotation,
                    0.2,
                    # draw_shadow=False,
                    color=carla.Color(r=0, g=255, b=0), 
                    life_time=drawing_lifetime,
                    persistent_lines=True)
                
                world.debug.draw_string(start_box_center,"        " + veh_name + ' START', draw_shadow=False,color = green, life_time=drawing_lifetime,persistent_lines=True)

            elif i == (len(final_waypoints) -1):
                end_box_center = final_waypoints[i].transform.location + draw_arrow_z_offset

                end_box = carla.BoundingBox(end_box_center,carla.Vector3D(car_length,car_width,0))

                world.debug.draw_box(
                    end_box, 
                    final_waypoints[i].transform.rotation,
                    0.2,
                    # draw_shadow=False,
                    color=red, 
                    life_time=drawing_lifetime,
                    persistent_lines=True)
            
                world.debug.draw_string(end_box_center,"             " +  veh_name + ' END', draw_shadow=False,color = red, life_time=drawing_lifetime,persistent_lines=True)

            elif i in segment_endpoints:
                mid_box_center = final_waypoints[i].transform.location + draw_arrow_z_offset

                mid_box = carla.BoundingBox(mid_box_center,carla.Vector3D(car_length/2,car_width/2,0))
                this_color = carla.Color(r=255, g=50, b=0)

                world.debug.draw_box(
                    mid_box, 
                    final_waypoints[i].transform.rotation,
                    0.2,
                    # draw_shadow=False,
                    color=this_color, 
                    life_time=drawing_lifetime,
                    persistent_lines=True)

                midpoint_count += 1
                world.debug.draw_string(mid_box_center,"" + 'MID_' + str(midpoint_count), draw_shadow=False,color=this_color, life_time=drawing_lifetime,persistent_lines=True)
            
            elif i % 12 == 0:
                world.debug.draw_arrow(
                    waypoint.transform.location + draw_arrow_z_offset, 
                    waypoint.transform.location + waypoint.transform.get_forward_vector() + draw_arrow_z_offset,
                    thickness=draw_arrow_thickness, 
                    arrow_size=draw_arrow_size, 
                    color=blue, 
                    life_time=drawing_lifetime)
                # world.debug.draw_string(waypoint.transform.location, 'O', draw_shadow=False,color=carla.Color(r=0, g=0, b=255), life_time=drawing_lifetime,persistent_lines=True)
            elif i % 3 == 0:
                world.debug.draw_arrow(
                    waypoint.transform.location + draw_arrow_z_offset, 
                    waypoint.transform.location + waypoint.transform.get_forward_vector() + draw_arrow_z_offset,
                    thickness=draw_arrow_thickness, 
                    arrow_size=draw_arrow_size, 
                    color=carla.Color(r=0, g=50, b=255), 
                    life_time=drawing_lifetime)
                # world.debug.draw_string(waypoint.transform.location, str(i), draw_shadow=False,color = carla.Color(r=0, g=50, b=255), life_time=drawing_lifetime,persistent_lines=True)

        waypoint_data["index"].append(i)
        waypoint_data["x"].append(waypoint.transform.location.x) # we swap x and y at the end for export to ltpENU
        waypoint_data["y"].append(waypoint.transform.location.y)
        waypoint_data["z"].append(waypoint.transform.location.z)

        # CARLA yaw (deg): 0 points along +X, +90 along +Y (Unreal coords: X forward, Y right, Z up)
        carla_yaw = waypoint.transform.rotation.yaw % 360
        waypoint_data["carla_yaw"].append(carla_yaw)
        
        # Geographic bearing (deg): 0 is North (+Y if you treat Y as North), 90 is East (+X), 180 South, 270 West
        # geo_heading = (90 - waypoint.transform.rotation.yaw ) % 360.0# we apply the coord transform (90 - yaw) % 360 at the end
        # waypoint_data["geo_heading"].append(geo_heading)

        # also try calculating bearing using the next waypoint
        # we apply the coord transform (90 - yaw) % 360 at the end
        if i < len(final_waypoints) - 1:
            next_waypoint = final_waypoints[i+1]
            dx = next_waypoint.transform.location.x - waypoint.transform.location.x
            dy = next_waypoint.transform.location.y - waypoint.transform.location.y
            if dx == 0 and dy == 0:
                 # if we didnt move, use the previous yaw
                 carla_bearing_yaw = waypoint_data["carla_bearing_yaw"][-1]  # degenerate
            else:
                # theta from +X (East), CCW
                carla_bearing_yaw = math.degrees(math.atan2(dy, dx))
        else:
            # if this is the last point, just use the prev value
            carla_bearing_yaw = waypoint_data["carla_bearing_yaw"][-1]

        

        waypoint_data["carla_bearing_yaw"].append(carla_bearing_yaw) # this is transformed at the end

        waypoint_data["roll"].append(0)
        waypoint_data["road_grade"].append(waypoint.transform.rotation.pitch)

        w_geo = map.transform_to_geolocation(waypoint.transform.location)

        waypoint_data["latitude"].append(w_geo.latitude)
        waypoint_data["longitude"].append(w_geo.longitude)
        waypoint_data["altitude"].append(w_geo.altitude)


        # if i > 0 and i < len(final_waypoints) - 1:
        #     get_road_grade(final_waypoints[i-1],final_waypoints[i+1],final_waypoints[i])



    return waypoint_data

def add_linear_distance(df):
    """
    Adds segment and cumulative linear distances in meters
    based on x,y,z columns (CARLA world coordinates).
    Assumes df is ordered in travel direction.
    """
    # Extract numpy arrays for speed
    coords = df[['x', 'y', 'z']].to_numpy()

    # Differences between successive rows (shifted by 1)
    deltas = coords[1:] - coords[:-1]

    # Euclidean distances
    seg_dist = np.linalg.norm(deltas, axis=1)

    # First waypoint has zero distance traveled from start
    seg_dist = np.insert(seg_dist, 0, 0.0)

    df['segment_distance_m'] = seg_dist
    df['distance_traveled_m'] = seg_dist.cumsum()

    return df

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
    # event2_spawn = {
    #         "veh_in_order" : ["MCITY","FHWA", "ORNL", "ANL", "UCLA"],
    #         "wp_btwn_veh" : 5,
    #         "waypoints" : [
    #             carla.Location(x=149.985428, y=-228.668350, z=244),          # start
    #             carla.Location(x=109.453369, y=-60.563061, z=238.563339),   # mid 1
    #             carla.Location(x=100.363297, y=63.378155, z=236.299454),    # mid 2
    #             carla.Location(x=55.186951, y=42.167976, z=236.767197),     # mid 3
    #             carla.Location(x=56.355728, y=-10.193906, z=237.101028),    # mid 4
    #             carla.Location(x=57.891472, y=-66.892288, z=238.003296),    # mid 5
    #             # carla.Location(x=78.490326, y=-120.598251, z=240.014816),   # mid 6
    #             carla.Location(104.506683, y=-130.960526, z=241.668213),    # end
    #         ],
    # }
    energy_campaign = {
            "veh_in_order" : ["UCLA"],
            "wp_btwn_veh" : 5,
            "waypoints" : [
                carla.Location(x=-754.107971, y=730.503052, z=1.324147), 
                carla.Location(x=-421.867340, y=764.293213, z=0.899224), 
                carla.Location(x=1.540522, y=720.684692, z=1.235085), 
                carla.Location(x=313.938751, y=726.293762, z=0.711762), 
                carla.Location(x=521.184143, y=845.688660, z=1.005151), 
                carla.Location(x=621.049255, y=832.310791, z=1.037112), 
            ],
    }

    spawn_data = energy_campaign

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
        
        spawn_data["waypoints"] = []

        while True:
            world = client.get_world()
            map = world.get_map() 

            follow_vehicle = get_veh_with_name(args.follow_vehicle)
            

            
            if len(spawn_data["waypoints"]) > 0:
                # add vehicle as final dest
                spawn_data["waypoints"].append(follow_vehicle.get_location())
                try:
                    draw_waypoints(world,map,spawn_data["waypoints"],True,"")
                except Exception as errMsg:
                    print("UNABLE TO FIND ROUTE")
                    print(errMsg)
                spawn_data["waypoints"].pop()
            else:
                print("No waypoints added. Add a new waypoint by pressing SPACE")

            

            time.sleep(draw_loop_sleep)
    else:
        
        waypoint_data = draw_waypoints(world,map,spawn_data["waypoints"],False,"")
        num_waypoints = len(waypoint_data["x"])
        print("num_waypoints: " + str(num_waypoints))
        new_spawns = []

        for i_v,veh_name in enumerate(spawn_data["veh_in_order"]):
            
            start_waypoint_index = 0 + (start_vehicle_wp_spacing * i_v)
            end_waypoint_index = (num_waypoints -1) - (end_vehicle_wp_spacing * (len(spawn_data["veh_in_order"]) - 1 - i_v))

            print(veh_name + " start_waypoint_index: " + str(start_waypoint_index))
            print(veh_name + " end_waypoint_index: " + str(end_waypoint_index))

            this_spawn = {
                "name" : veh_name,
                "line_order" : i_v,
                "waypoints" : []
            }

            for i_sp,this_waypoint in enumerate(spawn_data["waypoints"]):
                if i_sp == 0:
                    new_waypoint = carla.Location(
                        x=waypoint_data["x"][start_waypoint_index], 
                        y=waypoint_data["y"][start_waypoint_index], 
                        z=waypoint_data["z"][start_waypoint_index]
                    )

                elif i_sp == (len(spawn_data["waypoints"]) -1):
                    new_waypoint = carla.Location(
                        x=waypoint_data["x"][end_waypoint_index], 
                        y=waypoint_data["y"][end_waypoint_index], 
                        z=waypoint_data["z"][end_waypoint_index]
                    )
                
                else:
                    new_waypoint = this_waypoint

                this_spawn["waypoints"].append(new_waypoint)
                
            print(f'this_spawn: {this_spawn}')    

            new_spawns.append(this_spawn)


        for test_spawn in new_spawns:
            # world.debug.draw_string(test_spawn["spawn_point"], "o", draw_shadow=False,color = carla.Color(r=255, g=255, b=0), life_time=drawing_lifetime,persistent_lines=True)
            print("\nDrawing: " + test_spawn["name"])
            # world.debug.draw_string(test_spawn["spawn_point"], "     " + test_spawn["name"], draw_shadow=False,color = carla.Color(r=255, g=255, b=0), life_time=drawing_lifetime,persistent_lines=True)
            waypoint_data = draw_waypoints(world,map,test_spawn["waypoints"],True,test_spawn["name"])

            if args.export:
                df = pd.DataFrame(waypoint_data)
                df = add_linear_distance(df)
                df = df.rename(columns={"y": "y_tmp"})
                df = df.rename(columns={"x": "y"})
                df = df.rename(columns={"y_tmp": "x"})

                df["ltpENU_yaw"] = (90.0 - df["carla_yaw"]) % 360.0
                df["ltpENU_bearing_yaw"] = (90.0 - df["carla_bearing_yaw"]) % 360.0

                # df["y"] = -df["y_carla"]
                df.to_csv("waypoint_files/" + test_spawn["name"] + '_breadcrumbs.csv', index=False)

            time.sleep(draw_loop_sleep)

finally:
    print('\nDone!')