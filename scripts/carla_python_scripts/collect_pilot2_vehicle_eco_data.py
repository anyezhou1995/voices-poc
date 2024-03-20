import glob
import os
import sys
import time
import math
import argparse
import pandas as pd

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
    '-d', '--display',
    action='store_true',
    help='displays location of waypoint used for waypoint pitch')
argparser.add_argument(
    '--vehicle_rolenames', '-v',
    required=True,
    help='Comma separated list of vehicles to collect data for ( ex: VEH-1,VEH-2,VEH-3 )')
args = argparser.parse_args()


def check_for_vehicles(vehicle_rolename_actor_map,vehicle_list):
    # print("\nChecking for vehicles")

    vehicle_actor_list = world.get_actors().filter('vehicle.*')

    for vehicle_actor in vehicle_actor_list:
        # print(f"Checking {vehicle_actor}")

        if vehicle_actor.attributes["role_name"] in vehicle_list:
            # print(f'Found vehicle: {vehicle_actor.attributes["role_name"]}')
            vehicle_rolename_actor_map[vehicle_actor.attributes["role_name"]]["vehicle_actor"] = vehicle_actor

    return vehicle_rolename_actor_map

def draw_box_at_point(waypoint):
    draw_z_offset = carla.Location(0,0,2.5)

    start_box_center = waypoint.transform.location + draw_z_offset

    start_box = carla.BoundingBox(start_box_center,carla.Vector3D(0.2,0.2,0))

    world.debug.draw_box(
        start_box, 
        waypoint.transform.rotation,
        0.2,
        # draw_shadow=False,
        color=carla.Color(r=0, g=255, b=0), 
        life_time=0.1,
        persistent_lines=True)

def get_road_grade(start_point,end_point,mid_point):

    # print(f'mid_point: {mid_point}')
    
    run = math.sqrt((end_point.x - start_point.x)**2 + (end_point.y - start_point.y)**2 )
    # print(f'\nrun: {run}')
    rise = end_point.z - start_point.z
    # print(f'rise: {rise}')

    if run == 0:
        grade = 0
    else:
        grade = rise/run*100

    draw_arrow_size = 0.2
    draw_arrow_thickness = 0.2
    draw_arrow_z_offset = carla.Location(0,0,0)

    # world.debug.draw_arrow(
    #     mid_point + draw_arrow_z_offset, 
    #     mid_point + mid_point[0].transform.get_forward_vector() + draw_arrow_z_offset,
    #     thickness=draw_arrow_thickness, 
    #     arrow_size=draw_arrow_size, 
    #     color=carla.Color(r=0, g=50, b=255), 
    #     life_time=drawing_lifetime)
    
    # world.debug.draw_string(mid_point,str(grade), draw_shadow=False,color = carla.Color(r=0, g=255, b=0), life_time=0.1,persistent_lines=True)


    # print(f'grade: {grade}')
    return grade

def get_next_prev_waypoint_grade(waypoint):
    next_waypoint_list = waypoint.next(0.5)
    # print(f'Next Waypoints: {len(next_waypoint_list)}')
    
    prev_waypoint_list = waypoint.previous(0.5)
    # print(f'Prev Waypoints: {len(prev_waypoint_list)}')

    if len(next_waypoint_list) > 0 and len(prev_waypoint_list) > 0:
        next_prev_waypoint_grade = get_road_grade(prev_waypoint_list[0].transform.location, next_waypoint_list[0].transform.location, waypoint.transform.location)
    else:
        next_prev_waypoint_grade = "NA"

    this_eco_data['next_prev_waypoint_grade'].append(next_prev_waypoint_grade)

    if args.display:
        draw_box_at_point(waypoint)
        draw_box_at_point(next_waypoint_list[0])
        draw_box_at_point(prev_waypoint_list[0])


try:
    client = carla.Client(args.host, args.port)
    client.set_timeout(5.0)
    world = client.get_world()
    map = world.get_map()
    # Get actor information (Vehicles)
    
    # Print all index corresponding to all traffic vehicles in scene (CarlaUE4)

    current_directory = os.getcwd()
    eco_data_folder = os.getenv("VUG_LOG_FILES_ROOT") + "/pilot2_eco_data"
    eco_data_folder_path = os.path.join(current_directory, eco_data_folder)
    if not os.path.exists(eco_data_folder_path):
        os.makedirs(eco_data_folder_path)

    timestr = time.strftime("%Y%m%d-%H%M%S")

    vehicle_list = args.vehicle_rolenames.replace(" ","").split(",")
            
    index = 0

    vehicle_rolename_actor_map = {}

    eco_data_base = {
                "index" : [],
                "current_time" : [],
                "vehicle_type" : [],
                "x" : [],
                "y" : [],
                "z" : [],
                "speed" : [],
                "vehicle_pitch" : [],
                "waypoint_pitch" : [],
                # "next_prev_waypoint_grade" : [],
            }

    for vehicle_rolename in vehicle_list:

        vehicle_rolename_actor_map[vehicle_rolename] = {}

        vehicle_rolename_actor_map[vehicle_rolename]["vehicle_actor"] = None
        vehicle_rolename_actor_map[vehicle_rolename]["filename"] = str(timestr) + "_" + str(vehicle_rolename) + "_eco_data.csv"

        # write headers
        df = pd.DataFrame(eco_data_base)
        df.to_csv(eco_data_folder_path + "/" + vehicle_rolename_actor_map[vehicle_rolename]["filename"],header=True,index=False)
        

    
    found_all_veh = False

    while(True):
        

        if not found_all_veh:
            vehicle_rolename_actor_map = check_for_vehicles(vehicle_rolename_actor_map,vehicle_list)
            # print(f'vehicle_rolename_actor_map: {vehicle_rolename_actor_map}')

            # check if we found all vehicles, if we did, we can stop checking (and save get_world hits to API)
            for vehicle_rolename in vehicle_rolename_actor_map:
                if vehicle_rolename_actor_map[vehicle_rolename]["vehicle_actor"] == None:
                    found_all_veh = False
                    break
                else:
                    found_all_veh = True

            if found_all_veh:
                print("\nFOUND ALL COLLECTION VEHICLES")



        for vehicle_rolename in vehicle_rolename_actor_map:

            this_eco_data = {
                "index" : [],
                "current_time" : [],
                "vehicle_type" : [],
                "x" : [],
                "y" : [],
                "z" : [],
                "speed" : [],
                "vehicle_pitch" : [],
                "waypoint_pitch" : [],
                # "next_prev_waypoint_grade" : [],
            }

            # print(f'\nGetting data for {vehicle_rolename}')

            this_veh_actor = vehicle_rolename_actor_map[vehicle_rolename]["vehicle_actor"]
            
            if this_veh_actor == None:
                
                for eco_data_field in this_eco_data:
                    this_eco_data[eco_data_field] = [""]
                
                this_eco_data['index'] = [index]
                this_eco_data['current_time'] = [time.time()]

            else:

                this_eco_data['index'].append(index)
                
                current_time = time.time()
                # print(f'time: {current_time}')
                this_eco_data['current_time'].append(current_time)
                
                this_eco_data['vehicle_type'].append(this_veh_actor.type_id)

                cur_trans = this_veh_actor.get_transform()

                this_eco_data['x'].append(cur_trans.location.x)
                this_eco_data['y'].append(cur_trans.location.y)
                this_eco_data['z'].append(cur_trans.location.z)

                # print(str(cur_trans))
                vehicle_pitch = cur_trans.rotation.pitch
                while vehicle_pitch < -180:
                    vehicle_pitch += 360

                while vehicle_pitch > 180:
                    vehicle_pitch -= 360
                # print(f'vehicle_pitch: {vehicle_pitch}')
                this_eco_data['vehicle_pitch'].append(vehicle_pitch)


                closest_waypoint = map.get_waypoint(this_veh_actor.get_location(),project_to_road=True, lane_type=(carla.LaneType.Driving))
                waypoint_pitch = closest_waypoint.transform.rotation.pitch
                while waypoint_pitch < -180:
                    waypoint_pitch += 360

                while waypoint_pitch > 180:
                    waypoint_pitch -= 360
                # print(f'waypoint_pitch: {waypoint_pitch}')
                this_eco_data['waypoint_pitch'].append(waypoint_pitch)

                # get_next_prev_waypoint_grade(closest_waypoint)

                v = this_veh_actor.get_velocity()
                speed = (3.6 * math.sqrt
                (v.x ** 2 + v.y ** 2 + v.z ** 2))
                # print(f'speed: {speed}')
                this_eco_data['speed'].append(speed)


            df = pd.DataFrame(this_eco_data)

            df.to_csv(eco_data_folder_path + "/" + vehicle_rolename_actor_map[vehicle_rolename]["filename"],mode='a',header=False,index=False)

        index += 1
        time.sleep(0.1)

finally:
    print('Done, stopping collection')