#!/usr/bin/env python3

import argparse
from argparse import RawTextHelpFormatter
import random
import sys

import pygame  # for keyboard input

from find_carla_egg import find_carla_egg

# -------------------------------------------------------------------------
# Locate CARLA egg and add CARLA to sys.path
# -------------------------------------------------------------------------

carla_egg_file = find_carla_egg()

# Add egg so `import carla` works
sys.path.append(carla_egg_file)

import carla

from agents.navigation.behavior_agent import BehaviorAgent


# -------------------------------------------------------------------------
# Argument parser
# -------------------------------------------------------------------------

def parse_args():
    parser = argparse.ArgumentParser(
        formatter_class=RawTextHelpFormatter, description=(
            "Drive a CARLA vehicle with a route-based autopilot (destination required).\n\n"
            "How to run inside the distributed-testing container:\n"
            "  1) Enter the container: dt exec\n"
            "  2) Change to the script folder: cd ~/distributed-testing/scripts/carla_python_scripts\n"
            "  3) Run with your options: ./drive_route.py [args]\n\n"
            "Examples:\n"
            "  - Attach to an existing vehicle (role_name \"FHWA-M-1\") and start route:\n"
            "      ./drive_route.py --attach_vehicle FHWA-M-1 --dest 80.0 -10.0 1.0\n"
            "  - Spawn a new vehicle at a fixed pose and start route:\n"
            "      ./drive_route.py --x 12.5 --y -45.0 --z 1.0 --yaw 90 --dest 80.0 -10.0 1.0\n\n"
            "Keyboard controls:\n"
            "  - SPACE: stop/resume target speed\n"
            "  - UP / DOWN: increase/decrease target speed by 5 kph\n"
            "  - E: toggle manual speed limiting (adheres to map speed limit when off)\n"
            "  - T: toggle train mode (agent steers; you handle throttle/brake)\n"
            "  - W / S: throttle/brake when train mode is ON"
        )
    )
    parser.add_argument(
        '--dest',
        metavar=('DX', 'DY', 'DZ'),
        type=float,
        nargs=3,
        required=True,
        help='Destination XYZ for route autopilot (three values required)')
    parser.add_argument(
        '-v', '--verbose',
        action='store_true',
        dest='debug',
        help='Print debug details about CARLA world, vehicle selection, and agent setup')
    parser.add_argument(
        '--host',
        metavar='H',
        default='127.0.0.1',
        help='IP/hostname of the CARLA server reachable from inside the container (default: 127.0.0.1)')
    parser.add_argument(
        '-p', '--port',
        metavar='P',
        default=2000,
        type=int,
        help='TCP port for CARLA server RPC (default: 2000)')
    parser.add_argument(
        '--filter',
        metavar='PATTERN',
        default='vehicle.*',
        help='Blueprint filter for spawned vehicles (default: "vehicle.*")')
    parser.add_argument(
        '--rolename',
        metavar='NAME',
        default='hero',
        help='role_name attribute assigned to spawned vehicle (default: "hero")')
    parser.add_argument(
        '--attach_vehicle',
        help='Attach to an existing vehicle with this role_name; exits if not found')
    parser.add_argument(
        '-s', '--speed_limit',
        default=30,
        type=int,
        help='Initial target speed in kph for autopilot or manual step changes (default: 30)')
    parser.add_argument(
        '--x', type=float,
        help='X coordinate for spawn in CARLA left-handed map coordinates (use with --y and --z)')
    parser.add_argument(
        '--y', type=float,
        help='Y coordinate for spawn in CARLA left-handed map coordinates (use with --x and --z)')
    parser.add_argument(
        '--z', type=float,
        help='Z coordinate for spawn in CARLA left-handed map coordinates (use with --x and --y)')
    parser.add_argument(
        '--roll', type=float, default=0.0,
        help='Roll angle in degrees for spawn transform (default: 0.0)')
    parser.add_argument(
        '--pitch', type=float, default=0.0,
        help='Pitch angle in degrees for spawn transform (default: 0.0)')
    parser.add_argument(
        '--yaw', type=float, default=0.0,
        help='Yaw angle in degrees for spawn transform; set heading when providing XYZ (default: 0.0)')
    return parser.parse_args()


# -------------------------------------------------------------------------
# World & vehicle helpers
# -------------------------------------------------------------------------

def get_world_and_map(host, port):
    client = carla.Client(host, port)
    client.set_timeout(10.0)
    world = client.get_world()
    return world, world.get_map()


def find_existing_vehicle(world, follow_role_name):
    carla_vehicles = world.get_actors().filter('vehicle.*')
    for vehicle in carla_vehicles:
        current_attributes = vehicle.attributes
        print("Checking vehicle:", current_attributes.get("role_name", "<no-role>"))
        if current_attributes.get("role_name") == follow_role_name:
            print(f">>> Selected existing vehicle with role_name={follow_role_name}")
            return vehicle
    return None


def spawn_vehicle(world, carla_map, args):
    bp_lib = world.get_blueprint_library()
    bp_candidates = bp_lib.filter(args.filter)
    if not bp_candidates:
        raise RuntimeError(f"No blueprints found with filter {args.filter}")
    bp = bp_candidates[0]

    if bp.has_attribute('role_name'):
        bp.set_attribute('role_name', args.rolename)

    if args.x is not None and args.y is not None and args.z is not None:
        spawn_transform = carla.Transform(
            carla.Location(x=args.x, y=args.y, z=args.z),
            carla.Rotation(
                roll=args.roll,
                pitch=args.pitch,
                yaw=args.yaw
            )
        )
    else:
        spawn_points = carla_map.get_spawn_points()
        if not spawn_points:
            raise RuntimeError("No spawn points available in this map")
        spawn_transform = random.choice(spawn_points)

    print(f">>> Spawning vehicle with role_name={args.rolename} at {spawn_transform}")
    vehicle = world.try_spawn_actor(bp, spawn_transform)
    if vehicle is None:
        raise RuntimeError("Failed to spawn vehicle at requested transform")
    return vehicle


def setup_vehicle(world, carla_map, args):
    # Try existing vehicle first
    if not args.attach_vehicle: 
        vehicle = spawn_vehicle(world, carla_map, args)
    else:
        vehicle = find_existing_vehicle(world, args.attach_vehicle)
    
        if vehicle is None:
            print(f">>> No vehicle with role_name={args.attach_vehicle} found")
            sys.exit()

    # Make sure Traffic Manager autopilot is OFF – we are using BehaviorAgent
    vehicle.set_autopilot(False)

    return vehicle


# -------------------------------------------------------------------------
# BehaviorAgent setup (route-based autopilot)
# -------------------------------------------------------------------------

def setup_agent(vehicle, args):
    agent = BehaviorAgent(vehicle, behavior="normal")
    autopilot_active = False

    if args.dest:
        dest_loc = carla.Location(x=args.dest[0], y=args.dest[1], z=args.dest[2])
        print(f">>> Setting route destination to {dest_loc}")
        agent.set_destination(dest_loc)
        # Initial target speed in kph = args.speed_limit
        agent.set_target_speed(float(args.speed_limit))
        autopilot_active = True
    else:
        print(">>> No destination provided")

    return agent, autopilot_active


# -------------------------------------------------------------------------
# Main loop with keyboard speed control (SPACE / UP / DOWN)
# -------------------------------------------------------------------------

def run_loop(world, vehicle, agent, autopilot_active, args):
    # Pygame setup for keyboard events
    pygame.init()
    screen_width = 230
    screen_height = 350
    screen = pygame.display.set_mode((screen_width, screen_height))  # tiny window just to grab focus
    pygame.display.set_caption("Control Window")

    font = pygame.font.SysFont(None, 24, bold=True)

    # Track target speed in km/h (this is what BehaviorAgent expects)
    if args.speed_limit > 0:
        target_speed_kph = float(args.speed_limit)
    else:
        target_speed_kph = 0.0

    prev_target_speed_kph = 0.0

    print(f">>> Initial target speed = {target_speed_kph:.1f} kph")

    manual_speed_limit_enabled = True
    train_mode = False

    # Simple longitudinal control for train mode
    train_throttle = 0.0
    train_brake = 0.0

    try:
        while True:
            world.tick()

            # -----------------------------
            # Keyboard handling
            # -----------------------------
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    return

                if event.type == pygame.KEYDOWN:
                    # SPACE: stop / resume previous target speed
                    if event.key == pygame.K_SPACE:
                        if target_speed_kph not in (0.0, None):
                            prev_target_speed_kph = target_speed_kph
                            target_speed_kph = 0.0
                            print(">>> SPACE pressed: target speed set to 0.0 kph")
                        else:
                            # Restore previous target speed (only if manual speed is enabled)
                            if manual_speed_limit_enabled and prev_target_speed_kph is not None:
                                target_speed_kph = prev_target_speed_kph
                            print(f">>> SPACE pressed: returning target speed to {target_speed_kph} kph")

                    # UP: increase target speed by 5 kph
                    elif event.key == pygame.K_UP:
                        if manual_speed_limit_enabled:
                            # If we were in "auto" (None), start from 0
                            if target_speed_kph is None:
                                target_speed_kph = 0.0
                            target_speed_kph += 5.0
                            print(f">>> UP pressed: target speed = {target_speed_kph:.1f} kph")

                    # DOWN: decrease target speed by 5 kph (not below zero)
                    elif event.key == pygame.K_DOWN:
                        if manual_speed_limit_enabled:
                            if target_speed_kph is None:
                                target_speed_kph = 0.0
                            target_speed_kph = max(0.0, target_speed_kph - 5.0)
                            print(f">>> DOWN pressed: target speed = {target_speed_kph:.1f} kph")

                    # E: toggle manual speed limiting on/off
                    elif event.key == pygame.K_e:
                        manual_speed_limit_enabled = not manual_speed_limit_enabled
                        print(f">>> E pressed: toggling manual_speed_limit: {manual_speed_limit_enabled}")

                    # T: toggle train mode (agent steers, user does throttle/brake)
                    elif event.key == pygame.K_t:
                        train_mode = not train_mode
                        # Reset longitudinal when toggling
                        train_throttle = 0.0
                        train_brake = 0.0
                        print(f">>> T pressed: toggling train mode: {train_mode}")

                    # W: throttle (only in train mode)
                    elif event.key == pygame.K_w:
                        if train_mode:
                            train_throttle = 1.0   # full throttle
                            train_brake = 0.0
                            print(">>> W pressed (train mode): throttle=1.0, brake=0.0")

                    # S: brake (only in train mode)
                    elif event.key == pygame.K_s:
                        if train_mode:
                            train_throttle = 0.0
                            train_brake = 1.0   # full brake
                            print(">>> S pressed (train mode): throttle=0.0, brake=1.0")

                # When key is released, stop applying throttle/brake in train mode
                if event.type == pygame.KEYUP:
                    if train_mode and event.key in (pygame.K_w, pygame.K_s):
                        train_throttle = 0.0
                        train_brake = 0.0
                        print(">>> W/S released (train mode): throttle=0.0, brake=0.0")

            # If we disabled manual speed, pass None down to the agent
            manual_speed_value = target_speed_kph if manual_speed_limit_enabled else None

            # -----------------------------
            # Autopilot / agent control
            # -----------------------------
            if autopilot_active:
                if agent.done():
                    print(">>> Route completed")
                    autopilot_active = False
                    continue

                # Your modified BehaviorAgent.run_step(manual_speed_limit=...)
                control = agent.run_step(manual_speed_limit=manual_speed_value)

                if train_mode:
                    # Agent handles steering etc; user handles throttle/brake
                    control.throttle = train_throttle
                    control.brake = train_brake
                    control.hand_brake = 0

                vehicle.apply_control(control)

            # -----------------------------
            # Draw HUD text in the pygame window
            # -----------------------------
            screen.fill((0, 0, 0))  # black background

            # Static title text
            title1 = font.render("CLICK HERE", True, (255, 0, 0))
            title2 = font.render("TO CONTROL", True, (255, 0, 0))

            # Dynamic status text
            manual_text = f"Manual [E]: {'ON' if manual_speed_limit_enabled else 'OFF'}"
            if manual_speed_value is None:
                speed_text = "Speed: AUTO"
            else:
                speed_text = f"Speed: {manual_speed_value:.1f} kph"
            speed_increase_text = f"Increase Speed [UP]"
            speed_decrease_text = f"Decrease Speed [DOWN]"
            stop_text = f"Stop [SPACE]"
            train_text = f"Train [T]: {'ON' if train_mode else 'OFF'}"
            train_throttle_text = f"Train Throttle [W]"
            train_brake_text = f"Train Breke [S]"
            

            
            status1 = font.render(speed_text, True, (0, 255, 0))
            status2 = font.render(speed_increase_text, True, (255, 255, 255))
            status3 = font.render(speed_decrease_text, True, (255, 255, 255))
            status4 = font.render(stop_text, True, (255, 255, 255))
            status5 = font.render(manual_text, True, (255, 255, 255))
            status6 = font.render(train_text, True, (255, 255, 0))
            status7 = font.render(train_throttle_text, True, (255, 255, 0))
            status8 = font.render(train_brake_text, True, (255, 255, 0))

            # Positioning
            rect_title1 = title1.get_rect(center=(screen_width/2, 30))
            rect_title2 = title2.get_rect(center=(screen_width/2, 60))
            rect_status1 = status1.get_rect(center=(screen_width/2, 110))
            rect_status2 = status2.get_rect(center=(screen_width/2, 140))
            rect_status3 = status3.get_rect(center=(screen_width/2, 170))
            rect_status4 = status4.get_rect(center=(screen_width/2, 200))
            rect_status5 = status5.get_rect(center=(screen_width/2, 230))
            rect_status6 = status6.get_rect(center=(screen_width/2, 260))
            rect_status7 = status7.get_rect(center=(screen_width/2, 290))
            rect_status8 = status8.get_rect(center=(screen_width/2, 320))

            # Blit and flip
            screen.blit(title1, rect_title1)
            screen.blit(title2, rect_title2)
            screen.blit(status1, rect_status1)
            screen.blit(status2, rect_status2)
            screen.blit(status3, rect_status3)
            screen.blit(status4, rect_status4)
            screen.blit(status5, rect_status5)
            screen.blit(status6, rect_status6)
            screen.blit(status7, rect_status7)
            screen.blit(status8, rect_status8)
            pygame.display.flip()

    finally:
        print(">>> Exiting loop; ensuring autopilot is OFF.")
        try:
            vehicle.set_autopilot(False)
        except Exception as e:
            print(f"Failed to disable autopilot cleanly: {e}")



# -------------------------------------------------------------------------
# Entry point
# -------------------------------------------------------------------------

def main():
    args = parse_args()
    world, carla_map = get_world_and_map(args.host, args.port)
    vehicle = setup_vehicle(world, carla_map, args)
    agent, autopilot_active = setup_agent(vehicle, args)
    run_loop(world, vehicle, agent, autopilot_active, args)


if __name__ == '__main__':
    main()
