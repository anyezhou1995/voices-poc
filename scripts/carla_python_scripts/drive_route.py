#!/usr/bin/env python3

import argparse
import random
import sys
import os
import math

import pygame  # for keyboard input
import importlib.util

from find_carla_egg import find_carla_egg

# -------------------------------------------------------------------------
# Locate CARLA egg and add CARLA to sys.path
# -------------------------------------------------------------------------

carla_egg_file = find_carla_egg()
print(f"Found carla egg(s): {carla_egg_file}")

# Add egg so `import carla` works
sys.path.append(carla_egg_file)

# Get /home/.../CARLA/PythonAPI from egg path
# carla_pythonapi_root = os.path.dirname(os.path.dirname(os.path.dirname(carla_egg_file)))
# print(f"Using CARLA PythonAPI root: {carla_pythonapi_root}")

# Add PythonAPI root so we can load agent sources from there
# sys.path.append(carla_pythonapi_root)

import carla

# -------------------------------------------------------------------------
# Load BehaviorAgent directly from its file path
# -------------------------------------------------------------------------

def load_behavior_agent_class():
    # Try both common layouts:
    candidates = [
        os.path.join(carla_pythonapi_root, "agents", "navigation", "behavior_agent.py"),
        os.path.join(carla_pythonapi_root, "carla", "agents", "navigation", "behavior_agent.py"),
    ]

    behavior_agent_path = None
    for path in candidates:
        if os.path.isfile(path):
            behavior_agent_path = path
            break

    if behavior_agent_path is None:
        raise RuntimeError(
            "Could not find behavior_agent.py under PythonAPI. "
            f"Tried: {candidates}"
        )

    print(f"Loading BehaviorAgent from: {behavior_agent_path}")

    spec = importlib.util.spec_from_file_location(
        "carla_behavior_agent", behavior_agent_path
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module.BehaviorAgent


# BehaviorAgent = load_behavior_agent_class()

from agents.navigation.behavior_agent import BehaviorAgent


# -------------------------------------------------------------------------
# Argument parser
# -------------------------------------------------------------------------

def parse_args():
    parser = argparse.ArgumentParser(
        description='CARLA manual vehicle with optional route-based autopilot'
    )
    parser.add_argument(
        '-v', '--verbose',
        action='store_true',
        dest='debug',
        help='print debug information')
    parser.add_argument(
        '--host',
        metavar='H',
        default='127.0.0.1',
        help='IP of the host server (default: 127.0.0.1)')
    parser.add_argument(
        '-p', '--port',
        metavar='P',
        default=2000,
        type=int,
        help='TCP port to listen to (default: 2000)')
    parser.add_argument(
        '--filter',
        metavar='PATTERN',
        default='vehicle.*',
        help='actor filter (default: "vehicle.*")')
    parser.add_argument(
        '--rolename',
        metavar='NAME',
        default='hero',
        help='actor role name for spawned vehicle (default: "hero")')
    parser.add_argument(
        '--follow_vehicle',
        default="TFHRC-MANUAL-1",
        help='Vehicle role_name to use if it already exists in the world')
    parser.add_argument(
        '-s', '--speed_limit',
        metavar='S',
        default=50,
        type=int,
        help='Speed limit for vehicle in kph (default: 50 kph)')
    parser.add_argument(
        '--x', type=float,
        help='x coordinate of the spawn point in CARLA left handed coordinate system')
    parser.add_argument(
        '--y', type=float,
        help='y coordinate of the spawn point in CARLA left handed coordinate system')
    parser.add_argument(
        '--z', type=float,
        help='z coordinate of the spawn point in CARLA left handed coordinate system')
    parser.add_argument(
        '--roll', type=float, default=0.0,
        help='roll angle in degrees (CARLA left handed coordinate system)')
    parser.add_argument(
        '--pitch', type=float, default=0.0,
        help='pitch angle in degrees (CARLA left handed coordinate system)')
    parser.add_argument(
        '--yaw', type=float, default=0.0,
        help='yaw angle in degrees (CARLA left handed coordinate system)')
    parser.add_argument(
        '--dest',
        metavar=('DX', 'DY', 'DZ'),
        type=float,
        nargs=3,
        help='Destination coordinate for route autopilot (DX DY DZ)')
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
    vehicle = find_existing_vehicle(world, args.follow_vehicle)
    if vehicle is None:
        print(f">>> No vehicle with role_name={args.follow_vehicle} found, spawning a new one")
        vehicle = spawn_vehicle(world, carla_map, args)

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
        print(">>> No destination provided; route autopilot inactive, manual control expected.")

    return agent, autopilot_active


# -------------------------------------------------------------------------
# Main loop with keyboard speed control (SPACE / UP / DOWN)
# -------------------------------------------------------------------------

def run_loop(world, vehicle, agent, autopilot_active, args):
    # Pygame setup for keyboard events
    pygame.init()
    screen = pygame.display.set_mode((200, 200))  # tiny window just to grab focus
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
            manual_text = f"Manual: {'ON' if manual_speed_limit_enabled else 'OFF'}"
            if manual_speed_value is None:
                speed_text = "Speed: AUTO"
            else:
                speed_text = f"Speed: {manual_speed_value:.1f} kph"

            train_text = f"Train: {'ON' if train_mode else 'OFF'}"

            status1 = font.render(manual_text, True, (255, 255, 255))
            status2 = font.render(speed_text, True, (255, 255, 255))
            status3 = font.render(train_text, True, (255, 255, 0))

            # Positioning
            rect_title1 = title1.get_rect(center=(100, 30))
            rect_title2 = title2.get_rect(center=(100, 60))
            rect_status1 = status1.get_rect(center=(100, 110))
            rect_status2 = status2.get_rect(center=(100, 140))
            rect_status3 = status3.get_rect(center=(100, 170))

            # Blit and flip
            screen.blit(title1, rect_title1)
            screen.blit(title2, rect_title2)
            screen.blit(status1, rect_status1)
            screen.blit(status2, rect_status2)
            screen.blit(status3, rect_status3)
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
