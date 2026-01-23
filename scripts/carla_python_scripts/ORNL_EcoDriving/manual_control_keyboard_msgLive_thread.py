#!/usr/bin/env python3

# Copyright (c) 2019 Computer Vision Center (CVC) at the Universitat Autonoma de
# Barcelona (UAB).
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

# Allows controlling a vehicle with a keyboard. For a simpler and more
# documented example, please take a look at tutorial.py.

"""
Welcome to CARLA manual control.

Use ARROWS or WASD keys for control.

    W            : throttle
    S            : brake
    A/D          : steer left/right
    Q            : toggle reverse
    Space        : hand-brake
    P            : toggle autopilot
    M            : toggle manual transmission
    ,/.          : gear up/down
    CTRL + W     : toggle constant velocity mode at 60 km/h

    L            : toggle next light type
    SHIFT + L    : toggle high beam
    Z/X          : toggle right/left blinker
    I            : toggle interior light

    TAB          : change sensor position
    ` or N       : next sensor
    [1-9]        : change to sensor [1-9]
    G            : toggle radar visualization
    C            : change weather (Shift+C reverse)
    Backspace    : change vehicle

    R            : toggle recording images to disk

    CTRL + R     : toggle recording of simulation (replacing any previous)
    CTRL + P     : start replaying last recorded simulation
    CTRL + +     : increments the start time of the replay by 1 second (+SHIFT = 10 seconds)
    CTRL + -     : decrements the start time of the replay by 1 second (+SHIFT = 10 seconds)

    F1           : toggle HUD
    H/?          : toggle help
    ESC          : quit
"""

from __future__ import print_function


# ==============================================================================
# -- find carla module ---------------------------------------------------------
# ==============================================================================


import glob
import os
import sys

from find_carla_egg import find_carla_egg

carla_egg_file = find_carla_egg()

sys.path.append(carla_egg_file)

# Add the CARLA python scripts root so we can import local agent utilities
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
CARLA_SCRIPTS_ROOT = os.path.abspath(os.path.join(SCRIPT_DIR, os.pardir))
if CARLA_SCRIPTS_ROOT not in sys.path:
    sys.path.append(CARLA_SCRIPTS_ROOT)


# ==============================================================================
# -- imports -------------------------------------------------------------------
# ==============================================================================


import carla

from carla import ColorConverter as cc

import argparse
import collections
import datetime
import logging
import math
import random
import re
import weakref

try:
    import pygame
    from pygame.locals import KMOD_CTRL
    from pygame.locals import KMOD_SHIFT
    from pygame.locals import K_0
    from pygame.locals import K_9
    from pygame.locals import K_BACKQUOTE
    from pygame.locals import K_BACKSPACE
    from pygame.locals import K_COMMA
    from pygame.locals import K_DOWN
    from pygame.locals import K_ESCAPE
    from pygame.locals import K_F1
    from pygame.locals import K_LEFT
    from pygame.locals import K_PERIOD
    from pygame.locals import K_RIGHT
    from pygame.locals import K_SLASH
    from pygame.locals import K_SPACE
    from pygame.locals import K_TAB
    from pygame.locals import K_UP
    from pygame.locals import K_a
    from pygame.locals import K_c
    from pygame.locals import K_g
    from pygame.locals import K_d
    from pygame.locals import K_e
    from pygame.locals import K_h
    from pygame.locals import K_m
    from pygame.locals import K_n
    from pygame.locals import K_p
    from pygame.locals import K_q
    from pygame.locals import K_r
    from pygame.locals import K_s
    from pygame.locals import K_w
    from pygame.locals import K_l
    from pygame.locals import K_i
    from pygame.locals import K_z
    from pygame.locals import K_x
    from pygame.locals import K_MINUS
    from pygame.locals import K_EQUALS
except ImportError:
    raise RuntimeError('cannot import pygame, make sure pygame package is installed')

try:
    import numpy as np
except ImportError:
    raise RuntimeError('cannot import numpy, make sure numpy package is installed')


# ==============================================================================
# -- Global functions ----------------------------------------------------------
# ==============================================================================


def find_weather_presets():
    rgx = re.compile('.+?(?:(?<=[a-z])(?=[A-Z])|(?<=[A-Z])(?=[A-Z][a-z])|$)')
    name = lambda x: ' '.join(m.group(0) for m in rgx.finditer(x))
    presets = [x for x in dir(carla.WeatherParameters) if re.match('[A-Z].+', x)]
    return [(getattr(carla.WeatherParameters, x), name(x)) for x in presets]


def get_actor_display_name(actor, truncate=250):
    name = ' '.join(actor.type_id.replace('_', '.').title().split('.')[1:])
    return (name[:truncate - 1] + u'\u2026') if len(name) > truncate else name


# ==============================================================================
# -- World ---------------------------------------------------------------------
# ==============================================================================


class World(object):
    def __init__(self, carla_world, hud, spawn_loc, args):
        self.world = carla_world
        self.spectator = self.world.get_spectator()
        self._bev_height = 80.0
        self._spectator_location = None
        self._spectator_rotation = None
        self._spectator_height = None
        self._location_smoothing = 0.1
        self._height_smoothing = 0.02
        self._rotation_smoothing = 0.15
        self.actor_role_name = args.rolename
        try:
            self.map = self.world.get_map()
        except RuntimeError as error:
            print('RuntimeError: {}'.format(error))
            print('  The server could not send the OpenDRIVE (.xodr) file:')
            print('  Make sure it exists, has the same name of your town, and is correct.')
            sys.exit(1)
        self.hud = hud
        self.player = None
        self.PID = None
        self.collision_sensor = None
        self.lane_invasion_sensor = None
        self.gnss_sensor = None
        self.imu_sensor = None
        self.radar_sensor = None
        self.camera_manager = None
        self._weather_presets = find_weather_presets()
        self._weather_index = 0
        #self._actor_filter = args.filter
        self._actor_filter = 'vehicle.toyota.prius'
        self._gamma = args.gamma
        self.spawn_loc = spawn_loc
        self.restart(args)
        self.world.on_tick(hud.on_world_tick)
        self.recording_enabled = False
        self.recording_start = 0
        self.constant_velocity_enabled = False

    def restart(self,args):
        self.player_max_speed = 1.589
        self.player_max_speed_fast = 3.713
        # Keep same camera config if the camera manager exists.
        cam_index = self.camera_manager.index if self.camera_manager is not None else 0
        cam_pos_index = self.camera_manager.transform_index if self.camera_manager is not None else 0
        # Get a random blueprint.
        blueprint = random.choice(self.world.get_blueprint_library().filter(self._actor_filter))
        blueprint.set_attribute('role_name', self.actor_role_name)
        if blueprint.has_attribute('color'):
            color = random.choice(blueprint.get_attribute('color').recommended_values)
            blueprint.set_attribute('color', color)
        if blueprint.has_attribute('driver_id'):
            driver_id = random.choice(blueprint.get_attribute('driver_id').recommended_values)
            blueprint.set_attribute('driver_id', driver_id)
        if blueprint.has_attribute('is_invincible'):
            blueprint.set_attribute('is_invincible', 'true')
        # set the max speed
        if blueprint.has_attribute('speed'):
            self.player_max_speed = float(blueprint.get_attribute('speed').recommended_values[1])
            self.player_max_speed_fast = float(blueprint.get_attribute('speed').recommended_values[2])
        else:
            print("No recommended values for 'speed' attribute")
        # Spawn the player.
        if self.player is not None:
            spawn_point = self.player.get_transform()
            spawn_point.location.z += 2.0
            spawn_point.rotation.roll = 0.0
            spawn_point.rotation.pitch = 0.0
            self.destroy()
            self.player = self.world.try_spawn_actor(blueprint, spawn_point)
        while self.player is None:
            if args.x and args.y and args.z:
                print("spawning in custom loc")
                spawn_point = carla.Transform(carla.Location(x=args.x,y=args.y,z=args.z), carla.Rotation(yaw=90))
            else:
                if not self.map.get_spawn_points():
                    print('There are no spawn points available in your map/town.')
                    print('Please add some Vehicle Spawn Point to your UE4 scene.')
                    sys.exit(1)
                #spawn_points = self.map.get_spawn_points()
                #spawn_point = random.choice(spawn_points) if spawn_points else carla.Transform()
                #spawn_point = carla.Transform(carla.Location(x=52.122, y=2.986, z=237.5), carla.Rotation(pitch=0.766, yaw=-105.963, roll=-0.953))
                #spawn_point = carla.Transform(carla.Location(x=62.598, y=80.402, z=237.344), carla.Rotation(pitch=0.766, yaw=-105.963, roll=-0.953))
                spawn_point = self.spawn_loc

            self.player = self.world.try_spawn_actor(blueprint, spawn_point)
        # Set up the sensors.
        self.collision_sensor = CollisionSensor(self.player, self.hud)
        self.lane_invasion_sensor = LaneInvasionSensor(self.player, self.hud)
        self.gnss_sensor = GnssSensor(self.player)
        self.imu_sensor = IMUSensor(self.player)
        self.camera_manager = CameraManager(self.player, self.hud, self._gamma)
        self.camera_manager.transform_index = cam_pos_index
        self.camera_manager.set_sensor(cam_index, notify=False)
        actor_type = get_actor_display_name(self.player)
        self.hud.notification(actor_type)
        self._spectator_location = None
        self._spectator_rotation = None
        self._spectator_height = None
        #self._update_spectator()

        # settign up PID controller
        args_lateral = {'K_P': 1.95/2, 'K_D': 0.2/2, 'K_I': 0.075, 'dt': 0.08}
        args_longitudinal = {'K_P': 1.0*3.5, 'K_D': 0.02, 'K_I': 0.025, 'dt': 0.08}
        self.PID = VehiclePIDController(self.player, args_lateral, args_longitudinal)

    def next_weather(self, reverse=False):
        self._weather_index += -1 if reverse else 1
        self._weather_index %= len(self._weather_presets)
        preset = self._weather_presets[self._weather_index]
        self.hud.notification('Weather: %s' % preset[1])
        self.player.get_world().set_weather(preset[0])

    def toggle_radar(self):
        if self.radar_sensor is None:
            self.radar_sensor = RadarSensor(self.player)
        elif self.radar_sensor.sensor is not None:
            self.radar_sensor.sensor.destroy()
            self.radar_sensor = None

    def _update_spectator(self):
        if self.player is None or self.spectator is None:
            return

        transform = self.player.get_transform()
        target_rotation = carla.Rotation(pitch=-90.0, yaw=transform.rotation.yaw, roll=0.0)

        height_target = transform.location.z + self._bev_height
        if self._spectator_height is None:
            self._spectator_height = height_target
        else:
            self._spectator_height += (height_target - self._spectator_height) * self._height_smoothing

        target_location = carla.Location(
            x=transform.location.x,
            y=transform.location.y,
            z=self._spectator_height)

        if self._spectator_location is None:
            self._spectator_location = carla.Location(
                x=target_location.x,
                y=target_location.y,
                z=target_location.z)
        else:
            smoothing = self._location_smoothing
            self._spectator_location.x += (target_location.x - self._spectator_location.x) * smoothing
            self._spectator_location.y += (target_location.y - self._spectator_location.y) * smoothing
            self._spectator_location.z = self._spectator_height

        if self._spectator_rotation is None:
            self._spectator_rotation = target_rotation
        else:
            alpha = self._rotation_smoothing
            yaw = self._lerp_angle(self._spectator_rotation.yaw, target_rotation.yaw, alpha)
            pitch = self._lerp_angle(self._spectator_rotation.pitch, target_rotation.pitch, alpha)
            roll = self._lerp_angle(self._spectator_rotation.roll, target_rotation.roll, alpha)
            self._spectator_rotation = carla.Rotation(pitch=pitch, yaw=yaw, roll=roll)

        self.spectator.set_transform(carla.Transform(self._spectator_location, self._spectator_rotation))

    @staticmethod
    def _lerp_angle(current, target, alpha):
        delta = (target - current + 180.0) % 360.0 - 180.0
        return current + delta * alpha

    def tick(self, clock):
        #self._update_spectator()
        self.hud.tick(self, clock)

    def render(self, display):
        self.camera_manager.render(display)
        self.hud.render(display)

    def destroy_sensors(self):
        self.camera_manager.sensor.destroy()
        self.camera_manager.sensor = None
        self.camera_manager.index = None

    def destroy(self):
        if self.radar_sensor is not None:
            self.toggle_radar()
        sensors = [
            self.camera_manager.sensor,
            self.collision_sensor.sensor,
            self.lane_invasion_sensor.sensor,
            self.gnss_sensor.sensor,
            self.imu_sensor.sensor]
        for sensor in sensors:
            if sensor is not None:
                sensor.stop()
                sensor.destroy()
        if self.player is not None:
            self.player.destroy()


# ==============================================================================
# -- KeyboardControl -----------------------------------------------------------
# ==============================================================================


class KeyboardControl(object):
    """Class that handles keyboard input."""
    def __init__(self, world, start_in_autopilot):
        self._autopilot_enabled = start_in_autopilot
        if isinstance(world.player, carla.Vehicle):
            self._control = carla.VehicleControl()
            self._lights = carla.VehicleLightState.NONE
            world.player.set_autopilot(self._autopilot_enabled)
            world.player.set_light_state(self._lights)
            self.eco_drive = False
        elif isinstance(world.player, carla.Walker):
            self._control = carla.WalkerControl()
            self._autopilot_enabled = False
            self._rotation = world.player.get_transform().rotation
        else:
            raise NotImplementedError("Actor type not supported")
        self._steer_cache = 0.0
        world.hud.notification("Press 'H' or '?' for help.", seconds=4.0)

    def parse_events(self, client, world, clock, ref_speed, ref_transform, args):
        if isinstance(self._control, carla.VehicleControl):
            current_lights = self._lights
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                return True
            elif event.type == pygame.KEYUP:
                if self._is_quit_shortcut(event.key):
                    return True
                elif event.key == K_BACKSPACE:
                    if self._autopilot_enabled:
                        world.player.set_autopilot(False)
                        world.restart(args)
                        world.player.set_autopilot(True)
                    else:
                        world.restart(args)
                elif event.key == K_F1:
                    world.hud.toggle_info()
                elif event.key == K_h or (event.key == K_SLASH and pygame.key.get_mods() & KMOD_SHIFT):
                    world.hud.help.toggle()
                elif event.key == K_TAB:
                    world.camera_manager.toggle_camera()
                elif event.key == K_c and pygame.key.get_mods() & KMOD_SHIFT:
                    world.next_weather(reverse=True)
                elif event.key == K_c:
                    world.next_weather()
                elif event.key == K_e:
                    self.eco_drive = not self.eco_drive
                    print('################## ORNL eco-driving mode: ', self.eco_drive, '##################')
                elif event.key == K_g:
                    world.toggle_radar()
                elif event.key == K_BACKQUOTE:
                    world.camera_manager.next_sensor()
                elif event.key == K_n:
                    world.camera_manager.next_sensor()
                elif event.key == K_w and (pygame.key.get_mods() & KMOD_CTRL):
                    if world.constant_velocity_enabled:
                        world.player.disable_constant_velocity()
                        world.constant_velocity_enabled = False
                        world.hud.notification("Disabled Constant Velocity Mode")
                    else:
                        world.player.enable_constant_velocity(carla.Vector3D(17, 0, 0))
                        world.constant_velocity_enabled = True
                        world.hud.notification("Enabled Constant Velocity Mode at 60 km/h")
                elif event.key > K_0 and event.key <= K_9:
                    world.camera_manager.set_sensor(event.key - 1 - K_0)
                elif event.key == K_r and not (pygame.key.get_mods() & KMOD_CTRL):
                    world.camera_manager.toggle_recording()
                elif event.key == K_r and (pygame.key.get_mods() & KMOD_CTRL):
                    if (world.recording_enabled):
                        client.stop_recorder()
                        world.recording_enabled = False
                        world.hud.notification("Recorder is OFF")
                    else:
                        client.start_recorder("manual_recording.rec")
                        world.recording_enabled = True
                        world.hud.notification("Recorder is ON")
                elif event.key == K_p and (pygame.key.get_mods() & KMOD_CTRL):
                    # stop recorder
                    client.stop_recorder()
                    world.recording_enabled = False
                    # work around to fix camera at start of replaying
                    current_index = world.camera_manager.index
                    world.destroy_sensors()
                    # disable autopilot
                    self._autopilot_enabled = False
                    world.player.set_autopilot(self._autopilot_enabled)
                    world.hud.notification("Replaying file 'manual_recording.rec'")
                    # replayer
                    client.replay_file("manual_recording.rec", world.recording_start, 0, 0)
                    world.camera_manager.set_sensor(current_index)
                elif event.key == K_MINUS and (pygame.key.get_mods() & KMOD_CTRL):
                    if pygame.key.get_mods() & KMOD_SHIFT:
                        world.recording_start -= 10
                    else:
                        world.recording_start -= 1
                    world.hud.notification("Recording start time is %d" % (world.recording_start))
                elif event.key == K_EQUALS and (pygame.key.get_mods() & KMOD_CTRL):
                    if pygame.key.get_mods() & KMOD_SHIFT:
                        world.recording_start += 10
                    else:
                        world.recording_start += 1
                    world.hud.notification("Recording start time is %d" % (world.recording_start))
                
                if isinstance(self._control, carla.VehicleControl):
                    if event.key == K_q:
                        self._control.gear = 1 if self._control.reverse else -1
                    elif event.key == K_m:
                        self._control.manual_gear_shift = not self._control.manual_gear_shift
                        self._control.gear = world.player.get_control().gear
                        world.hud.notification('%s Transmission' %
                                               ('Manual' if self._control.manual_gear_shift else 'Automatic'))
                    elif self._control.manual_gear_shift and event.key == K_COMMA:
                        self._control.gear = max(-1, self._control.gear - 1)
                    elif self._control.manual_gear_shift and event.key == K_PERIOD:
                        self._control.gear = self._control.gear + 1
                    elif event.key == K_p and not pygame.key.get_mods() & KMOD_CTRL:
                        self._autopilot_enabled = not self._autopilot_enabled
                        world.player.set_autopilot(self._autopilot_enabled)
                        world.hud.notification(
                            'Autopilot %s' % ('On' if self._autopilot_enabled else 'Off'))
                    elif event.key == K_l and pygame.key.get_mods() & KMOD_CTRL:
                        current_lights ^= carla.VehicleLightState.Special1
                    elif event.key == K_l and pygame.key.get_mods() & KMOD_SHIFT:
                        current_lights ^= carla.VehicleLightState.HighBeam
                    elif event.key == K_l:
                        # Use 'L' key to switch between lights:
                        # closed -> position -> low beam -> fog
                        if not self._lights & carla.VehicleLightState.Position:
                            world.hud.notification("Position lights")
                            current_lights |= carla.VehicleLightState.Position
                        else:
                            world.hud.notification("Low beam lights")
                            current_lights |= carla.VehicleLightState.LowBeam
                        if self._lights & carla.VehicleLightState.LowBeam:
                            world.hud.notification("Fog lights")
                            current_lights |= carla.VehicleLightState.Fog
                        if self._lights & carla.VehicleLightState.Fog:
                            world.hud.notification("Lights off")
                            current_lights ^= carla.VehicleLightState.Position
                            current_lights ^= carla.VehicleLightState.LowBeam
                            current_lights ^= carla.VehicleLightState.Fog
                    elif event.key == K_i:
                        current_lights ^= carla.VehicleLightState.Interior
                    elif event.key == K_z:
                        current_lights ^= carla.VehicleLightState.LeftBlinker
                    elif event.key == K_x:
                        current_lights ^= carla.VehicleLightState.RightBlinker

        if not self._autopilot_enabled:
            if isinstance(self._control, carla.VehicleControl):
                self._parse_vehicle_keys(pygame.key.get_pressed(), clock.get_time())
                self._control.reverse = self._control.gear < 0
                # Set automatic control-related vehicle lights
                if self._control.brake:
                    current_lights |= carla.VehicleLightState.Brake
                else: # Remove the Brake flag
                    current_lights &= ~carla.VehicleLightState.Brake
                if self._control.reverse:
                    current_lights |= carla.VehicleLightState.Reverse
                else: # Remove the Reverse flag
                    current_lights &= ~carla.VehicleLightState.Reverse
                if current_lights != self._lights: # Change the light state only if necessary
                    self._lights = current_lights
                    world.player.set_light_state(carla.VehicleLightState(self._lights))
            elif isinstance(self._control, carla.WalkerControl):
                self._parse_walker_keys(pygame.key.get_pressed(), clock.get_time(), world)
            
            if self.eco_drive == True:
                control = world.PID.run_step(ref_speed, ref_transform)
                #print(control.throttle, control.brake)
                
                self._control.steer = control.steer
                self._control.throttle = control.throttle
                self._control.brake = control.brake
                self._control.hand_brake = False
                self._control.manual_gear_shift = False

                #if np.sqrt(world.player.get_velocity().x**2 + world.player.get_velocity().y**2) <= 0.01:
                    #self._control.steer = 0
                
            world.player.apply_control(self._control)

    def _parse_vehicle_keys(self, keys, milliseconds):
        if keys[K_UP] or keys[K_w]:
            self._control.throttle = min(self._control.throttle + 0.01, 1)
        else:
            self._control.throttle = 0.0

        if keys[K_DOWN] or keys[K_s]:
            self._control.brake = min(self._control.brake + 0.2, 1)
        else:
            self._control.brake = 0

        steer_increment = 5e-4 * milliseconds
        if keys[K_LEFT] or keys[K_a]:
            if self._steer_cache > 0:
                self._steer_cache = 0
            else:
                self._steer_cache -= steer_increment
        elif keys[K_RIGHT] or keys[K_d]:
            if self._steer_cache < 0:
                self._steer_cache = 0
            else:
                self._steer_cache += steer_increment
        else:
            self._steer_cache = 0.0
        self._steer_cache = min(0.7, max(-0.7, self._steer_cache))
        self._control.steer = round(self._steer_cache, 1)
        self._control.hand_brake = keys[K_SPACE]

    def _parse_walker_keys(self, keys, milliseconds, world):
        self._control.speed = 0.0
        if keys[K_DOWN] or keys[K_s]:
            self._control.speed = 0.0
        if keys[K_LEFT] or keys[K_a]:
            self._control.speed = .01
            self._rotation.yaw -= 0.08 * milliseconds
        if keys[K_RIGHT] or keys[K_d]:
            self._control.speed = .01
            self._rotation.yaw += 0.08 * milliseconds
        if keys[K_UP] or keys[K_w]:
            self._control.speed = world.player_max_speed_fast if pygame.key.get_mods() & KMOD_SHIFT else world.player_max_speed
        self._control.jump = keys[K_SPACE]
        self._rotation.yaw = round(self._rotation.yaw, 1)
        self._control.direction = self._rotation.get_forward_vector()

    @staticmethod
    def _is_quit_shortcut(key):
        return (key == K_ESCAPE) or (key == K_q and pygame.key.get_mods() & KMOD_CTRL)


# ==============================================================================
# -- HUD -----------------------------------------------------------------------
# ==============================================================================


class HUD(object):
    def __init__(self, width, height):
        self.dim = (width, height)
        font = pygame.font.Font(pygame.font.get_default_font(), 20)
        font_name = 'courier' if os.name == 'nt' else 'mono'
        fonts = [x for x in pygame.font.get_fonts() if font_name in x]
        default_font = 'ubuntumono'
        if len(fonts) == 0:
            fonts = [pygame.font.get_fonts()]
        mono = default_font if default_font in fonts else fonts[0]
        mono = pygame.font.match_font(mono)
        self._font_mono = pygame.font.Font(mono, 12 if os.name == 'nt' else 14)
        self._notifications = FadingText(font, (width, 40), (0, height - 40))
        self.help = HelpText(pygame.font.Font(mono, 16), width, height)
        self.server_fps = 0
        self.frame = 0
        self.simulation_time = 0
        self._show_info = True
        self._info_text = []
        self._server_clock = pygame.time.Clock()

    def on_world_tick(self, timestamp):
        self._server_clock.tick()
        self.server_fps = self._server_clock.get_fps()
        self.frame = timestamp.frame
        self.simulation_time = timestamp.elapsed_seconds

    def tick(self, world, clock):
        self._notifications.tick(world, clock)
        if not self._show_info:
            return
        t = world.player.get_transform()
        v = world.player.get_velocity()
        c = world.player.get_control()
        compass = world.imu_sensor.compass
        heading = 'N' if compass > 270.5 or compass < 89.5 else ''
        heading += 'S' if 90.5 < compass < 269.5 else ''
        heading += 'E' if 0.5 < compass < 179.5 else ''
        heading += 'W' if 180.5 < compass < 359.5 else ''
        colhist = world.collision_sensor.get_collision_history()
        collision = [colhist[x + self.frame - 200] for x in range(0, 200)]
        max_col = max(1.0, max(collision))
        collision = [x / max_col for x in collision]
        vehicles = world.world.get_actors().filter('vehicle.*')
        self._info_text = [
            'Server:  % 16.0f FPS' % self.server_fps,
            'Client:  % 16.0f FPS' % clock.get_fps(),
            '',
            'Vehicle: % 20s' % get_actor_display_name(world.player, truncate=20),
            'Map:     % 20s' % world.map.name,
            'Simulation time: % 12s' % datetime.timedelta(seconds=int(self.simulation_time)),
            '',
            'Speed:   % 15.0f km/h' % (3.6 * math.sqrt(v.x**2 + v.y**2 + v.z**2)),
            u'Compass:% 17.0f\N{DEGREE SIGN} % 2s' % (compass, heading),
            'Accelero: (%5.1f,%5.1f,%5.1f)' % (world.imu_sensor.accelerometer),
            'Gyroscop: (%5.1f,%5.1f,%5.1f)' % (world.imu_sensor.gyroscope),
            'Location:% 20s' % ('(% 5.6f, % 5.6f)' % (t.location.x, t.location.y)),
            'GNSS:% 24s' % ('(% 2.6f, % 3.6f)' % (world.gnss_sensor.lat, world.gnss_sensor.lon)),
            'Height:  % 18.0f m' % t.location.z,
            '']
        if isinstance(c, carla.VehicleControl):
            self._info_text += [
                ('Throttle:', c.throttle, 0.0, 1.0),
                ('Steer:', c.steer, -1.0, 1.0),
                ('Brake:', c.brake, 0.0, 1.0),
                ('Reverse:', c.reverse),
                ('Hand brake:', c.hand_brake),
                ('Manual:', c.manual_gear_shift),
                'Gear:        %s' % {-1: 'R', 0: 'N'}.get(c.gear, c.gear)]
        elif isinstance(c, carla.WalkerControl):
            self._info_text += [
                ('Speed:', c.speed, 0.0, 5.556),
                ('Jump:', c.jump)]
        self._info_text += [
            '',
            'Collision:',
            collision,
            '',
            'Number of vehicles: % 8d' % len(vehicles)]
        if len(vehicles) > 1:
            self._info_text += ['Nearby vehicles:']
            distance = lambda l: math.sqrt((l.x - t.location.x)**2 + (l.y - t.location.y)**2 + (l.z - t.location.z)**2)
            vehicles = [(distance(x.get_location()), x) for x in vehicles if x.id != world.player.id]
            for d, vehicle in sorted(vehicles, key=lambda vehicles: vehicles[0]):
                if d > 200.0:
                    break
                vehicle_type = get_actor_display_name(vehicle, truncate=22)
                self._info_text.append('% 4dm %s' % (d, vehicle_type))

    def toggle_info(self):
        self._show_info = not self._show_info

    def notification(self, text, seconds=2.0):
        self._notifications.set_text(text, seconds=seconds)

    def error(self, text):
        self._notifications.set_text('Error: %s' % text, (255, 0, 0))

    def render(self, display):
        if self._show_info:
            info_surface = pygame.Surface((220, self.dim[1]))
            info_surface.set_alpha(100)
            display.blit(info_surface, (0, 0))
            v_offset = 4
            bar_h_offset = 100
            bar_width = 106
            for item in self._info_text:
                if v_offset + 18 > self.dim[1]:
                    break
                if isinstance(item, list):
                    if len(item) > 1:
                        points = [(x + 8, v_offset + 8 + (1.0 - y) * 30) for x, y in enumerate(item)]
                        pygame.draw.lines(display, (255, 136, 0), False, points, 2)
                    item = None
                    v_offset += 18
                elif isinstance(item, tuple):
                    if isinstance(item[1], bool):
                        rect = pygame.Rect((bar_h_offset, v_offset + 8), (6, 6))
                        pygame.draw.rect(display, (255, 255, 255), rect, 0 if item[1] else 1)
                    else:
                        rect_border = pygame.Rect((bar_h_offset, v_offset + 8), (bar_width, 6))
                        pygame.draw.rect(display, (255, 255, 255), rect_border, 1)
                        f = (item[1] - item[2]) / (item[3] - item[2])
                        if item[2] < 0.0:
                            rect = pygame.Rect((bar_h_offset + f * (bar_width - 6), v_offset + 8), (6, 6))
                        else:
                            rect = pygame.Rect((bar_h_offset, v_offset + 8), (f * bar_width, 6))
                        pygame.draw.rect(display, (255, 255, 255), rect)
                    item = item[0]
                if item:  # At this point has to be a str.
                    surface = self._font_mono.render(item, True, (255, 255, 255))
                    display.blit(surface, (8, v_offset))
                v_offset += 18
        self._notifications.render(display)
        self.help.render(display)


# ==============================================================================
# -- FadingText ----------------------------------------------------------------
# ==============================================================================


class FadingText(object):
    def __init__(self, font, dim, pos):
        self.font = font
        self.dim = dim
        self.pos = pos
        self.seconds_left = 0
        self.surface = pygame.Surface(self.dim)

    def set_text(self, text, color=(255, 255, 255), seconds=2.0):
        text_texture = self.font.render(text, True, color)
        self.surface = pygame.Surface(self.dim)
        self.seconds_left = seconds
        self.surface.fill((0, 0, 0, 0))
        self.surface.blit(text_texture, (10, 11))

    def tick(self, _, clock):
        delta_seconds = 1e-3 * clock.get_time()
        self.seconds_left = max(0.0, self.seconds_left - delta_seconds)
        self.surface.set_alpha(500.0 * self.seconds_left)

    def render(self, display):
        display.blit(self.surface, self.pos)


# ==============================================================================
# -- HelpText ------------------------------------------------------------------
# ==============================================================================


class HelpText(object):
    """Helper class to handle text output using pygame"""
    def __init__(self, font, width, height):
        lines = __doc__.split('\n')
        self.font = font
        self.line_space = 18
        self.dim = (780, len(lines) * self.line_space + 12)
        self.pos = (0.5 * width - 0.5 * self.dim[0], 0.5 * height - 0.5 * self.dim[1])
        self.seconds_left = 0
        self.surface = pygame.Surface(self.dim)
        self.surface.fill((0, 0, 0, 0))
        for n, line in enumerate(lines):
            text_texture = self.font.render(line, True, (255, 255, 255))
            self.surface.blit(text_texture, (22, n * self.line_space))
            self._render = False
        self.surface.set_alpha(220)

    def toggle(self):
        self._render = not self._render

    def render(self, display):
        if self._render:
            display.blit(self.surface, self.pos)


# ==============================================================================
# -- CollisionSensor -----------------------------------------------------------
# ==============================================================================


class CollisionSensor(object):
    def __init__(self, parent_actor, hud):
        self.sensor = None
        self.history = []
        self._parent = parent_actor
        self.hud = hud
        world = self._parent.get_world()
        bp = world.get_blueprint_library().find('sensor.other.collision')
        self.sensor = world.spawn_actor(bp, carla.Transform(), attach_to=self._parent)
        # We need to pass the lambda a weak reference to self to avoid circular
        # reference.
        weak_self = weakref.ref(self)
        self.sensor.listen(lambda event: CollisionSensor._on_collision(weak_self, event))

    def get_collision_history(self):
        history = collections.defaultdict(int)
        for frame, intensity in self.history:
            history[frame] += intensity
        return history

    @staticmethod
    def _on_collision(weak_self, event):
        self = weak_self()
        if not self:
            return
        actor_type = get_actor_display_name(event.other_actor)
        self.hud.notification('Collision with %r' % actor_type)
        impulse = event.normal_impulse
        intensity = math.sqrt(impulse.x**2 + impulse.y**2 + impulse.z**2)
        self.history.append((event.frame, intensity))
        if len(self.history) > 4000:
            self.history.pop(0)


# ==============================================================================
# -- LaneInvasionSensor --------------------------------------------------------
# ==============================================================================


class LaneInvasionSensor(object):
    def __init__(self, parent_actor, hud):
        self.sensor = None
        self._parent = parent_actor
        self.hud = hud
        world = self._parent.get_world()
        bp = world.get_blueprint_library().find('sensor.other.lane_invasion')
        self.sensor = world.spawn_actor(bp, carla.Transform(), attach_to=self._parent)
        # We need to pass the lambda a weak reference to self to avoid circular
        # reference.
        weak_self = weakref.ref(self)
        self.sensor.listen(lambda event: LaneInvasionSensor._on_invasion(weak_self, event))

    @staticmethod
    def _on_invasion(weak_self, event):
        self = weak_self()
        if not self:
            return
        lane_types = set(x.type for x in event.crossed_lane_markings)
        text = ['%r' % str(x).split()[-1] for x in lane_types]
        self.hud.notification('Crossed line %s' % ' and '.join(text))


# ==============================================================================
# -- GnssSensor ----------------------------------------------------------------
# ==============================================================================


class GnssSensor(object):
    def __init__(self, parent_actor):
        self.sensor = None
        self._parent = parent_actor
        self.lat = 0.0
        self.lon = 0.0
        world = self._parent.get_world()
        bp = world.get_blueprint_library().find('sensor.other.gnss')
        self.sensor = world.spawn_actor(bp, carla.Transform(carla.Location(x=1.0, z=2.8)), attach_to=self._parent)
        # We need to pass the lambda a weak reference to self to avoid circular
        # reference.
        weak_self = weakref.ref(self)
        self.sensor.listen(lambda event: GnssSensor._on_gnss_event(weak_self, event))

    @staticmethod
    def _on_gnss_event(weak_self, event):
        self = weak_self()
        if not self:
            return
        self.lat = event.latitude
        self.lon = event.longitude


# ==============================================================================
# -- IMUSensor -----------------------------------------------------------------
# ==============================================================================


class IMUSensor(object):
    def __init__(self, parent_actor):
        self.sensor = None
        self._parent = parent_actor
        self.accelerometer = (0.0, 0.0, 0.0)
        self.gyroscope = (0.0, 0.0, 0.0)
        self.compass = 0.0
        world = self._parent.get_world()
        bp = world.get_blueprint_library().find('sensor.other.imu')
        self.sensor = world.spawn_actor(
            bp, carla.Transform(), attach_to=self._parent)
        # We need to pass the lambda a weak reference to self to avoid circular
        # reference.
        weak_self = weakref.ref(self)
        self.sensor.listen(
            lambda sensor_data: IMUSensor._IMU_callback(weak_self, sensor_data))

    @staticmethod
    def _IMU_callback(weak_self, sensor_data):
        self = weak_self()
        if not self:
            return
        limits = (-99.9, 99.9)
        self.accelerometer = (
            max(limits[0], min(limits[1], sensor_data.accelerometer.x)),
            max(limits[0], min(limits[1], sensor_data.accelerometer.y)),
            max(limits[0], min(limits[1], sensor_data.accelerometer.z)))
        self.gyroscope = (
            max(limits[0], min(limits[1], math.degrees(sensor_data.gyroscope.x))),
            max(limits[0], min(limits[1], math.degrees(sensor_data.gyroscope.y))),
            max(limits[0], min(limits[1], math.degrees(sensor_data.gyroscope.z))))
        self.compass = math.degrees(sensor_data.compass)


# ==============================================================================
# -- RadarSensor ---------------------------------------------------------------
# ==============================================================================


class RadarSensor(object):
    def __init__(self, parent_actor):
        self.sensor = None
        self._parent = parent_actor
        self.velocity_range = 7.5 # m/s
        world = self._parent.get_world()
        self.debug = world.debug
        bp = world.get_blueprint_library().find('sensor.other.radar')
        bp.set_attribute('horizontal_fov', str(35))
        bp.set_attribute('vertical_fov', str(20))
        self.sensor = world.spawn_actor(
            bp,
            carla.Transform(
                carla.Location(x=2.8, z=1.0),
                carla.Rotation(pitch=5)),
            attach_to=self._parent)
        # We need a weak reference to self to avoid circular reference.
        weak_self = weakref.ref(self)
        self.sensor.listen(
            lambda radar_data: RadarSensor._Radar_callback(weak_self, radar_data))

    @staticmethod
    def _Radar_callback(weak_self, radar_data):
        self = weak_self()
        if not self:
            return
        # To get a numpy [[vel, altitude, azimuth, depth],...[,,,]]:
        # points = np.frombuffer(radar_data.raw_data, dtype=np.dtype('f4'))
        # points = np.reshape(points, (len(radar_data), 4))

        current_rot = radar_data.transform.rotation
        for detect in radar_data:
            azi = math.degrees(detect.azimuth)
            alt = math.degrees(detect.altitude)
            # The 0.25 adjusts a bit the distance so the dots can
            # be properly seen
            fw_vec = carla.Vector3D(x=detect.depth - 0.25)
            carla.Transform(
                carla.Location(),
                carla.Rotation(
                    pitch=current_rot.pitch + alt,
                    yaw=current_rot.yaw + azi,
                    roll=current_rot.roll)).transform(fw_vec)

            def clamp(min_v, max_v, value):
                return max(min_v, min(value, max_v))

            norm_velocity = detect.velocity / self.velocity_range # range [-1, 1]
            r = int(clamp(0.0, 1.0, 1.0 - norm_velocity) * 255.0)
            g = int(clamp(0.0, 1.0, 1.0 - abs(norm_velocity)) * 255.0)
            b = int(abs(clamp(- 1.0, 0.0, - 1.0 - norm_velocity)) * 255.0)
            self.debug.draw_point(
                radar_data.transform.location + fw_vec,
                size=0.075,
                life_time=0.06,
                persistent_lines=False,
                color=carla.Color(r, g, b))

# ==============================================================================
# -- CameraManager -------------------------------------------------------------
# ==============================================================================


class CameraManager(object):
    def __init__(self, parent_actor, hud, gamma_correction):
        self.sensor = None
        self.surface = None
        self._parent = parent_actor
        self.hud = hud
        self.recording = False
        bound_y = 0.5 + self._parent.bounding_box.extent.y
        Attachment = carla.AttachmentType
        self._camera_transforms = [
            (carla.Transform(carla.Location(x=-5.5, z=2.5), carla.Rotation(pitch=8.0)), Attachment.SpringArm),
            (carla.Transform(carla.Location(x=1.6, z=1.7)), Attachment.Rigid),
            (carla.Transform(carla.Location(x=5.5, y=1.5, z=1.5)), Attachment.SpringArm),
            (carla.Transform(carla.Location(x=-8.0, z=6.0), carla.Rotation(pitch=6.0)), Attachment.SpringArm),
            (carla.Transform(carla.Location(x=-1, y=-bound_y, z=0.5)), Attachment.Rigid)]
        self.transform_index = 1
        self.sensors = [
            ['sensor.camera.rgb', cc.Raw, 'Camera RGB', {}],
            ['sensor.camera.depth', cc.Raw, 'Camera Depth (Raw)', {}],
            ['sensor.camera.depth', cc.Depth, 'Camera Depth (Gray Scale)', {}],
            ['sensor.camera.depth', cc.LogarithmicDepth, 'Camera Depth (Logarithmic Gray Scale)', {}],
            ['sensor.camera.semantic_segmentation', cc.Raw, 'Camera Semantic Segmentation (Raw)', {}],
            ['sensor.camera.semantic_segmentation', cc.CityScapesPalette,
                'Camera Semantic Segmentation (CityScapes Palette)', {}],
            ['sensor.lidar.ray_cast', None, 'Lidar (Ray-Cast)', {'range': '50'}],
            ['sensor.camera.dvs', cc.Raw, 'Dynamic Vision Sensor', {}],
            ['sensor.camera.rgb', cc.Raw, 'Camera RGB Distorted',
                {'lens_circle_multiplier': '3.0',
                'lens_circle_falloff': '3.0',
                'chromatic_aberration_intensity': '0.5',
                'chromatic_aberration_offset': '0'}]]
        world = self._parent.get_world()
        bp_library = world.get_blueprint_library()
        for item in self.sensors:
            bp = bp_library.find(item[0])
            if item[0].startswith('sensor.camera'):
                bp.set_attribute('image_size_x', str(hud.dim[0]))
                bp.set_attribute('image_size_y', str(hud.dim[1]))
                if bp.has_attribute('gamma'):
                    bp.set_attribute('gamma', str(gamma_correction))
                for attr_name, attr_value in item[3].items():
                    bp.set_attribute(attr_name, attr_value)
            elif item[0].startswith('sensor.lidar'):
                self.lidar_range = 50

                for attr_name, attr_value in item[3].items():
                    bp.set_attribute(attr_name, attr_value)
                    if attr_name == 'range':
                        self.lidar_range = float(attr_value)


            item.append(bp)
        self.index = None

    def toggle_camera(self):
        self.transform_index = (self.transform_index + 1) % len(self._camera_transforms)
        self.set_sensor(self.index, notify=False, force_respawn=True)

    def set_sensor(self, index, notify=True, force_respawn=False):
        index = index % len(self.sensors)
        needs_respawn = True if self.index is None else \
            (force_respawn or (self.sensors[index][2] != self.sensors[self.index][2]))
        if needs_respawn:
            if self.sensor is not None:
                self.sensor.destroy()
                self.surface = None
            self.sensor = self._parent.get_world().spawn_actor(
                self.sensors[index][-1],
                self._camera_transforms[self.transform_index][0],
                attach_to=self._parent,
                attachment_type=self._camera_transforms[self.transform_index][1])
            # We need to pass the lambda a weak reference to self to avoid
            # circular reference.
            weak_self = weakref.ref(self)
            self.sensor.listen(lambda image: CameraManager._parse_image(weak_self, image))
        if notify:
            self.hud.notification(self.sensors[index][2])
        self.index = index

    def next_sensor(self):
        self.set_sensor(self.index + 1)

    def toggle_recording(self):
        self.recording = not self.recording
        self.hud.notification('Recording %s' % ('On' if self.recording else 'Off'))

    def render(self, display):
        if self.surface is not None:
            display.blit(self.surface, (0, 0))

    @staticmethod
    def _parse_image(weak_self, image):
        self = weak_self()
        if not self:
            return
        if self.sensors[self.index][0].startswith('sensor.lidar'):
            points = np.frombuffer(image.raw_data, dtype=np.dtype('f4'))
            points = np.reshape(points, (int(points.shape[0] / 4), 4))
            lidar_data = np.array(points[:, :2])
            lidar_data *= min(self.hud.dim) / (2.0 * self.lidar_range)
            lidar_data += (0.5 * self.hud.dim[0], 0.5 * self.hud.dim[1])
            lidar_data = np.fabs(lidar_data)  # pylint: disable=E1111
            lidar_data = lidar_data.astype(np.int32)
            lidar_data = np.reshape(lidar_data, (-1, 2))
            lidar_img_size = (self.hud.dim[0], self.hud.dim[1], 3)
            lidar_img = np.zeros((lidar_img_size), dtype=np.uint8)
            lidar_img[tuple(lidar_data.T)] = (255, 255, 255)
            self.surface = pygame.surfarray.make_surface(lidar_img)
        elif self.sensors[self.index][0].startswith('sensor.camera.dvs'):
            # Example of converting the raw_data from a carla.DVSEventArray
            # sensor into a NumPy array and using it as an image
            dvs_events = np.frombuffer(image.raw_data, dtype=np.dtype([
                ('x', np.uint16), ('y', np.uint16), ('t', np.int64), ('pol', np.bool)]))
            dvs_img = np.zeros((image.height, image.width, 3), dtype=np.uint8)
            # Blue is positive, red is negative
            dvs_img[dvs_events[:]['y'], dvs_events[:]['x'], dvs_events[:]['pol'] * 2] = 255
            self.surface = pygame.surfarray.make_surface(dvs_img.swapaxes(0, 1))
        else:
            image.convert(self.sensors[self.index][1])
            array = np.frombuffer(image.raw_data, dtype=np.dtype("uint8"))
            array = np.reshape(array, (image.height, image.width, 4))
            array = array[:, :, :3]
            array = array[:, :, ::-1]
            self.surface = pygame.surfarray.make_surface(array.swapaxes(0, 1))
        if self.recording:
            image.save_to_disk('_out/%08d' % image.frame)


# ==============================================================================
# -- game_loop() ---------------------------------------------------------------
# ==============================================================================
class CustomFormatter(logging.Formatter):
    grey = "\x1b[38;20m"
    yellow = "\x1b[33;20m"
    red = "\x1b[31;20m"
    bold_red = "\x1b[31;1m"
    reset = "\x1b[0m"
    format = "%(asctime)s - %(levelname)s - %(message)s (%(filename)s:%(lineno)d)"

    FORMATS = {
        logging.DEBUG: grey + format + reset,
        logging.INFO: grey + format + reset,
        logging.WARNING: yellow + format + reset,
        logging.ERROR: red + format + reset,
        logging.CRITICAL: bold_red + format + reset
    }

    def format(self, record):
        log_fmt = self.FORMATS.get(record.levelno)
        formatter = logging.Formatter(log_fmt)
        return formatter.format(record)

#SAE J275 DECODER
import J2735_201603_combined_voices_mr_fix as J2735
import socket
import json
import csv
import binascii as ba
from time import sleep
import readline
import pandas as pd
from configparser import ConfigParser
import json, threading, time
import pickle
#import matplotlib.pyplot as plt

# Speed planner and low-level throttle/brake steering controller
from ORNL_utils import get_closest_intersection_carla, draw_box, configToDict, get_advisory_speed, getGreenWindow, lat_long_to_xyz_better, process_SPaT, process_BSM, decode_map, search_target_index, search_target_index_v2, determine_signal_phase_from_map_latlon, determine_leader, vehicle_logger, determine_leader_carla
from controller_ts import VehiclePIDController
from speed_control_implementation_ggg import IntelligentDriverModel

update_gap = 1


def _spat_log_path_for_now():
    timestamp = datetime.datetime.now().strftime("%Y%m%d_%H")
    filename = f"spat_record_log_{timestamp}.pkl"
    return os.path.join(SCRIPT_DIR, filename)


def _append_spat_record_snapshot(spat_record, log_file):
    snapshot = {
        "timestamp": time.time(),
        "record": spat_record,
    }
    pickle.dump(snapshot, log_file, protocol=pickle.HIGHEST_PROTOCOL)
    log_file.flush()

# Setup specs for eco-driving planner
RefSpd, ref_cache = 0, 0
speed, speed_cache = 0, 0
x, y = 0, 0
x1, y1 = 0, 0
ego_lat, ego_long = 0, 0
spatInfo = map_info = {}
SPaT_flag, BSM_flag = False, False

info_lock = threading.Lock()
shutdown = threading.Event()

# Config a logger
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s - %(levelname)s - %(message)s"
)
logger = logging.getLogger(__name__)

# # create console handler with a higher log level
# ch = logging.StreamHandler()
# ch.setLevel(logging.INFO)
# ch.setFormatter(CustomFormatter())

# logger.addHandler(ch)

BSM_interval, SPaT_interval, MAP_interval = [], [], []
last_time_BSM, last_time_SPaT, last_time_MAP = None, None, None

#####################################
# ----- receive_loop_thread ------- #
#####################################

def receive_loop(args):
    global SPaT_flag, BSM_flag, speed, speed_cache, x1, y1, spatInfo, spatCache, SPaT_Record, hex_data, map_info, intersection_id, logger, ego_lat, ego_long, last_time_SPaT, SPaT_interval, last_time_BSM, BSM_interval, MAP_Record
    spat_log_file = None
    try:
        # Set the UDP specs
        UDP_IP = "10.7.153.56" ##"10.7.108.81"
        UDP_PORT = 5398

        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.bind((UDP_IP, UDP_PORT))
        logger.info(f"[Receive] Listening on {UDP_IP}:{UDP_PORT}")
        # print(f"[Receive] Listening on {UDP_IP}:{UDP_PORT}")

        # Set a SPaT data to continue running the car
        #spatCache = {}
        spatCache = {'currentTime': 50924, 'status': 'green', 't1s': 50924, 't1e': 50939, 't2s': 50969, 't2e': 51009, 'r1s': 50939}
        SPaT_Record = {str(i): spatCache for i in range(1, 9)} # 8 intersections
        MAP_Record = {str(i): 2 for i in range(1, 9)} # assume all intersections have 2 signal groups
        
        if args.outfile:
            try:
                spat_log_path = _spat_log_path_for_now()
                spat_log_file = open(spat_log_path, "ab")
                logger.info(f"[Receive] SPaT record logging to {spat_log_path}")
            except Exception as e:
                logger.error(f"[Receive] Failed to open SPaT record log file: {e}")

        while True:
            if shutdown.is_set():
                break

            # with info_lock:
            data, addr = sock.recvfrom(4096)
            #print(f"[Receive] Packet received from {addr}, length: {len(data)} bytes")
            # logger.info(f"Packet received from {addr}, length: {len(data)} bytes")
            hex_data = data.hex()

            ## first receive to get SPaT
            # map_info = decode_map(hex_data)
            try:
                SPaT_flag, spatInfo, intersection_id = process_SPaT(hex_data, MAP_Record=MAP_Record)
            except Exception as e:
                logger.error(f"Error processing SPaT data: {e}")
                SPaT_flag, spatInfo, intersection_id = process_SPaT(hex_data, MAP_Record=None)
            
            if SPaT_flag:
                logger.info(f'**************** SPaT data updated for {str(intersection_id)}: {spatCache}')
                SPaT_Record[str(intersection_id)] = spatInfo
                if last_time_SPaT:
                    SPaT_interval.append(time.perf_counter() - last_time_SPaT)
                last_time_SPaT = time.perf_counter()
            else:
                print('..............No new SPaT info extracted................')

            ## second receive to get BSM
            #data, addr = sock.recvfrom(4096)
            #hex_data = data.hex()

            BSM_flag, x1, y1, speed = process_BSM(hex_data)
            if BSM_flag:
                ego_lat, ego_long = x1, y1
                print('BSM return: ', BSM_flag, x1, y1, speed)
                BSM_interval.append(time.perf_counter() - last_time_BSM) if last_time_BSM else None
                last_time_BSM = time.perf_counter()
            if args.outfile and spat_log_file is not None:
                try:
                    _append_spat_record_snapshot(SPaT_Record, spat_log_file)
                except Exception as e:
                    logger.error(f"[Receive] Failed to write SPaT record snapshot: {e}")
                    spat_log_file = None

    except Exception as e:
        #print(f"[Receive] Exception: {e}")
        logger.error(f"Exception in receive_loop: {e}")
    finally:
        if args.outfile and spat_log_file is not None:
            spat_log_file.close()
        sock.close()
        logger.info("[Receive] Socket closed in receive_loop.")
        #print("[Receive] Stopped")
    

#####################################
# ----- carla_loop_thread --------- #
#####################################

def game_loop(args):
    global speed, speed_cache, speed_lead, BSM_flag, SPaT_flag, x, y, x1, y1, spatInfo, spatCache, SPaT_Record, vehicle_logger, intersection_id, logger, RefSpd, ego_lat, ego_long, last_time_MAP, MAP_interval, MAP_Record
    pygame.init()
    pygame.font.init()
    world = None

    ego_speed_buffer = []
    Data4JH = []

    if args.outfile:
        timestamp_temp = datetime.datetime.now().strftime("%Y%m%d_%H")
        outDataFile = f"vehRecord_{timestamp_temp}.csv"
        vehicle_logger = vehicle_logger(outDataFile)

    pass_or_not = 0
    last_dist2bar = 1e6
    record_freq = 2

    speed_lead = 10

    cache_time, last_loop_time = datetime.datetime.now().timestamp(), 0
    wp_id_cache, wp_id = 0, 0
    reference_timestamp = datetime.datetime.strptime('06:30:00', '%H:%M:%S')

    df_waypoints = pd.read_csv('../../json_scripts/delave_waypoints.csv') # Get record waypoints
    cx, cy, cz = df_waypoints['y'].to_numpy(), df_waypoints['x'].to_numpy(), df_waypoints['z'].to_numpy()
    # df_waypoints = pd.read_csv('../../json_scripts/delave_waypoints_leftlane_v1.csv') # Get record waypoints
    # cx, cy, cz = df_waypoints['x'].to_numpy(), -df_waypoints['y'].to_numpy(), df_waypoints['z'].to_numpy()
    c_pitch, c_yaw, c_roll = df_waypoints['pitch'].to_numpy(), df_waypoints['yaw'].to_numpy(), df_waypoints['roll'].to_numpy()
    c_distance = df_waypoints['distance_traveled_m'].to_numpy()
    spawn_pos = carla.Transform(carla.Location(x=-726.36, y=740.29, z=3.0), carla.Rotation(pitch=0.0, yaw=18, roll=0.0))
    distance_traveled = np.hypot(spawn_pos.location.x - cx[0], spawn_pos.location.y - cy[0])

    best_phase = None
    closest_intersection_id, best_SG_id = '8', 2

    veh_coords= {}

    try:
        client = carla.Client(args.host, args.port)
        client.set_timeout(2.0)

        display = pygame.display.set_mode(
            (args.width, args.height),
            pygame.HWSURFACE | pygame.DOUBLEBUF)

        # Set up the world client and vehicle low-level controller
        hud = HUD(args.width, args.height)
        world = World(client.get_world(), hud, spawn_pos, args)
        controller = KeyboardControl(world, args.autopilot)

        clock = pygame.time.Clock()

        run_step = 0

        while True:
            if shutdown.is_set():
                break

            clock.tick_busy_loop(60)
            #ref_trans = world.player.get_transform()
            ref_rotation = world.player.get_transform().rotation
            
            actor_list = world.world.get_actors()
            actor_list = actor_list.filter('vehicle.*')

            #with info_lock:
            # SPaT_flag, spatInfo, intersection_id = process_SPaT(hex_data)
            # BSM_flag, x1, y1, speed = process_BSM(hex_data)

            if BSM_flag is True:
                speed_cache = speed
            else:
                speed = speed_cache
            if spatInfo:
                spatCache = spatInfo

            ## Hardcoded FHWA vehicle as the leader
            for actor in actor_list:
                #print(actor.id, actor.type_id)
                #if actor.type_id == 'vehicle.toyota.prius':
                # veh_coords[actor.attributes['role_name']] = (actor.get_transform().location.x, actor.get_transform().location.y, actor.get_transform().rotation.yaw)
                if actor.attributes['role_name'] == 'FHWA-M-3':
                    # Use actual name
                    ref_trans1 = actor.get_transform()
                    x, y = ref_trans1.location.x, ref_trans1.location.y
                    ref_rotation = actor.get_transform().rotation
                    speed_lead = np.sqrt(actor.get_velocity().x**2 + actor.get_velocity().y**2)
                    #print(actor.attributes)
                    #print('From carla speed: ', np.sqrt(actor.get_velocity().x**2 + actor.get_velocity().y**2))
                    break

            ## Compute info for speed planning
            speed = speed_lead  ##### Just a fake value to run
            x_ego, y_ego, z_ego = world.player.get_transform().location.x, world.player.get_transform().location.y, world.player.get_transform().location.z
            speed_ego = np.sqrt(world.player.get_velocity().x**2 + world.player.get_velocity().y**2)
            accel_ego = np.sqrt(world.player.get_acceleration().x**2 + world.player.get_acceleration().y**2)
            spacing = np.sqrt((x-x_ego)**2 + (y-y_ego)**2) - 5
            # spacing_bsm = np.sqrt((x1-x_ego)**2 + (y1-y_ego)**2) - 5
            #dist2bar = np.sqrt((barPos_x-x_ego)**2 + (barPos_y-y_ego)**2) - 3.5
            dist2bar = 500      ##### Just a fake value to run
            speed_diff = speed - speed_ego
            carla_info = {'pos_ego': (x_ego, y_ego, z_ego), 'spacing': spacing, 'heading': world.player.get_transform().rotation.yaw, 'speed_ego': speed_ego,
                        'pos_lead': (x, y), 'heading_lead': ref_rotation.yaw, 'speed_lead': speed}

            this_loop_time = datetime.datetime.now().timestamp()
            distance_traveled += speed_ego * (this_loop_time - last_loop_time)
            last_loop_time = this_loop_time

            closest_intersection_id_carla, closest_intersection_dist_carla = get_closest_intersection_carla(world.player.get_transform())

            # vehicle_logger.record([x1, y1, world.player.get_transform().rotation.yaw, speed_ego, accel_ego, RefSpd*1.6/3.6])

            #print('Carla speed: ', speed_lead, ' BSM speed: ', speed)
            #print('Carla spacing: ', spacing, ' BSM spacing: ', spacing_bsm)

            try:
                best_leader = determine_leader(world.player.get_transform().location, bsm_message=hex_data, ego_heading=world.player.get_transform().rotation.yaw, carla_info=carla_info)
                if best_leader:
                    #logger.info('Best leader from BSM: ', best_leader['bsm_id'], best_leader['distance'], best_leader['lead_speed'])
                    print('Best leader from BSM: ', best_leader['bsm_id'], best_leader['distance'], best_leader['lead_speed'])
                    # print("Update leader info from BSM!")
                else:
                    logger.warning('...............No leader found from BSM, using cached values..................')
                    best_leader = {'distance': 250, 'lead_speed': 20}
            except Exception as e:
                logger.error(f"[Carla] Finding leader exception: {e}")
                #best_leader = None
                best_leader = {'distance': 250, 'lead_speed': 20}

            try:
                best_leader_carla = determine_leader_carla(world.player.get_transform(), actor_list)
                if best_leader_carla:
                    temp_dist = best_leader_carla[1]
                    temp_speed = np.hypot(best_leader_carla[0].get_velocity().x, best_leader_carla[0].get_velocity().y)
                    best_leader = {'distance': temp_dist, 'lead_speed': temp_speed}
                    logger.info(f'Best leader from Carla: {best_leader_carla[0].attributes["role_name"]}, {best_leader["distance"]}, {best_leader["lead_speed"]}')
                else:
                    logger.warning('....................No leader found from Carla........................')
                    best_leader = {'distance': 250, 'lead_speed': 20}
            except Exception as e:
                logger.error(f"[Carla] Finding leader from Carla exception: {e}")
                best_leader = {'distance': 250, 'lead_speed': 20}

            try:
                #best_phase = determine_signal_phase_from_map(world.player.get_transform().location, ego_latlong=(x1, y1), map_message=hex_data, ego_heading=world.player.get_transform().rotation.yaw)
                best_phase = determine_signal_phase_from_map_latlon(ego_latlon=(ego_lat, ego_long), map_message=hex_data, ego_heading=world.player.get_transform().rotation.yaw)
                if best_phase:
                    #logger.info(f'################Best signal phase from MAP: {best_phase}')
                    # print("################# Update phase group info from MAP! ", best_phase)
                    closest_intersection_id, best_SG_id, closest_intersection_dist, dist2bar = best_phase['intersection_id'], best_phase['signal_group'], best_phase['intersection_distance'], best_phase['dist2StopLine']
                    logger.info(f'Closest intersection Carla: {closest_intersection_id_carla}, {closest_intersection_dist_carla}, Best Phase Group: {best_SG_id}, Closest intersection MAP: {closest_intersection_id}, {closest_intersection_dist}, {dist2bar}')
                    MAP_interval.append(time.perf_counter() - last_time_MAP) if last_time_MAP else None
                    last_time_MAP = time.perf_counter()
                    MAP_Record[str(closest_intersection_id)] = best_SG_id
                else:
                    logger.warning('No updated signal phase group from MAP.')
            except Exception as e:
                logger.error(f"[Carla] Finding signal phase exception: {e}")
                best_phase = None
            # logger.info('Best phase: ', best_phase)

            if args.outfile:
                if best_leader:
                    vehicle_logger.record([x1, y1, world.player.get_transform().rotation.yaw, speed_ego, accel_ego, RefSpd*1.6/3.6, best_leader['lead_speed'], best_leader['distance'], closest_intersection_dist_carla])
                else:
                    vehicle_logger.record([x1, y1, world.player.get_transform().rotation.yaw, speed_ego, accel_ego, RefSpd*1.6/3.6, np.nan, np.nan, closest_intersection_dist_carla])

            # print('Ego Carla Coordinate: ', world.player.get_transform().location.x, world.player.get_transform().location.y, world.player.get_transform().location.z)
            # print('Leader Carla Coordinate: ', x, y)

            ## Too large spacing set to nan
            # if spacing > 75 or np.isnan(speed):
            #     speed = np.nan
            #     spacing = np.nan

            ## Pass intersection stop bar or not    
            if controller.eco_drive and pass_or_not == 0 and closest_intersection_id_carla is '9':
                pass_or_not = 1

            # logger.info(f'Closest intersection Carla: {closest_intersection_id_carla}, {closest_intersection_dist_carla},  Closest intersection MAP: {closest_intersection_id}, {closest_intersection_dist}')
            
            int_intersect_id = min(int(closest_intersection_id_carla), int(closest_intersection_id))
            spatCache = SPaT_Record.get(str(int_intersect_id), spatCache)
            ## Record latest spat just in case
            # if str(intersection_id) == closest_intersection_id:
            #     spatCache = spatInfo
            #     logger.info(f'**************** SPaT data updated: {spatCache}')
                #logger.info('SPaT data updated for speed planning.')
                #print(spatInfo)
            # else:
            #     logger.warning("Using previous SPaT data for speed planning.")
                #print('########################## Use previous SPaT! ##########################')

            try:
                ## update reference speed every 0.2 secs
                current_update_time = datetime.datetime.now().timestamp()
                dt = current_update_time - cache_time

                #print(speed_ego, accel_ego, dist2bar, speed, spacing, pass_or_not, reference_timestamp, spatCache)

                if dt >= 0.2:
                    ##if approaching intersection, use eco-approaching algorithm
                    if not pass_or_not:
                        # RefSpd, dataToSave, errFlag = get_advisory_speed(speed_ego*3.6/1.6, accel_ego, closest_intersection_dist*3.28, speed*3.6/1.6, spacing*3.28, reference_timestamp, spatCache)
                        RefSpd, dataToSave, errFlag = get_advisory_speed(speed_ego*3.6/1.6, accel_ego, (closest_intersection_dist_carla-4)*3.28, best_leader['lead_speed']*3.6/1.6, (best_leader['distance']-4.5)*3.28, reference_timestamp, spatCache)
                        logger.info('....................Use the latest SPaT to update eco-driving speed!.........................')
                    ##if passed intersection, use CF model
                    else:
                        logger.info(f'****************Do CF with spd cmd {speed_ego:.2f}, lead spd {speed:.2f}, spacing: {spacing:.2f}********************************')
                        # _, RefSpd = IntelligentDriverModel(speed_ego*3.6/1.6, 20, speed*3.6/1.6, spacing*3.28)
                        _, RefSpd = IntelligentDriverModel(speed_ego*3.6/1.6, 20, best_leader['lead_speed']*3.6/1.6, (best_leader['distance']-4)*3.28)
                    RefSpd = min(30, RefSpd)
                    cache_time = datetime.datetime.now().timestamp()
            except Exception as e:
                logger.error(f"[Carla] Speed planning exception: {e}")
                logger.info('Cannot get advisory speed! Set to speed limit!')
                #print(f"[Carla] Speed planning exception: {e}")
                #print('------------------------ Cannot get advisory speed!!! Set to speed limit!!! ------------------------')
                RefSpd = 30

            #print('At time: ', reference_timestamp)
            #print(spatCache)
            print(f'-------------------- Ego speed: {speed_ego*3.6/1.6}mph;  Reference speed: {RefSpd}mph;  Lead speed: {speed*3.6/1.6}mph ----------------------')
            #print(f'-------------------- Gap: {spacing}m;  Speed diff: {speed_difference}m/s; Travel distance: {distance_traveled}m; To stopbar: {dist2bar}m --------------------------')
            
            #print(controller.eco_drive)

            ## To compensate early start in recorded testing scenario
            #if spacing >= 2 and speed_diff >= 1 and RefSpd<=0.1:
                #RefSpd = speed*3.6/1.6

            ## Get the desired waypoint
            if controller.eco_drive:
                wp_id = search_target_index(cx, cy, world.player.get_transform(), RefSpd*1.6/3.6)
                # wp_id = search_target_index_v2(cx, cy, world.player.get_transform(), RefSpd*1.6/3.6, c_distance, distance_traveled)
                #wp_id = search_target_index_lookBack(cx, cy, actor.get_transform(), speed_ego)
            
            #print('########### wp id: ' + str(wp_id))
            ## get the reference transformation
            ref_trans = carla.Transform(carla.Location(cx[wp_id],cy[wp_id],float(cz[wp_id])), ref_rotation)
            
            ## Compute the desired reference speed in km per hr
            speed2go = RefSpd*1.6
            ## collision consideration
            # if speed2go/3.6<0.1 or (spacing <= 1 and speed_diff <= 0) or spacing <= 2 or wp_id >= len(cx)-1:
            if speed2go/3.6<0.1 or (best_leader['distance']-4 <= 1 and best_leader['lead_speed']-speed_ego <= 0) or best_leader['distance']-4 <= 2 or wp_id >= len(cx)-1:
                #controller._control.brake = 0.99
                speed2go = 0

            ## Controller execution
            if controller.parse_events(client, world, clock, speed2go, ref_trans, args):
                return
            logger.info(f'Speed2Go: {speed2go:.2f}, Control command: Throttle {controller._control.throttle:.2f}, Brake {controller._control.brake:.2f}, Steer {controller._control.steer:.2f}')
            #print('Ego pos: ' + str(world.player.get_transform().energy-int-test-new-em.configlocation.x) + ', ' + str(world.player.get_transform().location.y) + ', ' + str(world.player.get_transform().location.z))
            #print('Ego ang: ' + str(world.player.get_transform().rotation.pitch) + ', ' + str(world.player.get_transform().rotation.yaw) + ', ' + str(world.player.get_transform().rotation.roll))
            #print('Ref pos: ' + str(ref_trans.location.x) + ', ' + str(ref_trans.location.y))

            #ego_speed_buffer.append(speed_ego)
            #Data4JH.append(dataToSave)

            #draw_box(world.world, x1, y1, 240)

            world.tick(clock)
            world.render(display)
            pygame.display.flip()

    
    except Exception as e:
        logger.error(f"[Carla] Exception: {e}")
        #print(f"[Carla] Exception: {e}")
    finally:
        #np.save('./JHDEBUG.npy', Data4JH)
        if (world and world.recording_enabled):
            client.stop_recorder()

        if world is not None:
            world.destroy()

        pygame.quit()
        logger.info("[Carla] Stopped")
        #print("[Carla] Stopped")

        '''
        plt.figure()
        plt.plot(ego_speed_buffer)
        plt.xlabel('Time step')
        plt.ylabel('Speed (m/s)')
        '''

# ==============================================================================
# -- main() --------------------------------------------------------------------
# ==============================================================================


def main():
    argparser = argparse.ArgumentParser(
        description='CARLA Manual Control Client')
    argparser.add_argument(
        '-v', '--verbose',
        action='store_true',
        dest='debug',
        help='print debug information')
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
        '-a', '--autopilot',
        action='store_true',
        help='enable autopilot')
    argparser.add_argument(
        '--res',
        metavar='WIDTHxHEIGHT',
        default='1280x720',
        help='window resolution (default: 1280x720)')
    argparser.add_argument(
        '--filter',
        metavar='PATTERN',
        default='vehicle.*',
        help='actor filter (default: "vehicle.*")')
    argparser.add_argument(
        '--rolename',
        metavar='NAME',
        default='ORNL-AUTO-1',
        help='actor role name (default: "hero")')
    argparser.add_argument(
        '--gamma',
        default=2.2,
        type=float,
        help='Gamma correction of the camera (default: 2.2)')
    argparser.add_argument(
        '-o', '--outfile',
        action='store_true',
        help = 'Check if we need to store data'
    )
    argparser.add_argument(
        '--x', type=float,
        help='x coordinate of the spawn point')
    argparser.add_argument(
        '--y', type=float, 
        help='y coordinate of the spawn point')
    argparser.add_argument(
        '--z', type=float,
        help='z coordinate of the spawn point')
    args = argparser.parse_args()

    args.width, args.height = [int(x) for x in args.res.split('x')]

    #log_level = logging.DEBUG if args.debug else logging.INFO
    #logging.basicConfig(format='%(levelname)s: %(message)s', level=log_level)

    logger.info('listening to server %s:%s', args.host, args.port)

    print(__doc__)

    thread_recv = threading.Thread(target=receive_loop, args=(args,) daemon=True)
    thread_carla = threading.Thread(target=game_loop, args=(args,), daemon=True)

    thread_recv.start()
    time.sleep(1.0)
    thread_carla.start()

    try:
        while thread_recv.is_alive() and thread_carla.is_alive():
            sleep(0.01)
    except KeyboardInterrupt:
        logger.error('\n [Main] Ctrl+C Cancelled by user. Bye!')
        #print('\n [Main] Ctril+C Cancelled by user. Bye!')
    finally:
        shutdown.set()
        thread_recv.join(timeout=1.0)
        thread_carla.join(timeout=1.0)
        logger.info('[Main] Exit!')
        logger.info(f'Stats of SPaT interval: mean {np.mean(SPaT_interval) if SPaT_interval else None} s, std {np.std(SPaT_interval) if SPaT_interval else None} s')
        logger.info(f'Stats of BSM interval: mean {np.mean(BSM_interval) if BSM_interval else None} s, std {np.std(BSM_interval) if BSM_interval else None} s')
        logger.info(f'Stats of MAP interval: mean {np.mean(MAP_interval) if MAP_interval else None} s, std {np.std(MAP_interval) if MAP_interval else None} s')
        #print('[Main] Exit!')


if __name__ == '__main__':

    main()
