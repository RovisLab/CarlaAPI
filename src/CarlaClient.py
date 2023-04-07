import os
import sys
import glob
import time
import threading
import random
import cv2
import numpy as np
import time

import config_utils as conf
from sensor_lib.Sensors import Sensors
from src.CustomSignal import CustomSignal

try:
    sys.path.append(glob.glob('%s/PythonAPI/carla/dist/carla-*%d.%d-%s.egg' % (
        conf.CARLA_PATH,
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass
import carla

try:
    import pygame
    from pygame.locals import K_ESCAPE
    from pygame.locals import K_SPACE
    from pygame.locals import K_q
    from pygame.locals import K_a
    from pygame.locals import K_d
    from pygame.locals import K_s
    from pygame.locals import K_w
except ImportError:
    raise RuntimeError('cannot import pygame, make sure pygame package is installed')


class CarlaClient(object):
    data_is_ready_signal = CustomSignal()
    terminate_signal = CustomSignal()

    def __init__(self, name, world):
        # Member variables
        self.name = name
        self.args = conf.Clients[self.name]
        self.world = world
        self.using_carla_ai = False
        self.waiting_for_tick = True

        self.height = self.args.cam_height
        self.width = self.args.cam_width
        self.fov = self.args.cam_fov

        # Set actuator name
        self.actuator_name = ''
        if self.args['control'] == 'rovis':
            for sen_name in self.args.Comm:
                if self.args.Comm[sen_name]['type'] == 'actuator':
                    self.actuator_name = sen_name
                    break

        # Flag for actor termination
        self.is_terminated = False

        # Get spawn point
        if self.args.random_spawn:
            spawn_points = self.world.get_map().get_spawn_points()
            spawn_point = random.choice(spawn_points) if spawn_points \
                else carla.Transform()
        else:
            spawn_point = carla.Transform(
                carla.Location(*self.args.position),
                carla.Rotation(*self.args.orientation)
            )

        # Instantiate actor
        bp = self.world.get_blueprint_library().filter(self.args.actor_type)[0]
        self.actor = self.world.spawn_actor(bp, spawn_point)

        # Vehicle Physics Control parameters
        # self.physics_control()

        if self.actor is None:
            print(' # Client {}: could not spawn the actor. Exiting..'.format(self.name))
            sys.exit(1)

        self.actor.apply_control(carla.VehicleControl(manual_gear_shift=True, gear=1))
        self.actor.apply_control(carla.VehicleControl(manual_gear_shift=False))

        # Setup sensors
        self.sensors = None
        self.setup_sensors()

        # Setup camera for manual control
        if self.args['control'] == 'manual':
            self.pyg_control_img = cv2.imread('include/pygame_controls.png')

    # Used for synchronisation with the main game loop
    def tick(self):
        self.waiting_for_tick = False

    # Setup sensors
    def setup_sensors(self):
        # Create Sensors object
        self.sensors = Sensors(self.name, self.actor, self.world)

        # Link terminate signal from the Sensors
        self.sensors.terminate_signal.connect(self.on_terminate)

        # Add sensors
        for sensor_name in self.args.Comm:
            self.sensors.add(sensor_name)

        # Setup each sensor
        self.sensors.setup()

        # Set signal if saving data
        if conf.Database['save_data']:
            self.sensors.all_data_is_collected_signal.connect(self.on_data_collected)

        print(' # {} initialised successfully.'.format(self.name))

    def print_ready(self):
        time.sleep(2)
        print(" # {} ready.".format(self.name))

    # Client loop
    def client_loop(self):
        threading.Thread(target=self.print_ready).start()

        # Start sensors
        self.sensors.start()

        # Setup pygame
        pyg_clock = None
        pyg_display = None
        if self.args['control'] == 'manual':
            pygame.init()
            pygame.font.init()
            pyg_shape = self.pyg_control_img.shape
            pyg_display = pygame.display.set_mode((pyg_shape[1], pyg_shape[0]),
                                                  pygame.HWSURFACE | pygame.DOUBLEBUF)
            pyg_clock = pygame.time.Clock()

        # Client loop start here
        while not self.is_terminated:
            # Wait for tick
            while self.waiting_for_tick:
                time.sleep(0.01)
                if self.is_terminated:
                    return
            self.waiting_for_tick = True

            # Update capture flags for sensors
            self.capture()

            # Signal sensors to send data
            self.sensors.send_data()

            # Draw radar on the world
            # self.radar_sensor.draw_radar(self.world)

            # Toggle between control methods
            control = self.args['control']
            if control == 'rovis':
                self.rovis_control(self.actor)
            elif control == 'auto':
                self.carla_ai_control(self.actor)
            elif control == 'manual':
                try:
                    pyg_clock.tick(conf.General['fps'])
                    self.manual_control(self.actor)
                except pygame.error:
                    pass
            elif control == 'static':
                pass  # Do nothing

            # self.custom_viewer()

            if self.args['control'] == 'manual':
                try:
                    self.render(pyg_display, fps=pyg_clock.get_fps())
                    pygame.display.flip()
                    pygame.event.pump()
                    for event in pygame.event.get():
                        if event.type == pygame.QUIT:
                            pygame.display.quit()
                            pygame.quit()
                            self.on_terminate()
                except pygame.error:
                    pass

    # Custom viewer
    def custom_viewer(self):
        # Combines 3D with Camera
        if 'CamFront' not in self.sensors.keys() or 'ObjDet3D' not in self.sensors.keys():
            return

        image = self.sensors.get_data('CamFront')
        bboxes = self.sensors.get_data('ObjDet3D', 'image')

        if image is not None:
            image = self.sensors.get('ObjDet3D').draw_bboxes(bboxes, image.copy())

        if image is not None:
            cv2.imshow('TEST', image)
            cv2.waitKey(1)

    # Update capture flags for sensors
    def capture(self):
        self.sensors.capture()

    # Render method for pygame display
    def render(self, display, fps=-1.0):
        # Method used to display the controls for pygame
        if self.pyg_control_img is not None:
            array = self.pyg_control_img[:, :, ::-1]
            surface = pygame.surfarray.make_surface(array.swapaxes(0, 1))
            display.blit(surface, (0, 0))

            if fps != -1.0:
                my_font = pygame.font.SysFont('Comic Sans MS', 20)
                display.blit(my_font.render(self.name, False, (0, 0, 0)), (4, 0))
                display.blit(my_font.render("%.2f" % fps, False, (0, 0, 0)), (4, 35))

    # Get data from Signals for saving
    def get_saving_data(self, ts_stop, frame_id):
        self.sensors.get_data_to_save(ts_stop, frame_id)

    # Slot that is called when data is ready. Emits "data_is_ready_signal" when done for CarlaClients.
    # def on_data_collected(self, data_collected):
    #     self.data_is_ready_signal.emit(self.name, data_collected)
    def on_data_collected(self, name, data_collected):
        self.data_is_ready_signal.emit(name, data_collected)

    # Control handler for 'rovis'
    def rovis_control(self, car):
        control = car.get_control()

        throttle, steering = self.sensors.get_data(self.actuator_name)

        control.throttle = throttle
        control.steer = -steering

        if throttle < 0:
            control.reverse = True
        else:
            control.reverse = False

        car.apply_control(control)

        return False

    # Control handler for 'auto'
    def carla_ai_control(self, car):
        if self.using_carla_ai is False:
            self.using_carla_ai = True
            car.set_autopilot()
            self.using_carla_ai = True

    # Control handler for 'manual'
    def manual_control(self, car):
        keys = pygame.key.get_pressed()
        if keys[K_ESCAPE] or keys[K_q]:
            pygame.display.quit()
            pygame.quit()
            self.on_terminate()
            return True

        control = car.get_control()

        control.throttle = 0
        if keys[K_w]:
            control.throttle = 1
            control.reverse = False
        elif keys[K_s]:
            control.throttle = 1
            control.reverse = True

        control.steer = 0
        if keys[K_a]:
            control.steer = max(-1., min(control.steer - 0.05, 0))
        elif keys[K_d]:
            control.steer = min(1., max(control.steer + 0.05, 0))

        control.hand_brake = keys[K_SPACE]

        car.apply_control(control)
        return False

    # Set advanced physics controls for the current actor
    def physics_control(self):
        # https://carla.readthedocs.io/en/latest/python_api/#carla.VehiclePhysicsControl

        # Change Vehicle Physics Control parameters of the vehicle
        physics_control = self.actor.get_physics_control()

        # Create Wheels Physics Control
        # tire_friction=3.0
        default_front_wheel = carla.WheelPhysicsControl(tire_friction=999.0, damping_rate=1.0,
                                                        max_steer_angle=70.0, radius=25.0)
        default_back_wheel = carla.WheelPhysicsControl(tire_friction=999.0, damping_rate=1.0,
                                                       max_steer_angle=0.0, radius=25.0)

        # wheels = [front_left_wheel, front_right_wheel, rear_left_wheel, rear_right_wheel]
        wheels = [default_front_wheel, default_front_wheel, default_back_wheel, default_back_wheel]

        # Parameters
        physics_control.torque_curve = [carla.Vector2D(x=0, y=400), carla.Vector2D(x=1300, y=600)]
        physics_control.max_rpm = 10000
        physics_control.moi = 1.0
        physics_control.damping_rate_full_throttle = 0.0
        physics_control.use_gear_autobox = True
        physics_control.gear_switch_time = 0.5
        physics_control.clutch_strength = 10
        physics_control.mass = 10000
        physics_control.drag_coefficient = 0.25
        physics_control.steering_curve = [carla.Vector2D(x=0, y=1), carla.Vector2D(x=100, y=1),
                                          carla.Vector2D(x=300, y=1)]
        physics_control.use_sweep_wheel_collision = True
        physics_control.wheels = wheels

        # Update params
        self.actor.apply_physics_control(physics_control)

    # Pass signal for termination
    def on_terminate(self):
        self.terminate_signal.emit()

    # Terminate client
    def terminate(self):
        # Terminate sensors
        self.sensors.terminate()

        # Terminate control
        if self.args['control'] == 'manual':
            pygame.display.quit()
            pygame.quit()

        # Terminate actor
        try:
            self.actor.destroy()
        except RuntimeError:  # Actor already destroyed
            pass

        self.is_terminated = True


if __name__ == '__main__':
    print('NOT runnable. Check \'Run_Carla.py\'')
