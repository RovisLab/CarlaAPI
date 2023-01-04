import os
import sys
import glob
import time
import threading
import random
import numpy as np
import time

from clients.VehicleActuatorClient import VehicleActuatorClient
from servers.MonoCameraServer import MonoCameraServer, MonoCameraSensor
from servers.SemSegCameraServer import SemSegCameraServer, SemSegCameraSensor
from servers.VehicleStateEstimationServer import StateMeasurementServer
from servers.ImuServer import IMUServer, IMUSensor
from servers.DepthServer import DepthSensor, DepthServer
from servers.LidarServer import LidarSensor, LidarServer
from servers.RadarServer import RadarSensor, RadarServer
from sensors import *

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

try:
    import cv2
except ImportError:
    raise RuntimeError("cannot import opencv, make sure python OpenCV is installed")

# Importing Carla
try:
    sys.path.append(glob.glob('carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass
import carla


class CarlaClientPygame(object):
    def __init__(self, name,
                 view_cam=False,
                 position=carla.Location(-75, 115, 0.3),
                 orientation=carla.Rotation(0, -89, 0),
                 random_spawn=False,
                 vehicle_type='vehicle.Seat.Leon',
                 town_name='Town03',
                 control='auto',  # auto manual rovis
                 ip_carla='127.0.0.1',
                 port_carla=2000,

                 ip_rovis='127.0.0.1',
                 port_rovis_actuator=None,
                 port_rovis_cam_front=None,
                 port_rovis_cam_back=None,
                 port_rovis_cam_left=None,
                 port_rovis_cam_right=None,
                 port_rovis_cam_back_left=None,
                 port_rovis_cam_back_right=None,
                 port_rovis_semseg_front=None,
                 port_rovis_semseg_back=None,
                 port_rovis_semseg_left=None,
                 port_rovis_semseg_right=None,
                 port_rovis_semseg_back_left=None,
                 port_rovis_semseg_back_right=None,
                 port_rovis_state_measurement=None,
                 port_rovis_imu=None,
                 port_rovis_depth=None,
                 port_rovis_lidar=None,
                 port_rovis_radar=None,

                 target_fps=30,
                 view_width=940,
                 view_height=480,
                 view_fov=90):

        # Member variables
        self.name = name
        self.control = control
        self.view_cam = False  # view_cam
        self.target_fps = target_fps
        self.target_freq = 1.0 / self.target_fps
        self.view_width = view_width
        self.view_height = view_height
        self.view_fov = view_fov
        self.client = carla.Client(ip_carla, port_carla)
        self.client.set_timeout(2.0)
        self.world = self.client.get_world()
        self.using_carla_ai = False

        self.past_steering = 0.

        # Check map
        if self.world.get_map().name != town_name:
            if town_name in [elem.split('/')[-1] for elem in self.client.get_available_maps()]:
                self.client.set_timeout(999)
                self.world = self.client.load_world(town_name)
                self.client.set_timeout(2.0)
                print('{} - Loaded the {} map.'.format(self.name, town_name))
            else:
                print('{} - The map {} is invalid or not existing. Exiting...'.format(self.name, town_name))
        self.map = self.world.get_map()

        # Flag to update PyGame image
        self.capture = True

        # Flag for actor termination
        self.is_terminated = False

        # Get spawn point
        if random_spawn:
            spawn_points = self.map.get_spawn_points()
            spawn_point = random.choice(spawn_points) if spawn_points \
                else carla.Transform()
        else:
            spawn_point = carla.Transform(position, orientation)

        # Instantiate car
        bp = self.world.get_blueprint_library().filter(vehicle_type)[0]
        self.car = self.world.spawn_actor(bp, spawn_point)
        if self.car is None:
            print('CarlaPygame - Could not spawn the car. Exiting..')
            sys.exit(1)

        self.car.apply_control(carla.VehicleControl(manual_gear_shift=True, gear=1))
        self.car.apply_control(carla.VehicleControl(manual_gear_shift=False))

        # Setup camera
        self.camera = MonoCameraSensor('',
                                       self.car,
                                       self.view_width,
                                       self.view_height,
                                       self.view_fov)
        self.camera.setup_camera(get_transform('default'))

        # Init actuators
        if port_rovis_actuator is not None:
            self.vehicle_actuator = VehicleActuatorClient(ip=ip_rovis, port=port_rovis_actuator)
            self.vehicle_actuator.init_client()

            if self.control != 'rovis':
                print('{} - In order to use Actuators, you must enable \'rovis\' control.'.format(self.name))
        elif self.control == 'rovis':
            print('{} - \'rovis\' control does not work without enabling actuators filter.'.format(self.name))
            self.control = 'auto'

        # Attach MonoCameras
        if port_rovis_cam_front is not None:
            self.front_cam_sensor = MonoCameraSensor('CarlaPygame - FrontCamera',
                                                     self.car,
                                                     self.view_width,
                                                     self.view_height,
                                                     self.view_fov)
            self.front_cam_sensor.setup_camera(get_transform('front'))
            # self.front_cam_sensor.init_view()

            self.front_cam_server = MonoCameraServer(ip=ip_rovis,
                                                     port=port_rovis_cam_front,
                                                     dt=0.0,
                                                     cam_sensor=self.front_cam_sensor)
            self.front_cam_server.init_server()

        if port_rovis_cam_back is not None:
            self.back_cam_sensor = MonoCameraSensor('CarlaPygame - BackCamera',
                                                    self.car,
                                                    self.view_width,
                                                    self.view_height,
                                                    self.view_fov)
            self.back_cam_sensor.setup_camera(get_transform('back'))
            # self.back_cam_sensor.init_view()

            self.back_cam_server = MonoCameraServer(ip=ip_rovis,
                                                    port=port_rovis_cam_back,
                                                    dt=0.0,
                                                    cam_sensor=self.back_cam_sensor)
            self.back_cam_server.init_server()

        if port_rovis_cam_left is not None:
            self.left_cam_sensor = MonoCameraSensor('CarlaPygame - LeftCamera',
                                                    self.car,
                                                    self.view_width,
                                                    self.view_height,
                                                    self.view_fov)
            self.left_cam_sensor.setup_camera(get_transform('front_left'))
            # self.left_cam_sensor.init_view()

            self.left_cam_server = MonoCameraServer(ip=ip_rovis,
                                                    port=port_rovis_cam_left,
                                                    dt=0.0,
                                                    cam_sensor=self.left_cam_sensor)
            self.left_cam_server.init_server()

        if port_rovis_cam_right is not None:
            self.right_cam_sensor = MonoCameraSensor('CarlaPygame - RightCamera',
                                                     self.car,
                                                     self.view_width,
                                                     self.view_height,
                                                     self.view_fov)
            self.right_cam_sensor.setup_camera(get_transform('front_right'))
            # self.right_cam_sensor.init_view()

            self.right_cam_server = MonoCameraServer(ip=ip_rovis,
                                                     port=port_rovis_cam_right,
                                                     dt=0.0,
                                                     cam_sensor=self.right_cam_sensor)
            self.right_cam_server.init_server()

        if port_rovis_cam_back_left is not None:
            self.back_left_cam_sensor = MonoCameraSensor('CarlaPygame - BackLeftCamera',
                                                         self.car,
                                                         self.view_width,
                                                         self.view_height,
                                                         self.view_fov)
            self.back_left_cam_sensor.setup_camera(get_transform('back_left'))
            # self.back_left_cam_sensor.init_view()

            self.back_left_cam_server = MonoCameraServer(ip=ip_rovis,
                                                         port=port_rovis_cam_back_left,
                                                         dt=0.0,
                                                         cam_sensor=self.back_left_cam_sensor)
            self.back_left_cam_server.init_server()

        if port_rovis_cam_back_right is not None:
            self.back_right_cam_sensor = MonoCameraSensor('CarlaPygame - BackRightCamera',
                                                          self.car,
                                                          self.view_width,
                                                          self.view_height,
                                                          self.view_fov)
            self.back_right_cam_sensor.setup_camera(get_transform('back_right'))
            # self.back_right_cam_sensor.init_view()

            self.back_right_cam_server = MonoCameraServer(ip=ip_rovis,
                                                          port=port_rovis_cam_back_right,
                                                          dt=0.0,
                                                          cam_sensor=self.back_right_cam_sensor)
            self.back_right_cam_server.init_server()

        # Attach SemSeg Camera
        if port_rovis_semseg_front is not None:
            self.semseg_front_sensor = SemSegCameraSensor('CarlaPygame - SemSegFront',
                                                          self.car,
                                                          self.view_width,
                                                          self.view_height,
                                                          self.view_fov)
            self.semseg_front_sensor.setup_camera(get_transform('front'))
            # self.semseg_front_sensor.init_view()

            self.semseg_front_server = SemSegCameraServer(ip=ip_rovis,
                                                          port=port_rovis_semseg_front,
                                                          dt=0.0,
                                                          cam_sensor=self.semseg_front_sensor)
            self.semseg_front_server.init_server()

        if port_rovis_semseg_back is not None:
            self.semseg_back_sensor = SemSegCameraSensor('CarlaPygame - SemSegBack',
                                                         self.car,
                                                         self.view_width,
                                                         self.view_height,
                                                         self.view_fov)
            self.semseg_back_sensor.setup_camera(get_transform('back'))
            # self.semseg_back_sensor.init_view()

            self.semseg_back_server = SemSegCameraServer(ip=ip_rovis,
                                                         port=port_rovis_semseg_back,
                                                         dt=0.0,
                                                         cam_sensor=self.semseg_back_sensor)
            self.semseg_back_server.init_server()

        if port_rovis_semseg_left is not None:
            self.semseg_left_sensor = SemSegCameraSensor('CarlaPygame - SemSegLeft',
                                                         self.car,
                                                         self.view_width,
                                                         self.view_height,
                                                         self.view_fov)
            self.semseg_left_sensor.setup_camera(get_transform('front_left'))
            # self.semseg_left_sensor.init_view()

            self.semseg_left_server = SemSegCameraServer(ip=ip_rovis,
                                                         port=port_rovis_semseg_left,
                                                         dt=0.0,
                                                         cam_sensor=self.semseg_left_sensor)
            self.semseg_left_server.init_server()

        if port_rovis_semseg_right is not None:
            self.semseg_right_sensor = SemSegCameraSensor('CarlaPygame - SemSegRight',
                                                          self.car,
                                                          self.view_width,
                                                          self.view_height,
                                                          self.view_fov)
            self.semseg_right_sensor.setup_camera(get_transform('front_right'))
            # self.semseg_right_sensor.init_view()

            self.semseg_right_server = SemSegCameraServer(ip=ip_rovis,
                                                          port=port_rovis_semseg_right,
                                                          dt=0.0,
                                                          cam_sensor=self.semseg_right_sensor)
            self.semseg_right_server.init_server()

        if port_rovis_semseg_back_left is not None:
            self.semseg_back_left_sensor = SemSegCameraSensor('CarlaPygame - SemSegBackLeft',
                                                              self.car,
                                                              self.view_width,
                                                              self.view_height,
                                                              self.view_fov)
            self.semseg_back_left_sensor.setup_camera(get_transform('back_left'))
            # self.semseg_back_left_sensor.init_view()

            self.semseg_back_left_server = SemSegCameraServer(ip=ip_rovis,
                                                              port=port_rovis_semseg_back_left,
                                                              dt=0.0,
                                                              cam_sensor=self.semseg_back_left_sensor)
            self.semseg_back_left_server.init_server()

        if port_rovis_semseg_back_right is not None:
            self.semseg_back_right_sensor = SemSegCameraSensor('CarlaPygame - SemSegBackRight',
                                                               self.car,
                                                               self.view_width,
                                                               self.view_height,
                                                               self.view_fov)
            self.semseg_back_right_sensor.setup_camera(get_transform('back_right'))
            # self.semseg_back_right_sensor.init_view()

            self.semseg_back_right_server = SemSegCameraServer(ip=ip_rovis,
                                                               port=port_rovis_semseg_back_right,
                                                               dt=0.0,
                                                               cam_sensor=self.semseg_back_right_sensor)
            self.semseg_back_right_server.init_server()

        # Attach Depth sensor
        if port_rovis_depth is not None:
            self.depth_sensor = DepthSensor(name='CarlaPygame - RGBD Front',
                                            parent_actor=self.car)
            self.depth_sensor.setup_depth(get_transform('depth'))
            # self.depth_sensor.init_view()

            self.depth_server = DepthServer(ip=ip_rovis,
                                            port=port_rovis_depth,
                                            dt=0.0,
                                            depth_sensor=self.depth_sensor)
            self.depth_server.init_server()

        # Attach Lidar sensor
        if port_rovis_lidar is not None:
            self.lidar_sensor = LidarSensor(name='CarlaPygame - LidarTop',
                                            parent_actor=self.car)
            self.lidar_sensor.setup_lidar(get_transform('lidar'))
            # self.lidar_sensor.init_view()

            self.lidar_server = LidarServer(ip=ip_rovis,
                                            port=port_rovis_lidar,
                                            dt=0.0,
                                            lidar_sensor=self.lidar_sensor)
            self.lidar_server.init_server()

        # Attach Radar sensor
        if port_rovis_radar is not None:
            self.radar_sensor = RadarSensor(name='CarlaPygame - RadarFront',
                                            parent_actor=self.car)
            self.radar_sensor.setup_radar(get_transform('radar'))
            # self.radar_sensor.init_view()

            self.radar_server = RadarServer(ip=ip_rovis,
                                            port=port_rovis_radar,
                                            dt=0.0,
                                            radar_sensor=self.radar_sensor)
            self.radar_server.init_server()

        # Attach IMU sensor
        if port_rovis_imu is not None:
            self.imu_sensor = IMUSensor(name='CarlaPygame - IMU',
                                        car=self.car)
            self.imu_sensor.setup_imu()
            # self.imu_sensor.init_view()

            self.imu_server = IMUServer(ip=ip_rovis,
                                        port=port_rovis_imu,
                                        dt=0.0,
                                        imu_sensor=self.imu_sensor)
            self.imu_server.init_server()

        # Attach State measurement sensor
        if port_rovis_state_measurement is not None:
            # Attach IMU sensor if not created
            if port_rovis_imu is None:
                self.imu_sensor = IMUSensor(name='IMU_state_measurement',
                                            car=self.car)
                self.imu_sensor.setup_imu()

            # State measurement server
            self.state_meas_server = StateMeasurementServer(ip=ip_rovis,
                                                            port=port_rovis_state_measurement,
                                                            dt=0.00,
                                                            car=self.car,
                                                            imu=self.imu_sensor,
                                                            client_name='CarlaPygame')
            self.state_meas_server.init_server()

        print(' # ClientPygame initialised successfully.')

    def terminate(self):
        self.car.destroy()
        self.camera.destroy()
        self.is_terminated = True
        return
        self.front_cam_sensor.destroy()
        self.back_cam_sensor.destroy()
        self.vehicle_actuator.is_terminated = True
        self.front_cam_server.is_terminated = True
        self.back_cam_server.is_terminated = True

        print(' # ClientPygame successfully terminated.')

    def print_available_maps(self):
        print([elem.split('/')[-1] for elem in self.client.get_available_maps()])

    def game_loop(self):
        # https://carla.readthedocs.io/en/0.8.4/configuring_the_simulation/#synchronous-vs-asynchronous-mode
        self.set_synchronous_mode(True)

        pygame.init()
        pygame.font.init()
        my_font = pygame.font.SysFont('Comic Sans MS', 30)
        display = pygame.display.set_mode((self.view_width, self.view_height), pygame.HWSURFACE | pygame.DOUBLEBUF)
        pygame_clock = pygame.time.Clock()

        while not self.is_terminated:
            self.world.tick()
            # Capture flag for cameras. Without this, the image will not send
            if hasattr(self, 'front_cam_sensor'):
                self.front_cam_sensor.capture = True
            if hasattr(self, 'back_cam_sensor'):
                self.back_cam_sensor.capture = True
            if hasattr(self, 'left_cam_sensor'):
                self.left_cam_sensor.capture = True
            if hasattr(self, 'right_cam_sensor'):
                self.right_cam_sensor.capture = True
            if hasattr(self, 'back_left_cam_sensor'):
                self.back_left_cam_sensor.capture = True
            if hasattr(self, 'back_right_cam_sensor'):
                self.back_right_cam_sensor.capture = True
            if hasattr(self, 'semseg_front_sensor'):
                self.semseg_front_sensor.capture = True
            if hasattr(self, 'semseg_back_sensor'):
                self.semseg_back_sensor.capture = True
            if hasattr(self, 'semseg_left_sensor'):
                self.semseg_left_sensor.capture = True
            if hasattr(self, 'semseg_right_sensor'):
                self.semseg_right_sensor.capture = True
            if hasattr(self, 'semseg_back_left_sensor'):
                self.semseg_back_left_sensor.capture = True
            if hasattr(self, 'semseg_back_right_sensor'):
                self.semseg_back_right_sensor.capture = True

            # Draw radar on the world
            # self.radar_sensor.draw_radar(self.world)

            # Capture flag for pygame camera.
            # If this flag is not set, the pygame image will not update
            self.camera.capture = True

            pygame_clock.tick(self.target_fps)

            # Toggle between control methods (Rovis, manual or Carla Auto Pilot)
            if self.control == 'rovis':
                self.rovis_control(self.car)
            elif self.control == 'manual':
                self.manual_control(self.car)
            else:  # auto
                self.carla_ai_control(self.car)

            self.world.tick()
            self.render(display, fps=pygame_clock.get_fps())
            pygame.display.flip()
            pygame.event.pump()
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    self.terminate()
        pygame.quit()

    def set_synchronous_mode(self, synchronous_mode):
        settings = self.world.get_settings()
        settings.synchronous_mode = synchronous_mode
        if synchronous_mode:
            settings.fixed_delta_seconds = self.target_freq
        self.world.apply_settings(settings)

    def render(self, display, fps=-1):
        """
        Transforms image from camera sensor and blits it to main pygame display.
        It also displays the FPS.
        """
        array = self.camera.process_image()
        if array is not None:
            array = array[:, :, ::-1]
            surface = pygame.surfarray.make_surface(array.swapaxes(0, 1))
            display.blit(surface, (0, 0))

            if fps != -1:
                my_font = pygame.font.SysFont('Comic Sans MS', 20)
                display.blit(my_font.render("%.2f" % fps, False, (255, 0, 0)), (0, 0))

    def manual_control(self, car):
        """
        Applies control to main car based on pygame pressed keys.
        Will return True If ESCAPE is hit, otherwise False to end main loop.
        """
        keys = pygame.key.get_pressed()
        if keys[K_ESCAPE] or keys[K_q]:
            self.terminate()
            return True

        control = car.get_control()
        control.throttle = 0
        if keys[K_w]:
            control.throttle = 1
            control.reverse = False
        elif keys[K_s]:
            control.throttle = 1
            control.reverse = True
        if keys[K_a]:
            control.steer = max(-1., min(control.steer - 0.05, 0))
        elif keys[K_d]:
            control.steer = min(1., max(control.steer + 0.05, 0))
        else:
            control.steer = 0
        control.hand_brake = keys[K_SPACE]

        car.apply_control(control)
        return False

    def rovis_control(self, car):
        keys = pygame.key.get_pressed()
        if keys[K_ESCAPE] or keys[K_q]:
            self.terminate()
            return True
        control = car.get_control()

        control.throttle = self.vehicle_actuator.throttle
        control.steer = -self.vehicle_actuator.steering

        if self.vehicle_actuator.throttle < 0:
            control.reverse = True
        else:
            control.reverse = False

        car.apply_control(control)
        return False

    def carla_ai_control(self, car):
        if self.using_carla_ai is False:
            self.using_carla_ai = True
            car.set_autopilot()
            self.using_carla_ai = True

        keys = pygame.key.get_pressed()
        if keys[K_ESCAPE] or keys[K_q]:
            self.terminate()
            return True


if __name__ == '__main__':
    print('NOT runnable. Check \'Run_CarlaClients.py\'')
