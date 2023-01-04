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

# Importing Carla
try:
    sys.path.append(glob.glob('carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass
import carla

ip_carla = "127.0.0.1"  # Loopback ip for Carla
port_carla = 2000       # Carla port


class CarlaClient(object):
    def __init__(self, name,
                 view_cam=False,
                 position=carla.Location(-75, 115, 0.3),
                 orientation=carla.Rotation(0, -89, 0),
                 random_spawn=False,
                 vehicle_type='vehicle.Seat.Leon',
                 control='auto',  # auto rovis
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

        self.start_time = time.time()
        self.move = False

        # Member variables
        self.name = name
        self.control = control
        self.view_cam = view_cam
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

        # Flag for actor termination
        self.is_terminated = False

        # Get spawn point
        if random_spawn:
            map = self.world.get_map()
            spawn_points = map.get_spawn_points()
            spawn_point = random.choice(spawn_points) if spawn_points \
                else carla.Transform()
        else:
            spawn_point = carla.Transform(position, orientation)

        # Instantiate car
        bp = self.world.get_blueprint_library().filter(vehicle_type)[0]
        self.car = self.world.spawn_actor(bp, spawn_point)

        # Vehicle Physics Control parameters
        # self.physics_control()

        if self.car is None:
            print('{} - Could not spawn the car. Exiting..'.format(self.name))
            sys.exit(1)

        self.car.apply_control(carla.VehicleControl(manual_gear_shift=True, gear=1))
        self.car.apply_control(carla.VehicleControl(manual_gear_shift=False))

        # Setup camera
        if view_cam:
            self.camera = MonoCameraSensor('{}'.format(self.name),
                                           self.car,
                                           self.view_width,
                                           self.view_height,
                                           self.view_fov)
            self.camera.setup_camera(get_transform('default'))

        # Init actuators server
        if port_rovis_actuator is not None:
            self.vehicle_actuator = VehicleActuatorClient(ip=ip_rovis, port=port_rovis_actuator)
            self.vehicle_actuator.init_client()

            if self.control != 'rovis':
                print('{} - In order to use Actuators, you must enable \'rovis\' control.'.format(self.name))
        elif self.control == 'rovis':
            print('{} - \'rovis\' control does not work without enabling actuators filter.'.format(self.name))

        # Init IMU sensor
        if port_rovis_imu is not None or port_rovis_state_measurement is not None:
            self.imu_sensor = IMUSensor(name='{} - IMU'.format(self.name),
                                        car=self.car)
            self.imu_sensor.setup_imu()
            # self.imu_sensor.init_view()

        # Attach IMU server
        if port_rovis_imu is not None:
            self.imu_server = IMUServer(ip=ip_rovis,
                                        port=port_rovis_imu,
                                        dt=0.0,
                                        imu_sensor=self.imu_sensor)
            self.imu_server.init_server()

        # Attach State Measurement server
        if port_rovis_state_measurement is not None:
            self.state_meas_server = StateMeasurementServer(ip=ip_rovis,
                                                            port=port_rovis_state_measurement,
                                                            dt=0.00,
                                                            car=self.car,
                                                            imu=self.imu_sensor,
                                                            client_name=self.name)
            self.state_meas_server.init_server()

        # Attach MonoCameras
        if port_rovis_cam_front is not None:
            self.front_cam_sensor = MonoCameraSensor('{} - FrontCamera'.format(self.name),
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
            self.back_cam_sensor = MonoCameraSensor('{} - BackCamera'.format(self.name),
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
            self.left_cam_sensor = MonoCameraSensor('{} - LeftCamera'.format(self.name),
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
            self.right_cam_sensor = MonoCameraSensor('{} - RightCamera'.format(self.name),
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
            self.back_left_cam_sensor = MonoCameraSensor('{} - BackLeftCamera'.format(self.name),
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
            self.back_right_cam_sensor = MonoCameraSensor('{} - BackRightCamera'.format(self.name),
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
            self.semseg_front_sensor = SemSegCameraSensor('{} - SemSegFront'.format(self.name),
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
            self.semseg_back_sensor = SemSegCameraSensor('{} - SemSegBack'.format(self.name),
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
            self.semseg_left_sensor = SemSegCameraSensor('{} - SemSegLeft'.format(self.name),
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
            self.semseg_right_sensor = SemSegCameraSensor('{} - SemSegRight'.format(self.name),
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
            self.semseg_back_left_sensor = SemSegCameraSensor('{} - SemSegBackLeft'.format(self.name),
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
            self.semseg_back_right_sensor = SemSegCameraSensor('{} - SemSegBackRight'.format(self.name),
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
            self.depth_sensor = DepthSensor(name='{} - RGBD Front'.format(self.name),
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
            self.lidar_sensor = LidarSensor(name='{} - LidarTop'.format(self.name),
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
            self.radar_sensor = RadarSensor(name='{} - RadarFront'.format(self.name),
                                            parent_actor=self.car)
            self.radar_sensor.setup_radar(get_transform('radar'))
            # self.radar_sensor.init_view()

            self.radar_server = RadarServer(ip=ip_rovis,
                                            port=port_rovis_radar,
                                            dt=0.0,
                                            radar_sensor=self.radar_sensor)
            self.radar_server.init_server()

        print(' # {} initialised successfully.'.format(self.name))

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

        print(' # {} successfully terminated.'.format(self.name))

    def print_available_maps(self):
        print([elem.split('/')[-1] for elem in self.client.get_available_maps()])

    def print_ready(self):
        time.sleep(2)
        print(" # {} ready.".format(self.name))

    def game_loop(self):
        threading.Thread(target=self.print_ready()).start()

        if self.view_cam:
            self.camera.init_view()

        while not self.is_terminated:

            self.world.tick()
            # Capture flag for cameras. Without this, the image will not update
            if self.view_cam:
                self.camera.capture = True

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

            # Toggle between control methods (Rovis or Carla Auto Pilot)
            if self.control == 'rovis':
                self.rovis_control(self.car)
            else:  # auto
                self.carla_ai_control(self.car)

            self.world.tick()

    def set_synchronous_mode(self, synchronous_mode):
        settings = self.world.get_settings()
        settings.synchronous_mode = synchronous_mode
        if synchronous_mode:
            settings.fixed_delta_seconds = self.target_freq
        self.world.apply_settings(settings)

    def rovis_control(self, car):
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

    def physics_control(self):
        # https://carla.readthedocs.io/en/latest/python_api/#carla.VehiclePhysicsControl

        # Change Vehicle Physics Control parameters of the vehicle
        physics_control = self.car.get_physics_control()

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
        self.car.apply_physics_control(physics_control)


if __name__ == '__main__':
    print('NOT runnable. Check \'Run_CarlaClients.py\'')
