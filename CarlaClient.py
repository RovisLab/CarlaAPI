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


class CarlaClient(object):
    def __init__(self, name,
                 view_cam=False,
                 position=carla.Location(-75, 115, 0.3),
                 orientation=carla.Rotation(0, -89, 0),
                 random_spawn=False,
                 vehicle_type='vehicle.Seat.Leon',
                 town_name='Town03',
                 control='auto',  # auto rovis
                 ip_carla='127.0.0.1',
                 port_carla=2000,

                 ip_rovis='127.0.0.1',
                 port_rovis_actuator=None,
                 port_rovis_camera_front=None,
                 port_rovis_camera_back=None,
                 port_rovis_semseg_camera=None,
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
        if port_rovis_camera_front is not None:
            self.front_cam_sensor = MonoCameraSensor('{} - FrontCamera'.format(self.name),
                                                     self.car,
                                                     self.view_width,
                                                     self.view_height,
                                                     self.view_fov)
            self.front_cam_sensor.setup_camera(get_transform('front'))
            # self.front_cam_sensor.init_view()

            self.front_cam_server = MonoCameraServer(ip=ip_rovis,
                                                     port=port_rovis_camera_front,
                                                     dt=0.0,
                                                     cam_sensor=self.front_cam_sensor)
            self.front_cam_server.init_server()

        if port_rovis_camera_back is not None:
            self.back_cam_sensor = MonoCameraSensor('{} - BackCamera'.format(self.name),
                                                    self.car,
                                                    self.view_width,
                                                    self.view_height,
                                                    self.view_fov)
            self.back_cam_sensor.setup_camera(get_transform('back'))
            # self.back_cam_sensor.init_view()

            self.back_cam_server = MonoCameraServer(ip=ip_rovis,
                                                    port=port_rovis_camera_back,
                                                    dt=0.0,
                                                    cam_sensor=self.back_cam_sensor)
            self.back_cam_server.init_server()

        # Attach SemSeg Camera
        if port_rovis_semseg_camera is not None:
            self.semseg_cam_sensor = SemSegCameraSensor('{} - SemSegFront'.format(self.name),
                                                        self.car,
                                                        self.view_width,
                                                        self.view_height,
                                                        self.view_fov)
            self.semseg_cam_sensor.setup_camera(get_transform('front'))
            # self.semseg_cam_sensor.init_view()

            self.semseg_cam_server = SemSegCameraServer(ip=ip_rovis,
                                                        port=port_rovis_semseg_camera,
                                                        dt=0.0,
                                                        cam_sensor=self.semseg_cam_sensor)
            self.semseg_cam_server.init_server()

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

        # Attach IMU sensor
        if port_rovis_imu is not None:
            self.imu_sensor = IMUSensor(name='{} - IMU'.format(self.name),
                                        parent_actor=self.car)
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
                                            parent_actor=self.car)
                self.imu_sensor.setup_imu()

            # State measurement server
            self.state_meas_server = StateMeasurementServer(ip=ip_rovis,
                                                            port=port_rovis_state_measurement,
                                                            dt=0.00,
                                                            car=self.car,
                                                            imu=self.imu_sensor,
                                                            client_name=self.name)
            self.state_meas_server.init_server()

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
            if hasattr(self, 'semseg_cam_sensor'):
                self.semseg_cam_sensor.capture = True

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

        control.throttle = self.vehicle_actuator.acceleration
        control.steer = -self.vehicle_actuator.steering

        if self.vehicle_actuator.acceleration < 0:
            control.reverse = True
        else:
            control.reverse = False

        #print("acc:", control.throttle)

        # elapsed = time.time() - self.start_time
        # if elapsed > 5:
        #     self.start_time = time.time()
        #     if self.move is False:
        #         control.throttle = 0.5
        #         self.move = True
        #     else:
        #         control.throttle = 0.0
        #         self.move = False
        #
        # print("elapsed:", elapsed)
        # print("throttle:", control.throttle)
        # print("")

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
        physics_control.moi = 0.1  # Default 1.0
        physics_control.damping_rate_full_throttle = 0.0
        physics_control.use_gear_autobox = True
        physics_control.gear_switch_time = 0.0
        physics_control.clutch_strength = 1
        physics_control.mass = 10000  # Default 10000
        physics_control.drag_coefficient = 0.
        physics_control.steering_curve = [carla.Vector2D(x=0, y=1), carla.Vector2D(x=100, y=1),
                                          carla.Vector2D(x=300, y=1)]
        physics_control.wheels = wheels

        # Update params
        self.car.apply_physics_control(physics_control)


if __name__ == '__main__':
    print('NOT runnable. Check \'Run_CarlaClients.py\'')
