import os
import sys
import glob
import time
import threading
import random
import numpy as np
import psutil

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
    sys.path.append(glob.glob('dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass
import carla

'''
    For this to work, add to the <root>/depend folder, the following from RovisDojo:L
 - RovisDatabaseFormat.py
 - types_ROVIS_TYPES.py
 - global_config.py
 - config.json
'''
from depend.RovisDatabaseFormat import RovisDataBase


def main():
    client = CarlaSaveData(db_path=r'C:\data\Carla\SemSegBig2')
    try:
        threading.Thread(target=client.game_loop).start()
        time.sleep(2)
        print(' - Starting to save data.')
        threading.Thread(target=client.timer_callback).start()
    except RuntimeError as e:
        print("Runtime error encountered.")
        print(e)
    except MemoryError as e:
        print("Memory error encountered.")
        print(e)


class CarlaSaveData(object):
    def __init__(self, db_path):
        # Member variables
        self.name = 'Client'
        self.sampling_time = 5.0  # dt
        self.target_fps = 30
        self.target_freq = 1.0 / self.target_fps
        self.view_width = 640
        self.view_height = 480
        self.view_fov = 90
        self.client = carla.Client('127.0.0.1', 2000)
        self.client.set_timeout(2.0)
        self.world = self.client.get_world()
        self.random_spawn = True
        self.using_carla_ai = False
        self.bool_save = False
        self.db_path = db_path
        self.count_saved_data = 1000  # Set to -1 to save endlessly
        self.db = None

        # Check map
        town_name = 'Town03'  # Town03 Town05
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
        if self.random_spawn:
            spawn_points = self.map.get_spawn_points()
            spawn_point = random.choice(spawn_points) if spawn_points \
                else carla.Transform()
        else:
            position = carla.Location(-77.3, 54.4, 1),
            orientation = carla.Rotation(0, -90, 0),
            spawn_point = carla.Transform(position, orientation)

        # Instantiate car
        bp = self.world.get_blueprint_library().filter('vehicle.Seat.Leon')[0]
        self.car = self.world.spawn_actor(bp, spawn_point)

        if self.car is None:
            print('{} - Could not spawn the car. Exiting..'.format(self.name))
            sys.exit(1)

        self.car.apply_control(carla.VehicleControl(manual_gear_shift=True, gear=1))
        self.car.apply_control(carla.VehicleControl(manual_gear_shift=False))

        self.cam_order = ['front', 'back', 'left', 'right', 'back_left', 'back_right']

        self.m_cams = {item: None for item in self.cam_order}
        # Attach MonoCameras ###########################################################################################
        self.m_cams['front'] = MonoCameraSensor('{} - CameraFront'.format(self.name), self.car,
                                                self.view_width, self.view_height, self.view_fov)
        self.m_cams['front'].setup_camera(get_transform('front'))
        self.m_cams['front'].init_view()
        
        self.m_cams['back'] = MonoCameraSensor('{} - CameraBack'.format(self.name), self.car,
                                               self.view_width, self.view_height, self.view_fov)
        self.m_cams['back'].setup_camera(get_transform('back'))
        # self.m_cams['cam_back'].init_view()
        
        self.m_cams['left'] = MonoCameraSensor('{} - CameraLeft'.format(self.name), self.car,
                                               self.view_width, self.view_height, self.view_fov)
        self.m_cams['left'].setup_camera(get_transform('front_left'))
        # self.m_cams['cam_left'].init_view()
        
        self.m_cams['right'] = MonoCameraSensor('{} - CameraRight'.format(self.name), self.car,
                                                self.view_width, self.view_height, self.view_fov)
        self.m_cams['right'].setup_camera(get_transform('front_right'))
        # self.m_cams['cam_right'].init_view()
        
        self.m_cams['back_left'] = MonoCameraSensor('{} - CameraBackLeft'.format(self.name), self.car,
                                                    self.view_width, self.view_height, self.view_fov)
        self.m_cams['back_left'].setup_camera(get_transform('back_left'))
        # self.m_cams['back_left'].init_view()
        
        self.m_cams['back_right'] = MonoCameraSensor('{} - CameraBackRight'.format(self.name), self.car,
                                                     self.view_width, self.view_height, self.view_fov)
        self.m_cams['back_right'].setup_camera(get_transform('back_right'))
        # self.m_cams['back_right'].init_view()

        self.semseg = {item: None for item in self.cam_order}
        # Attach SemSeg Cameras ########################################################################################
        self.semseg['front'] = SemSegCameraSensor('{} - SemSegFront'.format(self.name), self.car,
                                                  self.view_width, self.view_height, self.view_fov)
        self.semseg['front'].setup_camera(get_transform('front'))
        # self.semseg['front'].init_view()

        self.semseg['back'] = SemSegCameraSensor('{} - SemSegBack'.format(self.name), self.car,
                                                 self.view_width, self.view_height, self.view_fov)
        self.semseg['back'].setup_camera(get_transform('back'))
        # self.semseg['back'].init_view()

        self.semseg['left'] = SemSegCameraSensor('{} - SemSegLeft'.format(self.name), self.car,
                                                 self.view_width, self.view_height, self.view_fov)
        self.semseg['left'].setup_camera(get_transform('front_left'))
        # self.semseg['left'].init_view()

        self.semseg['right'] = SemSegCameraSensor('{} - SemSegRight'.format(self.name), self.car,
                                                  self.view_width, self.view_height, self.view_fov)
        self.semseg['right'].setup_camera(get_transform('front_right'))
        # self.semseg['right'].init_view()

        self.semseg['back_left'] = SemSegCameraSensor('{} - SemSegBackLeft'.format(self.name), self.car,
                                                      self.view_width, self.view_height, self.view_fov)
        self.semseg['back_left'].setup_camera(get_transform('back_left'))
        # self.semseg['back_left'].init_view()

        self.semseg['back_right'] = SemSegCameraSensor('{} - SemSegBackRight'.format(self.name), self.car,
                                                       self.view_width, self.view_height, self.view_fov)
        self.semseg['back_right'].setup_camera(get_transform('back_right'))
        # self.semseg['back_right'].init_view()

        self.init_db()

        print(' # {} initialised successfully.'.format(self.name))

    def init_db(self):
        # Create empty database
        self.db = RovisDataBase(db_path=self.db_path, core_id=1)
        self.db.add_stream(filter_type='image', filter_id=1, filter_name='CameraFront')
        self.db.add_stream(filter_type='image', filter_id=2, filter_name='CameraBack')
        self.db.add_stream(filter_type='image', filter_id=3, filter_name='CameraLeft')
        self.db.add_stream(filter_type='image', filter_id=4, filter_name='CameraRight')
        self.db.add_stream(filter_type='image', filter_id=5, filter_name='CameraBackLeft')
        self.db.add_stream(filter_type='image', filter_id=6, filter_name='CameraBackRight')
        self.db.add_stream(filter_type='sem_seg', filter_id=7, filter_name='SemSegFront', input_sources=[1])
        self.db.add_stream(filter_type='sem_seg', filter_id=8, filter_name='SemSegBack', input_sources=[2])
        self.db.add_stream(filter_type='sem_seg', filter_id=9, filter_name='SemSegLeft', input_sources=[3])
        self.db.add_stream(filter_type='sem_seg', filter_id=10, filter_name='SemSegRight', input_sources=[4])
        self.db.add_stream(filter_type='sem_seg', filter_id=11, filter_name='SemSegBackLeft', input_sources=[5])
        self.db.add_stream(filter_type='sem_seg', filter_id=12, filter_name='SemSegBackRight', input_sources=[6])
        # db.show_packing_info()
        self.db.create_db()

    def terminate(self):
        self.car.destroy()
        for cam in self.m_cams.values():
            cam.destroy()
        for semseg_cam in self.semseg.values():
            semseg_cam.destroy()
        self.is_terminated = True

        print(' # {} successfully terminated.'.format(self.name))
        return

    @staticmethod
    def check_carla_simulator_running_state():
        for p in psutil.process_iter():
            if "carla" in p.name().lower():
                return True
        return False

    def game_loop(self):
        # Timestamp generation
        ts_start = int(time.time())
        ts_stop = ts_start

        while not self.is_terminated:
            self.world.tick()

            # Capture flag for cameras. Without this, the image will not update
            for cam in self.m_cams.values():
                cam.capture = True
            for semseg_cam in self.semseg.values():
                semseg_cam.capture = True

            # Save data
            if self.bool_save:
                self.save_data(ts_start, ts_stop)
                ts_start = ts_stop
                ts_stop += 5  # ts_stop += self.sampling_time

            self.carla_ai_control(self.car)
            self.world.tick()

    def timer_callback(self):
        while not self.is_terminated:
            time.sleep(self.sampling_time)
            self.bool_save = True

    def save_data(self, ts_start, ts_stop):
        # Get data
        rgb_images = []
        semseg_images = []

        for key in self.cam_order:
            rgb_images.append(self.m_cams[key].process_image())
            semseg_images.append(self.semseg[key].process_image())

        if any(elem is None for elem in rgb_images) or any(elem is None for elem in semseg_images):
            self.bool_save = False
            return

        # Simulate enet network transfer
        for i in range(6):
            ret, rgb_images[i] = cv2.imencode(".jpg", rgb_images[i])
            rgb_images[i] = cv2.imdecode(rgb_images[i], -1)

        data = {
            'datastream_1': {  # image
                'name': '{}.png'.format(ts_stop), 'image': rgb_images[0]},
            'datastream_2': {  # image
                'name': '{}.png'.format(ts_stop), 'image': rgb_images[1]},
            'datastream_3': {  # image
                'name': '{}.png'.format(ts_stop), 'image': rgb_images[2]},
            'datastream_4': {  # image
                'name': '{}.png'.format(ts_stop), 'image': rgb_images[3]},
            'datastream_5': {  # image
                'name': '{}.png'.format(ts_stop), 'image': rgb_images[4]},
            'datastream_6': {  # image
                'name': '{}.png'.format(ts_stop), 'image': rgb_images[5]},

            'datastream_7': {  # sem_seg
                'name': '{}.png'.format(ts_stop), 'semantic': semseg_images[0]},
            'datastream_8': {  # sem_seg
                'name': '{}.png'.format(ts_stop), 'semantic': semseg_images[1]},
            'datastream_9': {  # sem_seg
                'name': '{}.png'.format(ts_stop), 'semantic': semseg_images[2]},
            'datastream_10': {  # sem_seg
                'name': '{}.png'.format(ts_stop), 'semantic': semseg_images[3]},
            'datastream_11': {  # sem_seg
                'name': '{}.png'.format(ts_stop), 'semantic': semseg_images[4]},
            'datastream_12': {  # sem_seg
                'name': '{}.png'.format(ts_stop), 'semantic': semseg_images[5]}
        }

        # Add data to database
        self.db.add_data(ts_start=ts_start, ts_stop=ts_stop, data=data)

        # Count
        if self.count_saved_data != -1:
            self.count_saved_data -= 1
            if not self.count_saved_data:
                self.terminate()

        self.bool_save = False

    def set_synchronous_mode(self, synchronous_mode):
        settings = self.world.get_settings()
        settings.synchronous_mode = synchronous_mode
        if synchronous_mode:
            settings.fixed_delta_seconds = self.target_freq
        self.world.apply_settings(settings)

    def carla_ai_control(self, car):
        if self.using_carla_ai is False:
            self.using_carla_ai = True
            car.set_autopilot()
            self.using_carla_ai = True


if __name__ == '__main__':
    main()
