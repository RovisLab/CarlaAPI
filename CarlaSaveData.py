import os
import sys
import glob
import time
import threading
import random
import numpy as np
import psutil

from clients.VehicleActuatorClient import VehicleActuatorClient
from servers.CameraServer import CameraServer, CameraSensor
from servers.SemSegCameraServer import SemSegCameraServer, SemSegCameraSensor
from servers.VehicleStateEstimationServer import StateMeasurementServer
from servers.ImuServer import IMUServer, IMUSensor
from servers.DepthServer import DepthSensor, DepthServer
from servers.LidarServer import LidarSensor, LidarServer
from servers.RadarServer import RadarSensor, RadarServer
from sensors import *

from Run_CarlaClients import CARLA_EXE_PATH, check_carla_simulator_running_state, start_carla

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
    For this to work, add to the <root>/depend folder, the following from RovisDojo:
 - RovisDatabaseFormat.py
 - types_ROVIS_TYPES.py
 - global_config.py
 - config.json
'''
from depend.RovisDatabaseFormat import RovisDataBase


def main():
    # start_carla(CARLA_EXE_PATH)

    client = CarlaSaveData(db_path=r'C:\data\Carla\tmp')
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
        self.view_width = 800
        self.view_height = 600
        self.view_fov = 90
        self.client = carla.Client('127.0.0.1', 2000)
        self.client.set_timeout(2.0)
        self.world = self.client.get_world()
        self.random_spawn = True
        self.using_carla_ai = False
        self.signal_save = False
        self.db_path = db_path
        self.data_counter_lim = 1000  # Set to -1 to save endlessly
        self.data_counter = 0
        self.db = None

        # Check map
        town_name = 'Town04'  # Town03 Town05
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

        settings = self.world.get_settings()
        settings.synchronous_mode = True  # Enables synchronous mode
        target_fps = 30
        target_freq = 1.0 / target_fps
        settings.fixed_delta_seconds = target_freq
        self.world.apply_settings(settings)

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
        self.m_cams['front'] = CameraSensor('{} - CameraFront'.format(self.name), self.car,
                                            self.view_width, self.view_height, self.view_fov)
        self.m_cams['front'].setup_camera_rgb(get_transform('front'))
        self.m_cams['front'].init_view()
        
        self.m_cams['back'] = CameraSensor('{} - CameraBack'.format(self.name), self.car,
                                           self.view_width, self.view_height, self.view_fov)
        self.m_cams['back'].setup_camera_rgb(get_transform('back'))
        # self.m_cams['cam_back'].init_view()
        
        self.m_cams['left'] = CameraSensor('{} - CameraLeft'.format(self.name), self.car,
                                           self.view_width, self.view_height, self.view_fov)
        self.m_cams['left'].setup_camera_rgb(get_transform('front_left'))
        # self.m_cams['cam_left'].init_view()
        
        self.m_cams['right'] = CameraSensor('{} - CameraRight'.format(self.name), self.car,
                                            self.view_width, self.view_height, self.view_fov)
        self.m_cams['right'].setup_camera_rgb(get_transform('front_right'))
        # self.m_cams['cam_right'].init_view()
        
        self.m_cams['back_left'] = CameraSensor('{} - CameraBackLeft'.format(self.name), self.car,
                                                self.view_width, self.view_height, self.view_fov)
        self.m_cams['back_left'].setup_camera_rgb(get_transform('back_left'))
        # self.m_cams['back_left'].init_view()
        
        self.m_cams['back_right'] = CameraSensor('{} - CameraBackRight'.format(self.name), self.car,
                                                 self.view_width, self.view_height, self.view_fov)
        self.m_cams['back_right'].setup_camera_rgb(get_transform('back_right'))
        # self.m_cams['back_right'].init_view()

        self.semseg = {item: None for item in self.cam_order}
        # Attach SemSeg Cameras ########################################################################################
        self.semseg['front'] = SemSegCameraSensor('{} - SemSegFront'.format(self.name), self.car,
                                                  self.view_width, self.view_height, self.view_fov)
        self.semseg['front'].setup_camera_rgb(get_transform('front'))
        # self.semseg['front'].init_view()

        self.semseg['back'] = SemSegCameraSensor('{} - SemSegBack'.format(self.name), self.car,
                                                 self.view_width, self.view_height, self.view_fov)
        self.semseg['back'].setup_camera_rgb(get_transform('back'))
        # self.semseg['back'].init_view()

        self.semseg['left'] = SemSegCameraSensor('{} - SemSegLeft'.format(self.name), self.car,
                                                 self.view_width, self.view_height, self.view_fov)
        self.semseg['left'].setup_camera_rgb(get_transform('front_left'))
        # self.semseg['left'].init_view()

        self.semseg['right'] = SemSegCameraSensor('{} - SemSegRight'.format(self.name), self.car,
                                                  self.view_width, self.view_height, self.view_fov)
        self.semseg['right'].setup_camera_rgb(get_transform('front_right'))
        # self.semseg['right'].init_view()

        self.semseg['back_left'] = SemSegCameraSensor('{} - SemSegBackLeft'.format(self.name), self.car,
                                                      self.view_width, self.view_height, self.view_fov)
        self.semseg['back_left'].setup_camera_rgb(get_transform('back_left'))
        # self.semseg['back_left'].init_view()

        self.semseg['back_right'] = SemSegCameraSensor('{} - SemSegBackRight'.format(self.name), self.car,
                                                       self.view_width, self.view_height, self.view_fov)
        self.semseg['back_right'].setup_camera_rgb(get_transform('back_right'))
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
        cv2.destroyAllWindows()
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

            # Do not stop at traffic lights
            if self.car.is_at_traffic_light():
                traffic_light = self.car.get_traffic_light()
                if traffic_light.get_state() == carla.TrafficLightState.Red:
                    # world.hud.notification("Traffic light changed! Good to go!")
                    traffic_light.set_state(carla.TrafficLightState.Green)

            # Capture flag for cameras. Without this, the image will not update
            for key in self.cam_order:
                self.semseg[key].capture = True
                self.m_cams[key].capture = True

            # Save data
            if self.signal_save:
                time.sleep(0.1)
                self.save_data(ts_start, ts_stop)
                ts_start = ts_stop
                ts_stop += 5  # ts_stop += self.sampling_time

            self.carla_ai_control(self.car)
            self.world.tick()

    def timer_callback(self):
        while not self.is_terminated:
            if not self.signal_save:
                time.sleep(self.sampling_time)
                self.signal_save = True

    def save_data(self, ts_start, ts_stop):
        # Get data
        rgb_images = []
        semseg_images = []

        for key in self.cam_order:
            rgb_images.append(self.m_cams[key].process_img_rgb())
            semseg_images.append(self.semseg[key].process_img_rgb())

        if any(elem is None for elem in rgb_images) or any(elem is None for elem in semseg_images):
            self.signal_save = False
            return

        # Simulate enet network transfer
        # for i in range(6):
        #     ret, rgb_images[i] = cv2.imencode(".jpg", rgb_images[i])
        #     rgb_images[i] = cv2.imdecode(rgb_images[i], -1)

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
        self.data_counter += 1
        if self.data_counter_lim != -1:
            if self.data_counter % 10 == 0:
                print(" - Saved {} out of {}.".format(self.data_counter, self.data_counter_lim))
            if self.data_counter == self.data_counter_lim:
                self.terminate()
        else:
            if self.data_counter % 10 == 0:
                print(" - Saved {}.".format(self.data_counter))

        self.signal_save = False

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
