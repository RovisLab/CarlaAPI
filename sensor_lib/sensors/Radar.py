import os
import sys
import glob
import time
import numpy as np
import math
import cv2
import weakref

import config_utils as conf
from sensor_lib.BaseSensor import BaseSensor

try:
    sys.path.append(glob.glob('%s/PythonAPI/carla/dist/carla-*%d.%d-%s.egg' % (
        conf.CARLA_PATH,
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass
import carla


class RadarSensor(BaseSensor):
    def __init__(self, name, parent_actor, client_args):
        super().__init__(name, parent_actor, client_args)
        self.has_capture = True
        self.debug = self._parent.get_world().debug  # for drawing radar
        self.h_fov = 35  # Horizontal FOV
        self.v_fov = 30  # Vertical FOV
        self.range = 100
        self.pps = 1500  # Points per second
        self.vel_range = 7.5  # Velocity range [m/s]

    # ==================== General sensor methods ====================
    def parse_args(self):
        super().parse_args()

        for key in self.args.keys():
            if key == 'h_fov':
                self.h_fov = self.args[key]
            elif key == 'v_fov':
                self.v_fov = self.args[key]
            elif key == 'range':
                self.range = self.args[key]
            elif key == 'pps':
                self.pps = self.args[key]
            elif key == 'vel_range':
                self.vel_range = self.args[key]

    def setup(self):
        world = self._parent.get_world()
        radar_bp = world.get_blueprint_library().find('sensor.other.radar')
        radar_bp.set_attribute('horizontal_fov', str(self.h_fov))
        radar_bp.set_attribute('vertical_fov', str(self.v_fov))
        radar_bp.set_attribute('range', str(self.range))
        radar_bp.set_attribute('points_per_second', str(self.pps))

        self.sensor = world.spawn_actor(
            radar_bp, self.sensor_transform, attach_to=self._parent,
            attachment_type=carla.AttachmentType.Rigid
        )
        weak_self = weakref.ref(self)
        self.sensor.listen(
            lambda data: weak_self().sensor_callback(weak_self, data)
        )

    def get_data(self, optional_data_type=''):
        if self.data is not None:
            points = np.frombuffer(self.data.raw_data, dtype=np.dtype('f4'))
            points = np.reshape(points, (len(self.data), 4))
            return points  # [[vel, altitude, azimuth, depth], ...]
        return None

    @staticmethod
    def sensor_callback(weak_ref, data, **kwargs):
        self = weak_ref()
        if self.capture:
            self.data = data
            self.capture = False

    # ==================== Viewer methods ====================
    def do_view(self):
        while not self.stop_view and self.sensor is not None:
            time.sleep(self.viewer_delay / 1000)
            self.draw_radar()

        time.sleep(0.001)

    # ==================== Server methods ====================
    def encode_data_transfer(self):
        data = self.get_data()

        if data is not None:
            string_data = "{\"RadarData\":\""
            string_data += ';'.join(str('%.3f' % val) for val in data.flatten())
            string_data += "\"}"

            return bytes(str(string_data), 'utf8')
        return None

    # ==================== Pack data for saving ====================
    def pack_data(self, ts_stop, frame_id):
        data = self.get_data()

        packed_data = {
            'name': '{}'.format(ts_stop),
            'radar_file': data
        }
        return {}  # TODO test
        # return packed_data

    # ==================== Custom methods ====================
    def draw_radar(self):
        if self.data is None or self.debug is None:
            return

        radar_data = self.data
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

            norm_velocity = detect.velocity / self.vel_range  # range [-1, 1]
            r = int(clamp(0.0, 1.0, 1.0 - norm_velocity) * 255.0)
            g = int(clamp(0.0, 1.0, 1.0 - abs(norm_velocity)) * 255.0)
            b = int(abs(clamp(- 1.0, 0.0, - 1.0 - norm_velocity)) * 255.0)
            self.debug.draw_point(
                radar_data.transform.location + fw_vec,
                size=0.075,
                life_time=0.06,
                persistent_lines=False,
                color=carla.Color(r, g, b))


if __name__ == '__main__':
    print('NOT runnable. Check \'Run_Carla.py\'')
