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


class GnssSensor(BaseSensor):
    def __init__(self, name, parent_actor, client_args):
        super().__init__(name, parent_actor, client_args)
        self.has_capture = True

    # ==================== General sensor methods ====================
    def setup(self):
        world = self._parent.get_world()
        sensor_bp = world.get_blueprint_library().find('sensor.other.gnss')

        self.sensor = world.spawn_actor(
            sensor_bp, self.sensor_transform, attach_to=self._parent
        )
        weak_self = weakref.ref(self)
        self.sensor.listen(
            lambda data: weak_self().sensor_callback(weak_self, data)
        )

    def get_data(self, optional_data_type=''):
        if self.data is not None:
            # gnss_data = [self.data.latitude, self.data.longitude, self.data.altitude]
            gnss_data = [self.data.latitude, self.data.longitude]
            return gnss_data
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
            if self.data is not None:
                gnss_data = self.get_data()
                print(' - {}/{} data: {}, {}'.format(self.parent_name, self.name, gnss_data[0], gnss_data[1]))
                time.sleep(self.viewer_delay / 1000)

    # ==================== Server methods ====================
    def encode_data_transfer(self):
        data = self.get_data()
        # data = [latitude, longitude]

        if data is not None:
            string_data = "{\"GnssData\":\""
            for val in data:
                string_data += str('%.6f' % val) + ";"
            string_data += "\"&}"

            return bytes(str(string_data), 'utf8')
        return None

    # ==================== Pack data for saving ====================
    def pack_data(self, ts_stop, frame_id):
        # Not yet implemented in RovisDatabaseFormat
        return {}


if __name__ == '__main__':
    print('NOT runnable. Check \'Run_Carla.py\'')
