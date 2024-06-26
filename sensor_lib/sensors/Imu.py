import os
import sys
import glob
import time
import numpy as np
import math
import cv2
import weakref
from scipy.spatial.transform import Rotation

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


class ImuSensor(BaseSensor):
    def __init__(self, name, parent_actor, client_args):
        super().__init__(name, parent_actor, client_args)
        self.has_capture = True

    # ==================== General sensor methods ====================
    def setup(self):
        world = self._parent.get_world()
        imu_bp = world.get_blueprint_library().find('sensor.other.imu')

        self.sensor = world.spawn_actor(
            imu_bp, self.sensor_transform, attach_to=self._parent
        )
        weak_self = weakref.ref(self)
        self.sensor.listen(
            lambda data: weak_self().sensor_callback(weak_self, data)
        )

    def get_data(self, optional_data_type=''):
        if self.data is not None:
            limits = (-99.9, 99.9)

            # Convert [0, 360] heading to [-180, 180] degrees
            heading = self.data.compass  # rad
            # if heading > math.pi:
            #    heading = heading - 2. * math.pi
            # heading = -heading

            rot = Rotation.from_euler('xyz', [0.0, 0.0, heading], degrees=False)
            quat = rot.as_quat()

            imu_data = [
                max(limits[0], min(limits[1], self.data.accelerometer.x)),  # Accelerometer X
                max(limits[0], min(limits[1], self.data.accelerometer.y)),  # Accelerometer Y
                max(limits[0], min(limits[1], self.data.accelerometer.z)),  # Accelerometer Z
                max(limits[0], min(limits[1], math.degrees(self.data.gyroscope.x))),  # Gyroscope X
                max(limits[0], min(limits[1], math.degrees(self.data.gyroscope.y))),  # Gyroscope Y
                max(limits[0], min(limits[1], math.degrees(self.data.gyroscope.z))),  # Gyroscope Z
                math.cos(self.data.compass),  # Magnetometer X
                math.sin(self.data.compass),  # Magnetometer Y
                0.0,  # Magnetometer Z
                quat[0],  # x
                quat[1],  # y
                quat[2],  # z
                quat[3]   # w
            ]
            return imu_data
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
            print(' - {}/{} data: {}'.format(self.parent_name, self.name, self.get_real_imu()))
            time.sleep(self.viewer_delay / 1000)

    # ==================== Server methods ====================
    def encode_data_transfer(self):
        data = self.get_data()
        # data = [accX, accY, accZ, gyroX, gyroY, gyroZ, magnetX, magnetY, magnetZ, roll, pitch, yaw]

        if data is not None:
            string_data = "{\"ImuData\":\""
            for val in data:
                string_data += str('%.6f' % val) + ";"
            string_data += "\"&}"

            return bytes(str(string_data), 'utf8')
        return None

    # ==================== Pack data for saving ====================
    def pack_data(self, ts_stop, frame_id):
        data = self.get_data()

        packed_data = {
            'acc_x': data[0], 'acc_y': data[1], 'acc_z': data[2],
            'gyro_x': data[3], 'gyro_y': data[4], 'gyro_z': data[5],
            'pitch': data[10],  # y
            'yaw': data[11],   # z
            'roll': data[9]    # x
        }
        return packed_data

    # ==================== Custom methods ====================
    def get_real_imu(self):
        cos_a = math.cos(math.radians(180))
        sin_a = math.sin(math.radians(180))
        R = np.array([[cos_a, -sin_a, 0], [sin_a, cos_a, 0], [0, 0, 1]])

        position = np.array([self._parent.get_location().x,
                             self._parent.get_location().y,
                             self._parent.get_location().z])
        velocity = self._parent.get_velocity()

        rot = R @ position
        speed = math.sqrt(velocity.x ** 2 + velocity.y ** 2 + velocity.z ** 2)

        _string = ""
        _string += "{}:{};".format('pose', self._parent.get_location())
        _string += "{}:{};".format('heading', math.degrees(self.data.compass))
        _string += "{}:{};".format('velocity', velocity)
        _string += "{}:{:.3f};".format('speed', speed)

        return _string


if __name__ == '__main__':
    print('NOT runnable. Check \'Run_Carla.py\'')
