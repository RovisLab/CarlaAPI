import os
import sys
import glob
import time
import math
import numpy as np
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


class VehicleStateEstimation(BaseSensor):
    def __init__(self, name, parent_actor, client_args):
        super().__init__(name, parent_actor, client_args)

    # ==================== General sensor methods ====================
    def setup(self):
        pass

    def get_data(self, optional_data_type=''):
        if not self._parent.is_alive:
            return ""
        tr = self._parent.get_transform()  # Location + Rotation
        vel = self._parent.get_velocity()  # Velocity

        # Get Compass => Get actor's yaw angle
        # yaw = 0 => actor is facing East
        # Rovis direction = West
        actor_yaw = tr.rotation.yaw  # degrees
        actor_yaw = actor_yaw * math.pi / 180  # rad
        heading = math.pi / 2 - actor_yaw

        if heading < -math.pi:
            heading = 2 * math.pi + heading

        # Convert position to rovis coord sys
        R = np.array([[1, 0, 0], [0, -1, 0], [0, 0, 1]])
        position = R @ np.array([tr.location.x, tr.location.y, tr.location.z])
        velocity = R @ np.array([vel.x, vel.y, vel.z])
        speed = math.sqrt(velocity[0] ** 2 + velocity[1] ** 2 + velocity[2] ** 2)

        _string = ""
        _string += str('%0.4f' % position[0]) + ";"  # Get x
        _string += str('%0.4f' % position[1]) + ";"  # Get y
        if speed != 0:
            _string += str('%.6f' % speed) + ";"
        else:
            _string += "0;"

        _string += str('%.6f' % heading) + ";"  # Get yaw angle
        _string += str('%.6f' % 0) + ";"  # TODO: Steering angle

        return _string + "/" + "&2&1;&"

    @staticmethod
    def sensor_callback(weak_ref, data):
        pass

    # ==================== Viewer methods ====================
    def do_view(self):
        while not self.stop_view:
            print(' - {}/{} data: {}'.format(self.parent_name, self.name, self.get_data()))
            time.sleep(self.viewer_delay / 1000)

    # ==================== Server methods ====================
    def encode_data_transfer(self):
        data = self.get_data()

        if data is not None:
            veh_state_data = "{\"CarData\":\""
            veh_state_data += "{}".format(data)
            veh_state_data += "\"}"

            return bytes(str(veh_state_data), 'utf8')
        return None

    # ==================== Pack data for saving ====================
    def pack_data(self, ts_stop, frame_id):
        data = self.get_data()

        # TODO get this vars from data
        packed_data = {
            's_var_0': float,
            's_var_1': float,
            's_var_2': float,
            's_var_3': float
        }
        return {}
        # return packed_data


if __name__ == '__main__':
    print('NOT runnable. Check \'Run_Carla.py\'')
