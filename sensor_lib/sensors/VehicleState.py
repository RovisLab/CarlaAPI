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
        heading = 2 * math.pi - actor_yaw

        if heading < -math.pi:
            heading = 2 * math.pi + heading

        # Convert position to rovis coord sys
        R = np.array([[1, 0, 0], [0, -1, 0], [0, 0, 1]])
        position = R @ np.array([tr.location.x, tr.location.y, tr.location.z])
        velocity = R @ np.array([vel.x, vel.y, vel.z])
        speed = math.sqrt(velocity[0] ** 2 + velocity[1] ** 2 + velocity[2] ** 2)

        # Get steering TODO
        steering = 0

        # [x, y, speed, heading, steering]
        return [position[0], position[1], speed, heading, steering]

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
        data = self.get_data()  # [x, y, speed, heading, steering]

        if data is not None:
            veh_state_data = "{\"CarData\":\""
            veh_state_data += str('%0.4f' % data[0]) + ";"
            veh_state_data += str('%0.4f' % data[1]) + ";"
            if data[2] != 0:
                veh_state_data += str('%.6f' % data[2]) + ";"
            else:
                veh_state_data += "0;"
            veh_state_data += str('%.6f' % data[3]) + ";"
            veh_state_data += str('%.6f' % data[4]) + ";"
            veh_state_data += "/" + "&2&1;&"
            veh_state_data += "\"}"

            return bytes(str(veh_state_data), 'utf8')
        return None

    # ==================== Pack data for saving ====================
    def pack_data(self, ts_stop, frame_id):
        data = self.get_data()

        packed_data = {
            's_var_0': str('%0.4f' % data[0]),
            's_var_1': str('%0.4f' % data[1]),
            's_var_2': str('%.6f' % data[2]) if data[2] != 0 else "0;",
            's_var_3': str('%.6f' % data[3])
        }
        return packed_data


if __name__ == '__main__':
    print('NOT runnable. Check \'Run_Carla.py\'')
