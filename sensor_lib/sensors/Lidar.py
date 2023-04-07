import os
import sys
import glob
import time
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


class LidarSensor(BaseSensor):
    def __init__(self, name, parent_actor, client_args):
        super().__init__(name, parent_actor, client_args)
        self.has_capture = True
        self.range = 75
        self.channels = 100  # Number of horizontal rays
        self.pps = 10**6  # points per second (divided between the rays)
        self.rot_freq = 60  # Rotation frequency
        self.lower_fov = -20  # Lowest angle the LiDAR will record
        self.upper_fov = 10  # Highest angle the LiDAR will record

    # ==================== General sensor methods ====================
    def parse_args(self):
        super().parse_args()

        for key in self.args.keys():
            if key == 'range':
                self.range = self.args[key]
            elif key == 'channels':
                self.channels = self.args[key]
            elif key == 'pps':
                self.pps = self.args[key]
            elif key == 'rot_freq':
                self.rot_freq = self.args[key]

    def setup(self):
        world = self._parent.get_world()
        lidar_bp = world.get_blueprint_library().find('sensor.lidar.ray_cast')
        lidar_bp.set_attribute('range', str(self.range))
        lidar_bp.set_attribute('channels', str(self.channels))
        lidar_bp.set_attribute('points_per_second', str(self.pps))
        lidar_bp.set_attribute('rotation_frequency', str(self.rot_freq))
        lidar_bp.set_attribute('lower_fov', str(self.lower_fov))
        lidar_bp.set_attribute('upper_fov', str(self.upper_fov))

        self.sensor = world.spawn_actor(
            lidar_bp, self.sensor_transform, attach_to=self._parent,
            attachment_type=carla.AttachmentType.Rigid
        )
        weak_self = weakref.ref(self)
        self.sensor.listen(
            lambda data: weak_self().sensor_callback(weak_self, data)
        )

    def get_data(self, optional_data_type=''):
        if self.data is not None:
            points = np.frombuffer(self.data.raw_data, dtype=np.dtype('f4'))
            points = np.reshape(points, (int(points.shape[0] / 4), 4))
            return points
        return None

    @staticmethod
    def sensor_callback(weak_ref, data):
        self = weak_ref()
        if self.capture:
            self.data = data
            self.capture = False

    # ==================== Viewer methods ====================
    def do_view(self):
        window_name = '{} - {}'.format(self.parent_name, self.name)
        while not self.stop_view and self.sensor is not None:
            point_cloud = self.get_data()

            key = None
            if point_cloud is not None:
                key = self.view_lidar(point_cloud, window_name)

            if key is not None and key != -1:
                if key == 27:  # ESC - close everything
                    self.terminate_signal.emit()
                    self.stop_view = True
                elif key == ord('q'):  # Close just the viewer
                    self.stop_view = True

        time.sleep(0.001)
        if cv2.getWindowProperty(window_name, cv2.WND_PROP_VISIBLE) > 1:
            cv2.destroyWindow(window_name)

    # ==================== Server methods ====================
    def encode_data_transfer(self):
        data = self.get_data()

        if data is not None:
            string_data = "{\"PointCloudData\":\""
            string_data += ';'.join(str('%.3f' % val) for val in data.flatten())
            string_data += "\"}"

            return bytes(str(string_data), 'utf8')
        return None

    # ==================== Pack data for saving ====================
    def pack_data(self, ts_stop, frame_id):
        # https://carla.readthedocs.io/en/latest/ref_sensors/#lidar-sensor
        save_intensity = True  # Check docs

        points = self.get_data().copy()
        points[:, [0, 1]] = points[:, [1, 0]]

        if not save_intensity:
            points = points[:, :3]

        packed_data = {
            'name': '{}'.format(ts_stop),
            'lidar_data': points  # [[y, x, z]...] or [[y, x, z, i]...]
        }

        return packed_data

    # ==================== Custom methods ====================
    def view_lidar(self, points, window_name):
        img_size = (self.width, self.height)

        lidar_data = np.array(points[:, :2])
        lidar_data *= min(img_size) / (2.0 * self.range)
        lidar_data += (0.5 * img_size[0], 0.5 * img_size[1])
        lidar_data = np.fabs(lidar_data)  # pylint: disable=E1111
        lidar_data = lidar_data.astype(np.int32)
        lidar_data = np.reshape(lidar_data, (-1, 2))
        lidar_img_size = (img_size[0], img_size[1], 3)
        lidar_img = np.zeros(lidar_img_size, dtype=np.uint8)
        lidar_img[tuple(lidar_data.T)] = (255, 255, 255)
        lidar_img = cv2.flip(lidar_img, 0)

        cv2.imshow(window_name, lidar_img)
        return cv2.waitKey(self.viewer_delay)


if __name__ == '__main__':
    print('NOT runnable. Check \'Run_Carla.py\'')
