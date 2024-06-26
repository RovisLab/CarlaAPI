import sys
import glob
import time
import base64
import threading
import numpy as np
import cv2
import weakref
import os

import config_utils as conf
from sensor_lib.BaseSensor import BaseSensor

""" 
This Sensor calculates the obstacles using Depth and SemSeg and shows them as a BEV image.
    author: Tomuta Gabriel
"""

try:
    sys.path.append(glob.glob('%s/PythonAPI/carla/dist/carla-*%d.%d-%s.egg' % (
        conf.CARLA_PATH,
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass
import carla


class ObstacleGaby(BaseSensor):
    def __init__(self, name, parent_actor, client_args):
        super().__init__(name, parent_actor, client_args)
        self.has_capture = False
        self.channels = 1

        self.seg_sensor = None
        self.seg_data = None

        self.depth_sensor = None
        self.depth_data = None

        self.seg_capture = False
        self.depth_capture = False

    # ==================== General sensor methods ====================
    def setup(self):
        world = self._parent.get_world()

        # Setup segmentation sensor
        seg_bp = world.get_blueprint_library().find('sensor.camera.semantic_segmentation')
        seg_bp.set_attribute('image_size_x', str(int(self.width)))
        seg_bp.set_attribute('image_size_y', str(int(self.height)))
        seg_bp.set_attribute('fov', str(self.fov))

        self.seg_sensor = world.spawn_actor(
            seg_bp, self.sensor_transform, attach_to=self._parent
        )
        weak_self = weakref.ref(self)
        self.seg_sensor.listen(
            lambda data: weak_self().seg_callback(weak_self, data)
        )

        # Setup depth sensor
        depth_bp = world.get_blueprint_library().find('sensor.camera.depth')
        depth_bp.set_attribute('image_size_x', str(int(self.width)))
        depth_bp.set_attribute('image_size_y', str(int(self.height)))
        depth_bp.set_attribute('fov', str(self.fov))

        self.depth_sensor = world.spawn_actor(
            depth_bp, self.sensor_transform, attach_to=self._parent
        )
        weak_self = weakref.ref(self)
        self.depth_sensor.listen(
            lambda data: weak_self().depth_callback(weak_self, data)
        )

    def get_data(self, optional_data_type=''):
        self.seg_capture = True
        self.depth_capture = True
        time.sleep(0.01)
        if self.seg_data is not None and self.depth_data is not None:

            # Get segmentation
            seg_array = np.frombuffer(self.seg_data.raw_data, dtype=np.dtype("uint8"))
            seg_array = np.reshape(seg_array, (self.seg_data.height, self.seg_data.width, 4))
            seg_array = seg_array[:, :, 2]  # Reduce the image to one channel

            # Get depth https://carla.readthedocs.io/en/latest/ref_sensors/#depth-camera
            depth_array = np.frombuffer(self.depth_data.raw_data, dtype=np.dtype("uint8"))
            depth_array = np.reshape(depth_array, (self.depth_data.height, self.depth_data.width, 4))
            depth_array = depth_array[:, :, :3]  # Carla Raw depth

            # Compute BEV
            cls_keep = [1, 2, 4, 8, 9, 10, 11]
            bev_img = np.zeros_like(seg_array)
            for j in range(self.seg_data.width):
                min_val = 255
                for i in range(self.seg_data.height):
                    if seg_array[i, j] in cls_keep:
                        min_val = min(min_val, int((depth_array[i, j, 2] + depth_array[i, j, 1] * 256 +
                                                    depth_array[i, j, 0] * 256 * 256) / (256 * 256 * 256 - 1)))
                focal_length = self.width / (2.0 * np.tan(self.fov * np.pi / 360.0))
                angle = np.arctan((j - self.width / 2) / focal_length)
                if angle != 0.0:
                    ip = (j - self.width / 2) / np.sin(angle)
                    ix = int(bev_img.shape[0] - 10 - np.cos(angle) * (min_val + ip))
                    iy = int(np.sin(angle) * (min_val + ip) + bev_img.shape[1] / 2)
                else:
                    ix = int(bev_img.shape[0] - 10 - focal_length - min_val)
                    iy = int(bev_img.shape[1] / 2)
                if bev_img.shape[0] > ix > 0 and bev_img.shape[1] > iy > 0:
                    # bev_img[ix, iy] = [1, 1, 1]
                    bev_img = cv2.circle(bev_img, (ix, iy), 2, (255, 255, 255), 2)

            return bev_img
        return None

    @staticmethod
    def sensor_callback(weak_ref, data, **kwargs):
        pass

    @staticmethod
    def seg_callback(weak_ref, data):
        self = weak_ref()
        if self.seg_capture:
            self.seg_data = data
            self.seg_capture = False

    @staticmethod
    def depth_callback(weak_ref, data):
        self = weak_ref()
        if self.depth_capture:
            self.depth_data = data
            self.depth_capture = False

    # ==================== Viewer methods ====================
    def do_view(self):
        window_name = '{} - {}'.format(self.parent_name, self.name)
        time.sleep(5)
        while not self.stop_view:
            image = self.get_data()

            if image is not None:
                cv2.imshow(window_name, image)
                key = cv2.waitKey(1)

                if key is not None and key != -1:
                    if key == 27:  # ESC - close everything
                        self.terminate_signal.emit()
                        self.stop_view = True
                    elif key == ord('q'):  # Close just the viewer
                        self.stop_view = True

                time.sleep(self.viewer_delay / 1000)

        time.sleep(0.001)
        if cv2.getWindowProperty(window_name, cv2.WND_PROP_VISIBLE) > 1:
            cv2.destroyWindow(window_name)

    # ==================== Server methods ====================
    def encode_data_transfer(self):
        return None

    # ==================== Pack data for saving ====================
    def pack_data(self, ts_stop, frame_id):
        return {}


if __name__ == '__main__':
    print('NOT runnable. Check \'Run_Carla.py\'')
