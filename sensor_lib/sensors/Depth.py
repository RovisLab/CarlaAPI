import os
import sys
import glob
import time
import base64
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


class DepthSensor(BaseSensor):
    def __init__(self, name, parent_actor, client_args):
        super().__init__(name, parent_actor, client_args)
        self.has_capture = True
        self.channels = 3

    # ==================== General sensor methods ====================
    def setup(self):
        world = self._parent.get_world()
        camera_bp = world.get_blueprint_library().find('sensor.camera.depth')
        camera_bp.set_attribute('image_size_x', str(int(self.width)))
        camera_bp.set_attribute('image_size_y', str(int(self.height)))
        camera_bp.set_attribute('fov', str(self.fov))

        self.sensor = world.spawn_actor(
            camera_bp, self.sensor_transform, attach_to=self._parent
        )
        weak_self = weakref.ref(self)
        self.sensor.listen(
            lambda data: weak_self().sensor_callback(weak_self, data)
        )

    def get_data(self, optional_data_type=''):
        if optional_data_type == '' or optional_data_type not in ['raw', 'gray', 'both']:
            optional_data_type = 'gray'

        if self.data is not None:
            array = np.frombuffer(self.data.raw_data, dtype=np.dtype("uint8"))
            array = np.reshape(array, (self.data.height, self.data.width, 4))
            array = array[:, :, :3]  # Carla Raw depth

            # Convert carla raw to gray depth
            array = 1000 * self.normalize_depth_img(array)  # scaled to 1000m
            gray = cv2.cvtColor(np.float32(array.copy()), cv2.COLOR_GRAY2BGR) / 256

            if optional_data_type == 'gray':
                return gray

            # Convert gray depth to rovis raw depth
            rovis_raw = self.depth2rovis(array) / 256

            if optional_data_type == 'raw':
                return rovis_raw
            elif optional_data_type == 'both':
                return rovis_raw, gray

        if optional_data_type == 'both':
            return None, None
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
            # img_raw, img_gray = self.get_data('both')
            img_depth = self.get_data('raw')  # gray, raw

            key = None
            if img_depth is not None:
                cv2.imshow(window_name, img_depth)
                key = cv2.waitKey(self.viewer_delay)

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
        rovis_raw = self.get_data('raw')

        if rovis_raw is not None:
            images_data = '{\"ImagesDataRaw\":\"'
            front_base = str(base64.b64encode(cv2.imencode(".jpg", raw)[1]))
            front_base = front_base[2:len(front_base) - 1]
            images_data += "{}&\"".format(front_base)
            images_data += ', \"ImagesDataDepth\":\"\"'  # TODO remove
            images_data += '}'

            return bytes(str(images_data), 'utf8')
        return None

    # ==================== Pack data for saving ====================
    def pack_data(self, ts_stop, frame_id):
        image = self.get_data('raw')

        packed_data = {
            'name': '{}.png'.format(ts_stop),
            'image': image*256,
        }
        return packed_data

    # ==================== Custom methods ====================
    @staticmethod
    def normalize_depth_img(depth_img: np.ndarray = None):
        return np.float32(np.sum(np.multiply(depth_img, np.asarray([256 ** 2, 256, 1])), axis=-1) / (256 ** 3 + 1))

    @staticmethod
    def depth2rovis(in_depth: np.ndarray = None):
        # 3 channels of uint8 = 24 bits
        # 2 ^ 24 = 16777216
        # assuming max distance of 1000m => scale of 16777.216
        scale = 16777.216

        depth = np.int32(in_depth.copy() * scale)
        img = np.zeros(shape=(depth.shape[0], depth.shape[1], 3), dtype=np.uint8)

        for k in range(img.shape[2]):
            img[:, :, k] = (depth[:, :] >> (8 * k)) & 0xFF

        return img


if __name__ == '__main__':
    print('NOT runnable. Check \'Run_Carla.py\'')
