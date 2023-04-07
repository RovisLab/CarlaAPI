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


class OpticalFlowSensor(BaseSensor):
    def __init__(self, name, parent_actor, client_args):
        super().__init__(name, parent_actor, client_args)
        self.has_capture = True

    # ==================== General sensor methods ====================
    def setup(self):
        world = self._parent.get_world()
        camera_bp = world.get_blueprint_library().find('sensor.camera.optical_flow')
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
        if self.data is not None:
            array = np.frombuffer(self.data.raw_data, dtype=np.dtype("uint8"))
            array = np.reshape(array, (self.data.height, self.data.width, 4))
            array = array[:, :, :3]
            array = array[:, :, ::-1]
            return array
        return None

    @staticmethod
    def sensor_callback(weak_ref, data):
        self = weak_ref()
        if self.capture:
            self.data = data.get_color_coded_flow()
            self.capture = False

    # ==================== Viewer methods ====================
    def do_view(self):
        window_name = '{} - {}'.format(self.parent_name, self.name)
        while not self.stop_view and self.sensor is not None:
            image = self.get_data()

            key = None
            if image is not None:
                cv2.imshow(window_name, image)
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
        data = self.get_data()

        if data is not None:
            images_data = "{\"OpticalFlowData\":\""
            front_base = str(base64.b64encode(cv2.imencode(".png", data)[1]))
            front_base = front_base[2:len(front_base) - 1]
            images_data += "{}".format(front_base)
            images_data += "&\"}"

            return bytes(str(images_data), 'utf8')
        return None

    # ==================== Pack data for saving ====================
    def pack_data(self, ts_stop, frame_id):
        image = self.get_data()

        packed_data = {
            'name': '{}.png'.format(ts_stop),
            'image': image,
        }
        return packed_data


if __name__ == '__main__':
    print('NOT runnable. Check \'Run_Carla.py\'')
