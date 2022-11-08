from absl import app
import sys
import glob
import enet
import time
import threading
import math
import numpy as np
import cv2
import json
import base64
import weakref
import os
from sensors import get_transform

try:
    sys.path.append(glob.glob('carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
    # sys.path.append(glob.glob('carla-0.9.10-py3.7-win-amd64.egg'))
except IndexError:
    pass

import carla


class MonoCameraSensor(object):
    def __init__(self, name,
                 parent_actor,
                 view_width=640,
                 view_height=480,
                 view_fov=90):
        self.name = name
        self.sensor = None
        self.image = None
        self._parent = parent_actor
        self.view_width = view_width
        self.view_height = view_height
        self.view_fov = view_fov
        self.capture = True

    def setup_camera(self, camera_transform):
        world = self._parent.get_world()
        camera_bp = world.get_blueprint_library().find('sensor.camera.rgb')
        camera_bp.set_attribute('image_size_x', str(int(self.view_width)))
        camera_bp.set_attribute('image_size_y', str(int(self.view_height)))
        camera_bp.set_attribute('fov', str(self.view_fov))

        self.sensor = world.spawn_actor(
            camera_bp, camera_transform, attach_to=self._parent
        )
        weak_self = weakref.ref(self)
        self.sensor.listen(
            lambda image: weak_self().camera_callback(weak_self, image)
        )

    def get_intrinsics(self):
        """
        Sources:
            https://github.com/carla-simulator/carla/issues/56
            https://github.com/carla-simulator/carla/blob/master/PythonAPI/examples/lidar_to_camera.py - line 113
        """
        f_length = self.view_width / (2.0 * np.tan(self.view_fov * np.pi / 360.0))  # Focal Length
        cx = self.view_width / 2  # Center X
        cy = self.view_height / 2  # Center Y

        mat = [[f_length, 0,        cx],
               [0,        f_length, cy],
               [0,        0,        1]]

        return mat

    def process_image(self):
        if self.image is not None:
            array = np.frombuffer(self.image.raw_data, dtype=np.dtype("uint8"))
            array = np.reshape(array, (self.image.height, self.image.width, 4))
            array = array[:, :, :3]
            return array
        return None

    def init_view(self):
        threading.Thread(target=self.do_view).start()

    def do_view(self):
        while self.sensor is not None:
            image = self.process_image()

            if image is not None:
                cv2.imshow(self.name, image)
                cv2.waitKey(1)
        time.sleep(0.001)

    @staticmethod
    def camera_callback(weak_ref, image):
        self = weak_ref()
        # print("camera callback, ", self._parent)
        # self.image = image
        if self.capture:
            self.image = image
            self.capture = False

    def destroy(self):
        self.sensor.destroy()


class MonoCameraServer(object):
    def __init__(self,
                 ip,
                 port,
                 dt,
                 cam_sensor):
        if isinstance(ip, str):
            self.ip = bytes(ip, "utf-8")
        else:
            self.ip = ip
        self.port = port
        self.dt = dt
        self.host = None
        self.event = None
        self.cam_sensor = cam_sensor
        self.is_terminated = False

    def init_server(self):
        self.host = enet.Host(enet.Address(self.ip, self.port), 10, 0, 0, 0)
        self.event = self.host.service(1000)
        threading.Thread(target=self.do_service).start()
        threading.Thread(target=self.do_send).start()

    def do_service(self):
        while not self.is_terminated:
            self.event = self.host.service(10)

            if self.event.type == enet.EVENT_TYPE_CONNECT:
                print(' - {} connected to {}'.format(self.cam_sensor.name, self.event.peer.address))

            elif self.event.type == enet.EVENT_TYPE_DISCONNECT:
                print(' - {} disconnected from {}'.format(self.cam_sensor.name, self.event.peer.address))
                self.is_terminated = True
            self.host.flush()
            time.sleep(0.01)

    def do_send(self):
        while not self.is_terminated:
            if self.cam_sensor is not None:
                data = self.cam_sensor.process_image()

                if data is not None:
                    images_data = "{\"ImagesData\":\""
                    front_base = str(base64.b64encode(cv2.imencode(".jpg", data)[1]))
                    front_base = front_base[2:len(front_base) - 1]
                    images_data += "{}".format(front_base)
                    images_data += "&\"}"

                    encoded_data = bytes(str(images_data), 'utf8')
                    packet = enet.Packet(encoded_data, enet.PACKET_FLAG_RELIABLE)
                    self.host.broadcast(0, packet)
                elif self.event.type == enet.EVENT_TYPE_DISCONNECT:
                    break
            time.sleep(self.dt + 0.015)


def tu_ImageServer():
    # TODO Not runnable!!!!
    print("NOT RUNNABLE. Execute CarlaClient.py")


if __name__ == '__main__':
    try:
        tu_ImageServer()
    except SystemExit:
        pass
