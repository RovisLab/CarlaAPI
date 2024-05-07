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

class CameraSensor(object):
    def __init__(self, name,
                 parent_actor,
                 view_width=640,
                 view_height=360,
                 view_fov=90):
        self.name = name
        self.sensor_rgb = None
        self.sensor_depth = None
        self.img_rgb = None
        self.img_depth = None
        self._parent = parent_actor
        self.view_width = view_width
        self.view_height = view_height
        self.view_fov = view_fov

    def setup_cameras(self, camera_transform):
        self.setup_camera_rgb(camera_transform)
        self.setup_camera_depth(camera_transform)

    def setup_camera_rgb(self, camera_transform):
        world = self._parent.get_world()
        weak_self = weakref.ref(self)
        camera_bp_rgb = world.get_blueprint_library().find('sensor.camera.rgb')
        camera_bp_rgb.set_attribute('image_size_x', str(int(self.view_width)))
        camera_bp_rgb.set_attribute('image_size_y', str(int(self.view_height)))
        camera_bp_rgb.set_attribute('fov', str(self.view_fov))
        self.sensor_rgb = world.spawn_actor(camera_bp_rgb, camera_transform, attach_to=self._parent)
        self.sensor_rgb.listen(lambda image: weak_self().camera_rgb_callback(weak_self, image))

    def setup_camera_depth(self, camera_transform):
        world = self._parent.get_world()
        weak_self = weakref.ref(self)
        camera_bp_depth = world.get_blueprint_library().find('sensor.camera.depth')
        camera_bp_depth.set_attribute('image_size_x', str(int(self.view_width)))
        camera_bp_depth.set_attribute('image_size_y', str(int(self.view_height)))
        camera_bp_depth.set_attribute('fov', str(self.view_fov))
        self.sensor_depth = world.spawn_actor(camera_bp_depth, camera_transform, attach_to=self._parent)
        self.sensor_depth.listen(lambda image: weak_self().camera_depth_callback(weak_self, image))

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

    def process_img_rgb(self):
        if self.img_rgb is not None:
            array = np.frombuffer(self.img_rgb.raw_data, dtype=np.dtype("uint8"))
            array = np.reshape(array, (self.img_rgb.height, self.img_rgb.width, 4))
            return array[:, :, :3]
        return None

    def process_img_depth(self):
        if self.img_depth is not None:
            array = np.frombuffer(self.img_depth.raw_data, dtype=np.dtype("uint8"))
            array = np.reshape(array, (self.img_depth.height, self.img_depth.width, 4))
            array = array[:, :, :3]

            # array = array[:, :, ::-1]
            # gray_depth = ((array[:, :, 0] + array[:, :, 1] * 256.0 + array[:, :, 2] * 256.0 * 256.0) / ((256.0 * 256.0 * 256.0) - 1))
            # gray_depth = gray_depth.astype(np.float32)
            # gray_depth = gray_depth * 1000.0

            return array
        return None

    def init_view(self):
        threading.Thread(target=self.do_view).start()

    def do_view(self):
        while self.sensor_rgb is not None and self.sensor_depth is not None:
            img_rgb = self.process_img_rgb()
            if img_rgb is not None:
                cv2.imshow(self.name + " RGB", img_rgb)

            # img_depth = self.process_img_depth()
            # if img_depth is not None:
            #     img_depth = img_depth.astype(np.uint8)
            #     cv2.imshow(self.name + " Depth", img_depth)

            cv2.waitKey(1)

    @staticmethod
    def camera_rgb_callback(weak_ref, image):
        self = weak_ref()
        self.img_rgb = image

    @staticmethod
    def camera_depth_callback(weak_ref, image):
        self = weak_ref()
        self.img_depth = image

    def destroy(self):
        self.sensor_rgb.destroy()
        self.sensor_depth.destroy()


class CameraServer(object):
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
                rgb = self.cam_sensor.process_img_rgb()
                depth = self.cam_sensor.process_img_depth()

                if rgb is not None and depth is not None:
                    images_data = "{\"RGB\":\""
                    front_base = str(base64.b64encode(cv2.imencode(".jpg", rgb)[1]))
                    front_base = front_base[2:len(front_base) - 1]
                    images_data += "{}".format(front_base)

                    images_data += '\",\"Depth\":\"'
                    depth_base = str(base64.b64encode(cv2.imencode(".png", depth)[1]))
                    depth_base = depth_base[2:len(depth_base) - 1]
                    images_data += "{}&".format(depth_base)

                    images_data += "&\"}"

                    encoded_data = bytes(str(images_data), 'utf8')
                    packet = enet.Packet(encoded_data, enet.PACKET_FLAG_RELIABLE)
                    self.host.broadcast(0, packet)

                    print("Send image\t[{}]".format(time.time()))
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
