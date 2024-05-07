import sys
import glob
import os
import enet
import time
import threading
import weakref
import numpy as np
import cv2
import base64

try:
    sys.path.append(glob.glob('carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
    # sys.path.append(glob.glob('carla-0.9.10-py3.7-win-amd64.egg'))
except IndexError:
    pass

import carla

# Global variables
LOG_DEPTH = True  # Logarithmic normalisation


class DepthSensor(object):
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

    def setup_depth(self, camera_transform):
        world = self._parent.get_world()
        camera_bp = world.get_blueprint_library().find('sensor.camera.depth')
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

    def process_image(self):
        if self.image is not None:
            array = np.frombuffer(self.image.raw_data, dtype=np.dtype("uint8"))
            array = np.reshape(array, (self.image.height, self.image.width, 4))
            array = array[:, :, :3]
            raw = array

            # Convert to depth gray
            array = array.astype(np.float32)  # shape = (480, 640, 3)
            # Apply (R + G * 256 + B * 256 * 256) / (256 * 256 * 256 - 1).
            array = np.dot(array, [65536.0, 256.0, 1.0])
            array /= 16777215.0  # (256.0 * 256.0 * 256.0 - 1.0)

            # Apply the logarithmic normalisation
            if LOG_DEPTH:
                logdepth = np.ones(array.shape) + (np.log(array) / 5.70378)
                logdepth = np.clip(logdepth, 0.0, 1.0)
                logdepth *= 255.0
                # Expand to three colors.
                array = np.repeat(logdepth[:, :, np.newaxis], 3, axis=2)
            else:
                array = array * 1000
            depth = array

            # Change what to return / to be sent
            # return raw, depth
            return None, depth
            # return raw, None
        return None, None

    def init_view(self):
        threading.Thread(target=self.do_view).start()

    def do_view(self):
        while self.sensor is not None:
            _, depth = self.process_image()

            if depth is not None:
                cv2.imshow(self.name, depth)
                cv2.waitKey(1)
        time.sleep(0.001)

    @staticmethod
    def camera_callback(weak_ref, image):
        self = weak_ref()
        # print("camera callback, ", self._parent)
        self.img_rgb = image

    def destroy(self):
        self.sensor.destroy()


class DepthServer(object):
    def __init__(self, ip, port, dt, depth_sensor):
        if isinstance(ip, str):
            self.ip = bytes(ip, "utf-8")
        else:
            self.ip = ip
        self.port = port
        self.dt = dt
        self.host = None
        self.event = None
        self.depth_sensor = depth_sensor
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
                print(' - {} connected to {}'.format(self.depth_sensor.name, self.event.peer.address))

            elif self.event.type == enet.EVENT_TYPE_DISCONNECT:
                print(' - {} disconnected from {}'.format(self.depth_sensor.name, self.event.peer.address))
                self.is_terminated = True
            self.host.flush()
            time.sleep(0.01)

    def do_send(self):
        while not self.is_terminated:
            if self.depth_sensor is not None:
                raw, depth = self.depth_sensor.process_img_rgb()

                # cv2.imshow("Test_raw", raw)
                # cv2.imshow("Test_depth", depth)
                # cv2.waitKey(1)

                images_data = '{\"ImagesDataRaw\":\"'
                if raw is not None:
                    raw_base = str(base64.b64encode(cv2.imencode(".jpg", raw)[1]))
                    raw_base = raw_base[2:len(raw_base) - 1]
                    images_data += "{}&".format(raw_base)
                images_data += '\",\"ImagesDataDepth\":\"'
                if depth is not None:
                    depth_base = str(base64.b64encode(cv2.imencode(".jpg", depth)[1]))
                    depth_base = depth_base[2:len(depth_base) - 1]
                    images_data += "{}&".format(depth_base)
                images_data += '\"}'

                if raw is not None or depth is not None:
                    encoded_data = bytes(str(images_data), 'utf8')
                    packet = enet.Packet(encoded_data, enet.PACKET_FLAG_RELIABLE)
                    self.host.broadcast(0, packet)
                elif self.event.type == enet.EVENT_TYPE_DISCONNECT:
                    break
            time.sleep(self.dt + 0.015)


def tu_DepthServer():
    # TODO Not runnable!!!!
    print("NOT RUNNABLE. Execute CarlaClient.py")


if __name__ == '__main__':
    try:
        tu_DepthServer()
    except SystemExit:
        pass
