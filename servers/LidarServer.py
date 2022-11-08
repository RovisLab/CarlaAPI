"""
 * LidarServer.py
 *
 *  Created on: 30.08.2022
 *      Author: Tomuta Gabriel
"""

import sys
import glob
import enet
import time
import threading
import json
import numpy as np
import cv2
import base64
import weakref
import os

try:
    sys.path.append(glob.glob('carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

import carla


class LidarSensor(object):
    def __init__(self, name, parent_actor):
        self.name = name
        self._parent = parent_actor
        self.sensor = None
        self.point_cloud = None

    def setup_lidar(self, lidar_transform):
        world = self._parent.get_world()
        lidar_bp = world.get_blueprint_library().find('sensor.lidar.ray_cast')
        lidar_bp.set_attribute('range', str(50))
        lidar_bp.set_attribute('channels', str(32))
        lidar_bp.set_attribute('points_per_second', str(90000))
        lidar_bp.set_attribute('rotation_frequency', str(40))

        self.sensor = world.spawn_actor(
            lidar_bp, lidar_transform, attach_to=self._parent,
            attachment_type=carla.AttachmentType.Rigid
        )
        weak_self = weakref.ref(self)
        self.sensor.listen(
            lambda point_cloud: weak_self().lidar_callback(weak_self, point_cloud)
        )

    def process_point_cloud(self):
        if self.point_cloud is not None:
            points = np.frombuffer(self.point_cloud.raw_data, dtype=np.dtype('f4'))
            points = np.reshape(points, (int(points.shape[0] / 3), 3))

            return points
        return None

    def view_lidar(self, points):
        img_size = (400, 500)  # (width, height)

        lidar_data = np.array(points[:, :2])
        lidar_data *= min(img_size) / 100.0
        lidar_data += (0.5 * img_size[0], 0.5 * img_size[1])
        lidar_data = np.fabs(lidar_data)  # pylint: disable=assignment-from-no-return
        lidar_data = lidar_data.astype(np.int32)
        lidar_data = np.reshape(lidar_data, (-1, 2))
        lidar_img_size = (img_size[0], img_size[1], 3)
        lidar_img = np.zeros(lidar_img_size)
        lidar_img[tuple(lidar_data.T)] = (255, 255, 255)
        lidar_img = cv2.rotate(lidar_img, cv2.ROTATE_90_CLOCKWISE)
        lidar_img = cv2.flip(lidar_img, 1)

        cv2.imshow(self.name, lidar_img)
        cv2.waitKey(1)

    def init_view(self):
        threading.Thread(target=self.do_view).start()

    def do_view(self):
        while self.sensor is not None:
            points = self.process_point_cloud()

            if points is not None:
                self.view_lidar(points)
        time.sleep(0.001)

    @staticmethod
    def lidar_callback(weak_ref, point_cloud):
        self = weak_ref()
        # print("lidar callback, ", self._parent)
        self.point_cloud = point_cloud

    def destroy(self):
        self.sensor.destroy()


class LidarServer(object):
    def __init__(self,
                 ip,
                 port,
                 dt,
                 lidar_sensor):
        if isinstance(ip, str):
            self.ip = bytes(ip, "utf-8")
        else:
            self.ip = ip
        self.port = port
        self.dt = dt
        self.host = None
        self.event = None
        self.lidar_sensor = lidar_sensor
        self.is_terminated = False
        self.send_empty_packet = True

    def init_server(self):
        self.host = enet.Host(enet.Address(self.ip, self.port), 10, 0, 0, 0)
        self.event = self.host.service(1000)
        threading.Thread(target=self.do_service).start()
        threading.Thread(target=self.do_send).start()

    def do_service(self):
        while not self.is_terminated:
            self.event = self.host.service(10)

            if self.event.type == enet.EVENT_TYPE_CONNECT:
                print(' - {} connected to {}'.format(self.lidar_sensor.name, self.event.peer.address))

            elif self.event.type == enet.EVENT_TYPE_DISCONNECT:
                print(' - {} disconnected from {}'.format(self.lidar_sensor.name, self.event.peer.address))
                self.is_terminated = True
            self.host.flush()
            time.sleep(0.01)

    def do_send(self):
        while not self.is_terminated:
            if self.lidar_sensor is not None:
                data = self.lidar_sensor.process_point_cloud()

                if data is not None:
                    # View Lidar
                    # self.lidar_sensor.view_lidar(data)

                    string_data = ';'.join(str('%.3f' % val) for val in data.flatten())
                    # print(string_data)

                    msg = json.dumps({"PointCloudData": string_data})
                    msg = bytes(msg, 'utf8')

                    packet = enet.Packet(msg, enet.PACKET_FLAG_RELIABLE)
                    self.host.broadcast(0, packet)
                elif self.event.type == enet.EVENT_TYPE_DISCONNECT:
                    break

            time.sleep(self.dt + 0.015)


def tu_LidarServer():
    # TODO Not runnable!!!!
    print("NOT RUNNABLE. Execute CarlaClient.py")


if __name__ == '__main__':
    try:
        tu_LidarServer()
    except SystemExit:
        pass
