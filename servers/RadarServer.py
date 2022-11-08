"""
 * RadarServer.py
 *
 *  Created on: 30.08.2022
 *      Author: Tomuta Gabriel
"""

import sys
import glob
import enet
import time
import math
import json
import threading
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


class RadarSensor(object):
    def __init__(self, name, parent_actor):
        self.name = name
        self._parent = parent_actor
        self.sensor = None
        self.radar_data = None

    def setup_radar(self, radar_transform):
        world = self._parent.get_world()
        radar_bp = world.get_blueprint_library().find('sensor.other.radar')
        radar_bp.set_attribute('horizontal_fov', str(35))
        radar_bp.set_attribute('vertical_fov', str(30))
        radar_bp.set_attribute('range', str(100))
        radar_bp.set_attribute('points_per_second', str(1500))

        self.sensor = world.spawn_actor(
            radar_bp, radar_transform, attach_to=self._parent,
            attachment_type=carla.AttachmentType.Rigid
        )
        weak_self = weakref.ref(self)
        self.sensor.listen(
            lambda radar_data: weak_self().radar_callback(weak_self, radar_data)
        )

    def process_radar(self):
        if self.radar_data is not None:
            points = np.frombuffer(self.radar_data.raw_data, dtype=np.dtype('f4'))
            points = np.reshape(points, (len(self.radar_data), 4))

            return points  # [[vel, azimuth, altitude, depth],...[,,,]]
        return None

    def draw_radar(self, world):
        if self.radar_data is None:
            return

        velocity_range = 7.5  # m/s
        current_rot = self.radar_data.transform.rotation
        for detect in self.radar_data:
            azi = math.degrees(detect.azimuth)
            alt = math.degrees(detect.altitude)
            # The 0.25 adjusts a bit the distance so the dots can
            # be properly seen
            fw_vec = carla.Vector3D(x=detect.depth - 0.25)
            carla.Transform(
                carla.Location(),
                carla.Rotation(
                    pitch=current_rot.pitch + alt,
                    yaw=current_rot.yaw + azi,
                    roll=current_rot.roll)).transform(fw_vec)

            def clamp(min_v, max_v, value):
                return max(min_v, min(value, max_v))

            norm_velocity = detect.velocity / velocity_range  # range [-1, 1]
            r = int(clamp(0.0, 1.0, 1.0 - norm_velocity) * 255.0)
            g = int(clamp(0.0, 1.0, 1.0 - abs(norm_velocity)) * 255.0)
            b = int(abs(clamp(- 1.0, 0.0, - 1.0 - norm_velocity)) * 255.0)
            world.debug.draw_point(
                self.radar_data.transform.location + fw_vec,
                size=0.075,
                life_time=0.06,
                persistent_lines=False,
                color=carla.Color(r, g, b))

    def init_view(self):
        threading.Thread(target=self.do_view).start()

    def do_view(self):
        print(' - {}: Radar view not yet implemented'.format(self.name))
        # while self.sensor is not None:
        #     pass
        # time.sleep(0.001)

    @staticmethod
    def radar_callback(weak_ref, radar_data):
        self = weak_ref()
        # print("radar callback, ", self._parent)
        self.radar_data = radar_data

    def destroy(self):
        self.sensor.destroy()


class RadarServer(object):
    def __init__(self,
                 ip,
                 port,
                 dt,
                 radar_sensor):
        if isinstance(ip, str):
            self.ip = bytes(ip, "utf-8")
        else:
            self.ip = ip
        self.port = port
        self.dt = dt
        self.host = None
        self.event = None
        self.radar_sensor = radar_sensor
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
                print(' - {} connected to {}'.format(self.radar_sensor.name, self.event.peer.address))

            elif self.event.type == enet.EVENT_TYPE_DISCONNECT:
                print(' - {} disconnected from {}'.format(self.radar_sensor.name, self.event.peer.address))
                self.is_terminated = True
            self.host.flush()
            time.sleep(0.01)

    def do_send(self):
        while not self.is_terminated:
            if self.radar_sensor is not None:
                data = self.radar_sensor.process_radar()

                if data is not None:
                    string_data = ';'.join(str('%.3f' % val) for val in data.flatten())
                    # print(string_data)

                    msg = json.dumps({"RadarData": string_data})
                    msg = bytes(msg, 'utf8')

                    packet = enet.Packet(msg, enet.PACKET_FLAG_RELIABLE)
                    self.host.broadcast(0, packet)
                elif self.event.type == enet.EVENT_TYPE_DISCONNECT:
                    break

            time.sleep(self.dt + 0.015)


def tu_RadarServer():
    # TODO Not runnable!!!!
    print("NOT RUNNABLE. Execute CarlaClient.py")


if __name__ == '__main__':
    try:
        tu_RadarServer()
    except SystemExit:
        pass
