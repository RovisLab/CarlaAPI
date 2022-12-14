"""
 * ImuServer.py
 *
 *  Created on: 31.08.2022
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

from numpy import vectorize

from sensors import get_transform
from scipy.spatial.transform import Rotation

"""
Documentation:
    https://carla.readthedocs.io/en/latest/ref_sensors/#imu-sensor
"""

try:
    sys.path.append(glob.glob('carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

import carla


class IMUSensor(object):
    def __init__(self, name, car):
        self.name = name
        self.car = car
        self.sensor = None
        self.sensor_data = None

    def setup_imu(self):
        world = self.car.get_world()
        imu_bp = world.get_blueprint_library().find('sensor.other.imu')

        self.sensor = world.spawn_actor(
            imu_bp, get_transform('null'), attach_to=self.car
        )
        weak_self = weakref.ref(self)
        self.sensor.listen(
            lambda sensor_data: weak_self().imu_callback(weak_self, sensor_data)
        )

    def process_sensor_data(self):
        if self.sensor_data is not None:
            limits = (-99.9, 99.9)

            # Convert [0, 360] heading to [-180, 180] degrees
            heading = self.sensor_data.compass
            #if heading > math.pi:
            #    heading = heading - 2. * math.pi

            # heading = -heading

            rot = Rotation.from_euler('xyz', [0.0, 0.0, heading], degrees=False)
            quat = rot.as_quat()

            data = [
                max(limits[0], min(limits[1], self.sensor_data.accelerometer.x)),   # Accelerometer X
                max(limits[0], min(limits[1], self.sensor_data.accelerometer.y)),   # Accelerometer Y
                max(limits[0], min(limits[1], self.sensor_data.accelerometer.z)),   # Accelerometer Z
                max(limits[0], min(limits[1], math.degrees(self.sensor_data.gyroscope.x))),     # Gyroscope X
                max(limits[0], min(limits[1], math.degrees(self.sensor_data.gyroscope.y))),     # Gyroscope Y
                max(limits[0], min(limits[1], math.degrees(self.sensor_data.gyroscope.z))),     # Gyroscope Z
                math.cos(self.sensor_data.compass),     # Magnetometer X
                math.sin(self.sensor_data.compass),     # Magnetometer Y
                0.0,                                    # Magnetometer Z
                quat[0],    # x
                quat[1],    # y
                quat[2],    # z
                quat[3]     # w
            ]

            # Debug
            cos_a = math.cos(math.radians(180))
            sin_a = math.sin(math.radians(180))
            R = np.array([[cos_a, -sin_a, 0], [sin_a, cos_a, 0], [0, 0, 1]])

            position = np.array([self.car.get_location().x, self.car.get_location().y, self.car.get_location().z])
            velocity = self.car.get_velocity()

            rot = R @ position
            speed = math.sqrt(velocity.x ** 2 + velocity.y ** 2 + velocity.z ** 2)

            # print("pose:", self.car.get_location(), math.degrees(heading))
            # print("heading:", math.degrees(heading))
            # print("velocity:", velocity)
            # print("speed: {:.3f}".format(speed))
            # print("")

            return data
        return None

    def init_view(self):
        threading.Thread(target=self.do_view).start()

    def do_view(self):
        print('{}: IMU view not yet implemented'.format(self.name))
        # while self.sensor is not None:
        #     pass
        # time.sleep(0.001)

    @staticmethod
    def imu_callback(weak_ref, sensor_data):
        self = weak_ref()
        # print("imu callback, ", self._parent)
        self.sensor_data = sensor_data

    def destroy(self):
        self.sensor.destroy()


class IMUServer(object):
    def __init__(self,
                 ip,
                 port,
                 dt,
                 imu_sensor):
        if isinstance(ip, str):
            self.ip = bytes(ip, "utf-8")
        else:
            self.ip = ip
        self.port = port
        self.dt = dt
        self.host = None
        self.event = None
        self.imu_sensor = imu_sensor
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
                print(' - {} connected to {}'.format(self.imu_sensor.name, self.event.peer.address))

            elif self.event.type == enet.EVENT_TYPE_DISCONNECT:
                print(' - {} disconnected from {}'.format(self.imu_sensor.name, self.event.peer.address))
                self.is_terminated = True
            self.host.flush()
            time.sleep(0.01)

    def do_send(self):
        while not self.is_terminated:
            if self.imu_sensor is not None:
                data = self.imu_sensor.process_sensor_data()
                # data = [accX, accY, accZ, gyroX, gyroY, gyroZ, magnetX, magnetY, magnetZ, roll, pitch, yaw]

                if data is not None:
                    # View IMU
                    # print('\nIMU data:')
                    # print('  - Accelerometer xyz: {}'.format(data[0:3]))
                    # print('  - Gyroscope xyz: {}'.format(data[3:6]))
                    # print('  - Magnetometer xyz: {}'.format(data[6:9]))

                    string_data = ""
                    for val in data:
                        string_data += str('%.6f' % val) + ";"
                    string_data += "&"

                    msg = json.dumps({"ImuData": string_data})
                    msg = bytes(msg, 'utf8')

                    packet = enet.Packet(msg, enet.PACKET_FLAG_RELIABLE)
                    self.host.broadcast(0, packet)
                elif self.event.type == enet.EVENT_TYPE_DISCONNECT:
                    break

            time.sleep(self.dt + 0.015)


def tu_IMUServer():
    # TODO Not runnable!!!!
    print("NOT RUNNABLE. Execute CarlaClient.py")


if __name__ == '__main__':
    try:
        tu_IMUServer()
    except SystemExit:
        pass
