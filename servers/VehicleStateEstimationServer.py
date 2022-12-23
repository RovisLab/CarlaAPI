"""
vehicle state = [x position, y position, speed, heading, steering angle]
"""

import enet
import time
import threading
import math
import numpy as np
import json


def state_measurement_data(car, imu):
    if not car.is_alive:
        return ""

    # Get IMU compass (Carla coord sys)
    # N (-Y axis):  0
    # E (X axis):   90
    # S (Y axis):   180
    # W (-x axis):  270

    heading_carla = 0.0 if imu.sensor_data is None else imu.sensor_data.compass

    # Rovis heading wrt W (0 is East)
    heading_rovis = math.pi / 2 - heading_carla

    if heading_rovis < -math.pi:
        heading_rovis = 2 * math.pi + heading_rovis

    # Convert position to rovis coord sys
    R = np.array([[1, 0, 0], [0, -1, 0], [0, 0, 1]])
    position = R @ np.array([car.get_location().x, car.get_location().y, car.get_location().z])
    velocity = R @ np.array([car.get_velocity().x, car.get_velocity().y, car.get_velocity().z])
    speed = math.sqrt(velocity[0] ** 2 + velocity[1] ** 2 + velocity[2] ** 2)

    _string = ""
    _string += str('%0.4f' % position[0]) + ";"  # Get x
    _string += str('%0.4f' % position[1]) + ";"  # Get y
    if speed != 0:
        _string += str('%.6f' % speed) + ";"
    else:
        _string += "0;"

    _string += str('%.6f' % heading_rovis) + ";"  # Get yaw angle
    _string += str('%.6f' % 0) + ";"  # TODO: Steering angle

    return _string + "/" + "&2&1;&"


class StateMeasurementServer(object):
    def __init__(self, ip, port, dt, car, imu, client_name):
        if isinstance(ip, str):
            self.ip = bytes(ip, "utf-8")
        else:
            self.ip = ip
        self.port = port
        self.dt = dt
        self.host = None
        self.event = None
        self.car = car
        self.imu = imu
        self.client_name = client_name
        self.is_terminated = False

    def init_server(self):
        self.host = enet.Host(enet.Address(self.ip, self.port), 10, 0, 0, 0)
        self.event = self.host.service(1000)
        threading.Thread(target=self.do_service, daemon=True).start()
        threading.Thread(target=self.do_send, args=(self.car, self.imu), daemon=True).start()

    def do_service(self):
        while not self.is_terminated:
            self.event = self.host.service(10)

            if self.event.type == enet.EVENT_TYPE_CONNECT:
                print(' - {} State Measurement connected to {}'.format(self.client_name, self.event.peer.address))

            elif self.event.type == enet.EVENT_TYPE_DISCONNECT:
                print(' - {} State Measurement disconnected from {}'.format(self.client_name, self.event.peer.address))
                self.is_terminated = True
            self.host.flush()
            time.sleep(0.001)

    def do_send(self, car, imu):
        while not self.is_terminated:
            data = state_measurement_data(car, imu)
            if data != "":
                msg = json.dumps({"CarData": data})
                msg = bytes(msg, 'utf8')

                packet = enet.Packet(msg, enet.PACKET_FLAG_RELIABLE)
                self.host.broadcast(0, packet)
            else:
                break
            time.sleep(self.dt + 0.015)
