from absl import app
import enet
import time
import threading
import math
import numpy as np
import cv2
import json
import base64


class VehicleActuatorClient(object):
    def __init__(self, ip, port):
        if isinstance(ip, str):
            self.ip = bytes(ip, "utf-8")
        else:
            self.ip = ip
        self.port = port
        self.host = None
        self.event = None
        self.acceleration = 0
        self.steering = 0
        self.is_terminated = False

    def init_client(self):
        self.host = enet.Host(enet.Address(self.ip, self.port), 32, 1, 0, 0)
        #self.event = self.host.service(1000)
        threading.Thread(target=self.do_service, daemon=True).start()
        #threading.Thread(target=self.event_handler, daemon=True).start()

    def do_service(self):
        while not self.is_terminated:
            try:
                self.event = self.host.service(10)
                self.event_handler_once()
            except OSError:
                pass
            time.sleep(0.001)

    def event_handler(self):
        while not self.is_terminated:
            if self.event.type == enet.EVENT_TYPE_CONNECT:
                print("%s: Connect VehicleActuatorClient\n" % self.event.peer.address)

            elif self.event.type == enet.EVENT_TYPE_DISCONNECT:
                print("%s: Disconnect VehicleActuatorClient\n" % self.event.peer.address)
                self.is_terminated = True

            elif self.event.type == enet.EVENT_TYPE_RECEIVE:
                self.decode()

            else:
                self.acceleration = 0
                self.steering = 0

            self.host.flush()
            time.sleep(0.001)

    def decode(self):
        msg = self.event.packet.data.decode("utf-8")
        self.acceleration = float((msg.split('\"acc\":')[1]).split(',')[0])
        self.steering = float((msg.split('\"delta\":')[1]).split('}')[0])

    def do_service_once(self):
        try:
            self.event = self.host.service(10)
        except OSError:
            pass

    def event_handler_once(self):
        if self.event.type == enet.EVENT_TYPE_CONNECT:
            print("%s: VehicleActuatorClient CONNECT" % self.event.peer.address)

        elif self.event.type == enet.EVENT_TYPE_DISCONNECT:
            print("%s: VehicleActuatorClient DISCONNECT" % self.event.peer.address)

        elif self.event.type == enet.EVENT_TYPE_RECEIVE:
            self.decode()

        self.host.flush()


def tu_VehicleActuatorClient(_argv):
    ip = "127.0.0.1"
    port = 2003
    vehicle_actuator = VehicleActuatorClient(ip=ip, port=port)
    vehicle_actuator.init_client()

    while True:
        time.sleep(0.002)
        print(vehicle_actuator.acceleration, vehicle_actuator.steering)

if __name__ == '__main__':
    try:
        app.run(tu_VehicleActuatorClient)
    except SystemExit:
        pass
