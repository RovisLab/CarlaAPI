import os
import sys
import glob
import time
import enet
import threading

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


class ActuatorControl(BaseSensor):
    def __init__(self, name, parent_actor, client_args):
        super().__init__(name, parent_actor, client_args)
        self.send = True  # Actuator always starts a connection

        self.throttle = 0
        self.steering = 0
        self.recv_timeout = 0.3  # If no data received in this time, return 0
        self.ts_decode = 0  # Records the timestamp data has been received

    # ==================== General sensor methods ====================
    def setup(self):
        pass

    def get_data(self, optional_data_type=''):
        if (time.time() - self.ts_decode) > self.recv_timeout:
            return 0.0, 0.0
        return self.throttle, self.steering

    @staticmethod
    def sensor_callback(weak_ref, data):
        pass

    # ==================== Viewer methods ====================
    def do_view(self):
        while not self.stop_view:
            throttle, steering = self.get_data()
            print(' - {}/{} data: {}, {}'.format(self.parent_name, self.name, throttle, steering))
            time.sleep(self.viewer_delay / 1000)

    # ==================== Server methods ====================
    def init_server(self):
        self.host = enet.Host(enet.Address(bytes(conf.General['rovis_ip'], "utf-8"), self.port), 32, 1, 0, 0)
        # self.event = self.host.service(1000)

        self.service_thread = threading.Thread(target=self.do_service, daemon=True)
        self.service_thread.start()

        self.terminate_server = False

    def do_service(self):
        while not self.terminate_server:
            try:
                self.event = self.host.service(10)

                if self.event.type == enet.EVENT_TYPE_CONNECT:
                    print(' - {}:{} connected to {}.'.format(self.parent_name, self.name, self.event.peer.address))

                elif self.event.type == enet.EVENT_TYPE_DISCONNECT:
                    print(' - {}:{} disconnected from {}.'.format(self.parent_name, self.name, self.event.peer.address))
                    self.throttle = 0
                    self.terminate_server = True

                elif self.event.type == enet.EVENT_TYPE_RECEIVE:
                    self.decode_data_transfer()

                self.host.flush()
            except OSError:
                pass
            time.sleep(0.001)
        self.host = None

    def send_data(self):
        pass

    def decode_data_transfer(self):
        msg = self.event.packet.data.decode("utf-8")
        self.throttle = float((msg.split('\"acc\":')[1]).split(',')[0])
        self.steering = float((msg.split('\"delta\":')[1]).split('}')[0])
        self.ts_decode = time.time()

    def encode_data_transfer(self):
        pass

    # ==================== Pack data for saving ====================
    def pack_data(self, ts_stop, frame_id):
        pass


if __name__ == '__main__':
    print('NOT runnable. Check \'Run_Carla.py\'')
