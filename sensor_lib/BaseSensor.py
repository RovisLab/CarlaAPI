import sys
import glob
import time
import enet
import threading
import os

import config_utils as conf
from sensor_lib.sensor_utils import *
from src.CustomSignal import CustomSignal

try:
    sys.path.append(glob.glob('%s/PythonAPI/carla/dist/carla-*%d.%d-%s.egg' % (
        conf.CARLA_PATH,
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass
import carla


# Virtual class for sensors
class BaseSensor(object):
    data_sent_signal = CustomSignal()
    terminate_signal = CustomSignal()

    def __init__(self, name, parent_name, parent_actor):
        self.name = name  # Sensor name
        self.parent_name = parent_name  # Client name
        self._parent = parent_actor  # Parent actor
        self.args = conf.Clients[parent_name].Comm[name]  # Sensor conf
        self.width = conf.Clients[parent_name]['cam_width']
        self.height = conf.Clients[parent_name]['cam_height']
        self.fov = conf.Clients[parent_name]['cam_fov']
        self.sensor_transform = get_transform('default')  # Sensor pos in vehicle scope
        self.calib = []  # Calib matrix
        self.ch = 3  # Channels

        # Data vars
        self.sensor = None  # Carla sensor
        self.data = None  # Raw data

        # Capture vars
        self.has_capture = False  # Toggle capture (limits data capturing)
        self.capture = True  # Capture flag

        # Viewer vars
        self.viewer_delay = 1  # Delay between image changes [ms]
        self.view = False  # Toggle viewer
        self.viewer_thread = threading.Thread()
        self.stop_view = False  # Stop viewer

        # Server vars
        self.port = -1  # Server port
        self.send = False  # Toggle server connection
        self.send_bool = False  # Toggle send data
        self.host = None  # Enet host
        self.event = None  # Enet event
        self.service_thread = threading.Thread()
        self.sending_thread = threading.Thread()
        self.connected = False  # True if connected
        self.terminate_server = False  # Bool for server termination

    # ==================== General sensor methods ====================
    # Argument parser for sensor
    def parse_args(self):
        for key in self.args.keys():
            if key in ['type']:
                continue
            elif key == 'width':
                self.width = self.args[key]
            elif key == 'height':
                self.height = self.args[key]
            elif key == 'fov':
                self.fov = self.args[key]
            elif key == 'port':
                self.port = self.args[key]
            elif key == 'send':
                self.send = self.args[key]
            elif key == 'pos':
                self.sensor_transform = get_transform(self.args[key])
            elif key == 'view':
                self.view = self.args[key]
            elif key == 'view_dt':
                self.viewer_delay = self.args[key]

    # Setup sensor
    def setup(self):
        raise NotImplementedError

    # Preprocess and get data
    def get_data(self, optional_data_type=''):
        raise NotImplementedError

    @staticmethod
    # Sensor callback for carla
    def sensor_callback(weak_ref, data):
        raise NotImplementedError

    # Terminate sensor
    def terminate(self):
        # Stop viewer
        self.stop_view = True

        # Stop Server
        self.terminate_server = True

        # Stop sensor
        if self.sensor is not None:
            try:
                self.sensor.destroy()
            except RuntimeError:  # Sensor already destroyed
                pass

        # Check threads
        time.sleep(self.viewer_delay / 1000 + 0.1)
        if self.view and self.viewer_thread.is_alive():
            print(' - {}:{} - Viewer alive!'.format(self.parent_name, self.name))
        if self.send:
            if self.service_thread.is_alive():
                print(' - {}:{} - Server service alive!'.format(self.parent_name, self.name))
            if self.sending_thread.is_alive():
                print(' - {}:{} - Server sending alive!'.format(self.parent_name, self.name))

    # ==================== Viewer methods ====================
    # Start viewer
    def init_view(self):
        if self.view:
            self.viewer_thread = threading.Thread(target=self.do_view)
            self.viewer_thread.start()
            # print('viewer thread: {}'.format(self.viewer_thread.name))

    # Viewer
    def do_view(self):
        if self.view:
            raise NotImplementedError

    # ==================== Server methods ====================
    def init_server(self):
        if self.send:
            self.host = enet.Host(enet.Address(bytes(conf.General['rovis_ip'], "utf-8"), self.port), 10, 0, 0, 0)
            self.event = self.host.service(1000)

            self.service_thread = threading.Thread(target=self.do_service)
            self.sending_thread = threading.Thread(target=self.do_send)

            self.service_thread.start()
            self.sending_thread.start()

            self.terminate_server = False

    # Service for server
    def do_service(self):
        while not self.terminate_server:
            self.event = self.host.service(10)

            if self.event.type == enet.EVENT_TYPE_CONNECT:
                print(' - {}:{} connected to {}.'.format(self.parent_name, self.name, self.event.peer.address))
                self.connected = True

            elif self.event.type == enet.EVENT_TYPE_DISCONNECT:
                # print(' - {}:{} disconnected from {}.'.format(self.parent_name, self.name, self.event.peer.address))
                self.connected = False
            self.host.flush()
            time.sleep(0.01)
        self.host = None

    # Controls when the data is sent (can be signaled from above or implemented a timer)
    def send_data(self):
        self.send_bool = True

    # Check server state. Returns True if server is alright
    def check_server(self):
        return self.connected

    # Encode data for rovis transfer
    def encode_data_transfer(self):
        if self.send:
            raise NotImplementedError

    # Send data to rovis
    def do_send(self):
        while not self.terminate_server:
            # Pause sending data if not connected
            if not self.connected:
                time.sleep(0.1)
                continue

            encoded_data = self.encode_data_transfer()

            if encoded_data is not None:
                assert type(encoded_data) == bytes

                packet = enet.Packet(encoded_data, enet.PACKET_FLAG_RELIABLE)
                try:
                    self.host.broadcast(0, packet)
                except AttributeError:  # self.host = None
                    pass
                # self.data_sent_signal.emit()
                self.send_bool = False

            elif self.event.type == enet.EVENT_TYPE_DISCONNECT:
                self.connected = False
                continue

            # Wait for signal
            while not self.send_bool:
                time.sleep(0.01)
                if self.terminate_server:
                    break

    # ==================== Pack data for saving ====================
    def pack_data(self, ts_stop, frame_id):
        raise NotImplementedError

    # ==================== Others ====================
    # Generate calibration matrix
    def generate_calib_mat(self):
        self.calib = get_intrinsics(self.width, self.height, self.fov)

    # Generate calibration dict for calibration.cal file
    def generate_calib_dict(self):
        self.generate_calib_mat()

        # Explanations found in RovisDatabaseFormat.py - create_calib_file
        cal_dict = {
            'width': self.width, 'height': self.height,

            'rx': self.sensor_transform.rotation.yaw,  # yaw
            'ry': -self.sensor_transform.rotation.pitch + 90,  # -pitch + 90
            'rz': self.sensor_transform.rotation.roll - 90,  # roll - 90
            'tx': self.sensor_transform.location.x,
            'ty': self.sensor_transform.location.y,
            'tz': self.sensor_transform.location.z,

            'ch': self.channels,
            'fx': self.calib[0][0], 'fy': self.calib[1][1],
            'cx': self.calib[0][2], 'cy': self.calib[1][2],
            'px': '4.5000001591688488e-006', 'py': '4.5000001591688488e-006',
            'dist0': 0.0, 'dist1': 0.0, 'dist2': 0.0, 'dist3': 0.0, 'dist4': 0.0,
            'bline': 0.0,
        }

        return cal_dict


if __name__ == '__main__':
    print('NOT runnable. Check \'Run_Carla.py\'')
