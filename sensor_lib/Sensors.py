import sys
import glob
import time
import os

import config_utils as conf
from sensor_lib.sensors import *
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


class Sensors(object):
    all_data_is_collected_signal = CustomSignal()
    terminate_signal = CustomSignal()

    def __init__(self, parent_name, parent, world):
        self._parent_name = parent_name
        self._parent = parent
        self._world = world
        self.args = conf.Clients[parent_name]  # Client args
        self.sensors = {}

        # Prepare sensors to save
        self.sensors_to_save = {}
        for ds in conf.Database['Datastreams']:
            if ds['cl'] == self._parent_name:
                self.sensors_to_save[ds['sen']] = False

    # Add a sensor to Sensors
    def add(self, sensor_name):
        if sensor_name in self.sensors.keys():
            print(' # A sensor with this name already exists: {}'.format(sensor_name))
            return

        # Create sensor
        self.sensors[sensor_name] = None
        sensor_args = self.args.Comm[sensor_name]
        if sensor_args.type in sensor_types.keys():
            self.sensors[sensor_name] = sensor_types[sensor_args.type](sensor_name, self._parent_name, self._parent)
        else:
            return

        # Parse sensor args
        self.sensors[sensor_name].parse_args()

        # Link terminate signal from the sensor
        self.sensors[sensor_name].terminate_signal.connect(self.on_terminate)

    # Get a sensor
    def get(self, sensor_name):
        return self.sensors[sensor_name]

    # Get data from a sensor
    def get_data(self, sensor_name, optional_data_type=''):
        return self.sensors[sensor_name].get_data(optional_data_type)

    # Get sensor names / keys
    def keys(self):
        return self.sensors.keys()

    # Setup sensors
    def setup(self):
        # Setup sensors
        for sensor_key in self.sensors:
            self.sensors[sensor_key].setup()

    # Start sensors (viewer, server)
    def start(self):
        # Start view
        for sensor_key in self.sensors:
            if self.sensors[sensor_key].view:
                self.sensors[sensor_key].init_view()

        # Start Servers
        for sensor_key in self.sensors:
            if self.sensors[sensor_key].send:
                self.sensors[sensor_key].init_server()

    # Update capture flag for sensors
    def capture(self):
        for sensor_key in self.sensors:
            if self.sensors[sensor_key].has_capture:
                self.sensors[sensor_key].capture = True

    # Update send data flag for sensors
    def send_data(self):
        for sensor_key in self.sensors:
            if self.sensors[sensor_key].send and self.sensors[sensor_key].check_server():
                self.sensors[sensor_key].send_data()

    # Call this to collect and pack data from sensors. Emits "all_data_is_collected_signal" when done for CarlaClient.
    def get_data_to_save(self, ts_stop, frame_id):
        data_ready = {'name_in_sensors': self._parent_name}
        for name in self.sensors_to_save.keys():
            data_ready[name] = self.sensors[name].pack_data(ts_stop, frame_id)

        self.all_data_is_collected_signal.emit(self._parent_name, data_ready)

    def on_terminate(self):
        self.terminate_signal.emit()

    # Terminate all sensors
    def terminate(self):
        for sensor_key in self.sensors.keys():
            self.sensors[sensor_key].terminate()
        time.sleep(0.001)


if __name__ == '__main__':
    print('NOT runnable. Check \'Run_Carla.py\'')
