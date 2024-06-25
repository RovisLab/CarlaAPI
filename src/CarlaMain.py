import os
import sys
import glob
import psutil
import time
import threading
import random
import cv2
import numpy as np
import time

import config_utils as conf
from src.CarlaClient import CarlaClient
from src.PopulateSim import PopulateSim
from depend.RovisDatabaseFormat import RovisDataBase, create_calib_file

try:
    sys.path.append(glob.glob('%s/PythonAPI/carla/dist/carla-*%d.%d-%s.egg' % (
        conf.CARLA_PATH,
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass
import carla


class CarlaMain(object):
    def __init__(self):
        self.clients = {}
        self.sim_client = carla.Client(conf.General['carla_ip'], conf.General['carla_port'])
        self.sim_client.set_timeout(2.0)
        self.world = self.sim_client.get_world()
        self.game_loop_thread = threading.Thread()
        self.terminate = False  # Termination flag

        # Database vars
        self.db = None
        self.dt = conf.Database['sampling_time']  # Delay between saves
        self.signal_save = False  # Bool that signals when to save data
        self.data_count = 0  # Data counter
        self.save_data_thread = threading.Thread()  # Signals when to save data

        # Populate Sim
        self.population = PopulateSim(self.sim_client)

        # Set sync mode
        self.set_synchronous_mode(conf.General['sync_mode'])

    def setup(self):
        for key in conf.Clients.keys():
            self.clients[key] = {
                'client': CarlaClient(name=key, world=self.world),
                'thread': threading.Thread(),
                'send': False,  # Bool that indicates if data is being sent
                'save': False,  # Bool that indicates if data is saved
                'data': None  # Data to be retrieved
            }

            # Link terminate signal from the CarlaClient
            self.clients[key]['client'].terminate_signal.connect(self.on_terminate)

            for ds in conf.Database.Datastreams:
                if ds['cl'] == key:
                    self.clients[key]['save'] = True
                    break

            if self.clients[key]['save']:
                self.clients[key]['client'].data_is_ready_signal.connect(self.on_data_ready)

    def start(self):
        # Start each individual client in its own thread
        for key in self.clients:
            self.clients[key]['thread'] = threading.Thread(target=self.clients[key]['client'].client_loop)
            self.clients[key]['thread'].start()
            # print(' Client {} - thread name: {}'.format(key, self.clients[key]['thread'].name))

        # Populate simulation
        if conf.General['populate_sim']:
            self.population.start()

        # Start game loop
        self.game_loop_thread = threading.Thread(target=self.game_loop)
        self.game_loop_thread.start()
        # print(' Game loop - thread name: {}'.format(self.game_loop_thread.name))

        # Init database and start the timer
        if conf.Database['save_data']:
            self.prepare_database()
            self.save_data_thread = threading.Thread(target=self.save_data_timer)
            self.save_data_thread.start()

    def game_loop(self):
        # Prepare timestamp
        ts_start = int(time.time_ns()/1000)
        ts_stop = ts_start
        frame_id = 0  # First frame

        while not self.terminate:
            # Tick world
            self.world.tick()

            # Tick clients
            for name in self.clients:
                self.clients[name]['client'].tick()

            # Save data if needed
            if conf.Database['save_data'] and self.signal_save:
                self.save_data_sequence(ts_start, ts_stop, frame_id)

                # Advance
                ts_start = ts_stop
                ts_stop += int(1000*self.dt)
                frame_id += 1

    def save_data_timer(self):
        time.sleep(5)
        print(" # Starting to save data.")
        while not self.terminate:
            if not self.signal_save:
                time.sleep(self.dt)
                self.signal_save = True
            time.sleep(0.01)

    def save_data_sequence(self, ts_start, ts_stop, frame_id):
        # Freeze actors if not sync
        if not conf.General['sync_mode']:
            next(iter(self.clients.values()))['client'].set_actors_physics(False)

        # Clean saved data
        self.clean_saved_data()

        # Trigger data capturing
        for name in self.clients.keys():
            self.clients[name]['client'].capture()
        time.sleep(0.01)

        # Tick world
        self.world.tick()

        # Check termination
        if self.terminate:
            return

        # Start saving
        get_data_threads = []
        for name in self.clients.keys():
            if self.clients[name]['save']:
                # self.clients[name]['client'].get_saving_data(ts_stop, frame_id)
                get_data_threads.append(threading.Thread(target=self.clients[name]['client'].get_saving_data,
                                                         args=(ts_stop, frame_id)))
                get_data_threads[-1].start()

        # Wait for data to be ready
        while not self.check_if_data_ready():
            time.sleep(0.01)
            if self.terminate:
                break

        # Check if threads are closed
        for th in get_data_threads:
            if th.is_alive():
                print(' - Thread in get_data_threads is alive.')

        # Check termination
        if self.terminate:
            return

        # Add data to database
        self.add_to_database(ts_start, ts_stop)

        # Unfreeze actors if not sync
        if not conf.General['sync_mode']:
            next(iter(self.clients.values()))['client'].set_actors_physics(True)

        # Count data
        self.data_count += 1

        # Exit if data done
        if conf.Database['samples'] != -1:
            if conf.Database['samples'] == self.data_count:
                threading.Thread(target=self.on_terminate, daemon=True).start()

        # Print progress
        if self.data_count % 5 == 0:
            print(' - Saved {} samples.'.format(self.data_count))

        # Mark end of saving data sequence
        self.signal_save = False

    def on_data_ready(self, name, data):
        self.clients[name]['data'] = data

    def check_if_data_ready(self):
        for name in self.clients.keys():
            if self.clients[name]['save'] and self.clients[name]['data'] is None:
                return False
        return True

    def clean_saved_data(self):
        # Reset data
        for name in self.clients.keys():
            self.clients[name]['data'] = None

    def prepare_database(self):
        # Init empty database
        self.db = RovisDataBase(db_path=conf.Database['database_path'], core_id=1)

        # Prepare obj_classes file
        obj_cls_path = '{}/include/carla_obj_classes.conf'.format(os.getcwd())

        # Add streams
        for ds_idx, ds in enumerate(conf.Database['Datastreams']):
            # {name = "<name>", cl = "<client>", sen = "<sensor_name>", input=<ds_id>}
            cl = ds['cl']
            sen = ds['sen']
            filter_type = conf.Clients[cl].Comm[sen].type

            if filter_type in ['depth', 'oflow']:
                filter_type = 'camera'

            # Add data stream
            if 'input' not in ds.keys():
                self.db.add_stream(filter_type=filter_type, filter_id=ds_idx + 1,
                                   filter_name=ds['name'])
            else:
                self.db.add_stream(filter_type=filter_type, filter_id=ds_idx + 1,
                                   filter_name=ds['name'], input_sources=ds['input'])

        # Create database
        self.db.create_db()

        # Add obj classes and calib files
        for ds_idx, ds in enumerate(conf.Database['Datastreams']):
            # {name = "<name>", cl = "<client>", sen = "<sensor_name>", input=<ds_id>}
            cl = ds['cl']
            sen = ds['sen']
            filter_type = conf.Clients[cl].Comm[sen].type

            if filter_type in ['depth', 'oflow']:
                filter_type = 'camera'

            # Add obj classes file
            if filter_type in ['semseg', 'det2d', 'det3d']:
                self.db.add_custom(filter_id=ds_idx + 1, data=obj_cls_path, name='object_classes.conf')

            # Add calibration file
            if filter_type in ['camera', 'lidar', 'radar']:
                calib_dict = self.clients[cl]['client'].sensors.get(sen).generate_calib_dict()
                if filter_type in ['lidar', 'radar']:
                    calib_file = create_calib_file('{} - {}'.format(cl, sen), calib_dict, exclude_intrinsic=True)
                else:
                    calib_file = create_calib_file('{} - {}'.format(cl, sen), calib_dict, exclude_intrinsic=False)
                self.db.add_custom(filter_id=ds_idx + 1, data=calib_file, name='calibration.cal')

    def add_to_database(self, ts_start, ts_stop):
        # All data is collected and ready to be saved
        data = {}

        for ds_idx, ds in enumerate(conf.Database['Datastreams']):
            data['datastream_{}'.format(ds_idx + 1)] = self.clients[ds['cl']]['data'][ds['sen']]

        # Add data to database
        self.db.add_data(ts_start=ts_start, ts_stop=ts_stop, data=data)

    # Set synchronous mode
    def set_synchronous_mode(self, synchronous_mode):
        settings = self.world.get_settings()
        if settings.synchronous_mode != synchronous_mode:
            settings.synchronous_mode = synchronous_mode
            if synchronous_mode:
                settings.fixed_delta_seconds = 1 / conf.General['fps']
        self.world.apply_settings(settings)

    # Set physics for all actors
    def set_actors_physics(self, value: bool):
        actor_list = self.world.get_actors()
        for actor in actor_list:
            actor.set_simulate_physics(value)

    # Print maps
    def print_available_maps(self):
        maps = [elem.split('/')[-1] for elem in self.sim_client.get_available_maps()]
        print(' # Available maps: {}.'.format(', '.join(maps)))

    # Terminate everything
    def on_terminate(self):
        # Termination flag
        self.terminate = True

        # Progress
        print('')
        print(' # Termination started..')
        if conf.Database['save_data']:
            if conf.Database['samples'] == -1:
                print(' - Saved {} samples.'.format(self.data_count))
            else:
                print(' - Saved {} out of {} samples.'.format(self.data_count, conf.Database['samples']))

        # Terminate population
        if conf.General['populate_sim']:
            self.population.terminate()

        # Terminate clients
        for name in self.clients.keys():
            self.clients[name]['client'].terminate()
            self.clients[name]['thread'].join(timeout=1)  # Wait for thread to close
            if self.clients[name]['thread'].is_alive():
                print(' # {} terminated but the thread is still alive.'.format(name))
            else:
                print(' # {} successfully terminated.'.format(name))

        # Wait for main threads
        if self.game_loop_thread.is_alive():
            self.game_loop_thread.join(timeout=1)
        if self.game_loop_thread.is_alive():
            print(' # Game loop thread is still alive...')

        # Kill simulator
        # for proc in psutil.process_iter():
        #     if proc.name() in ['CarlaUE4-Win64-Shipping.exe', 'CarlaUE4.exe']:
        #         proc.kill()

        # Exit
        exit(1)
