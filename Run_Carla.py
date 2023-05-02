import os
import sys
import glob
import threading
import subprocess
import psutil
import time

import config_utils as conf
from src.CarlaMain import CarlaMain


try:
    sys.path.append(glob.glob('%s/PythonAPI/carla/dist/carla-*%d.%d-%s.egg' % (
        conf.CARLA_PATH,
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass
import carla

__all__ = [
    'check_carla_simulator_running_state'
]


def check_carla_simulator_running_state():
    for p in psutil.process_iter():
        if "carla" in p.name().lower():
            return True
    return False


def start_carla():
    carla_exe = '{}/CarlaUE4.exe'.format(conf.CARLA_PATH)
    res_x = conf.General['res_x']
    res_y = conf.General['res_y']
    fps = conf.General['fps']
    quality = conf.General['quality']

    print('')
    if not check_carla_simulator_running_state():
        print(" # Carla simulator starting...")
        try:
            subprocess.Popen([carla_exe, "-windowed", f"-ResX={res_x}", f"-ResY={res_y}",
                              f"-fps={fps}", f"-quality-level={quality}"],
                             start_new_session=True)
        except FileNotFoundError:
            print(" # Carla executable path is invalid.")
        while not check_carla_simulator_running_state():
            time.sleep(0.001)
        print(" # Carla started. Please run again after the simulation appears.")
        exit()
    else:
        print(" # Carla simulator ready.")


def load_town():
    town_name = conf.General['town']
    # print('Loading the {} map.'.format(town_name))
    client = carla.Client(conf.General['carla_ip'], conf.General['carla_port'])
    if town_name in [elem.split('/')[-1] for elem in client.get_available_maps()]:
        client.set_timeout(999)
        client.load_world(town_name)
    else:
        print(' # The map {} is invalid or not existing. Exiting...'.format(town_name))
        print(' - Available maps: {}.'.format(client.get_available_maps()))
        exit()
    print(' # Map {} loaded.'.format(town_name))


def main():
    # Start Carla
    start_carla()

    # Load town
    load_town()

    # Check carla
    if not check_carla_simulator_running_state():
        print(' # Carla simulator could not be reached. Exiting..')
        exit(-3)

    # Create CarlaMain instance
    carla_main = CarlaMain()

    # Setup CarlaMain
    carla_main.setup()

    # Start CarlaMain
    carla_main.start()


if __name__ == "__main__":
    config_path = r"configs/Save_data_Iulia.conf"

    # Configuration setup
    if not conf.config_setup(config_path):
        print(' # Problem with the configuration setup.')
        exit(-2)

    # Run main
    main()
