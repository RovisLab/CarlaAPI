import os
import sys
import glob
import threading
import subprocess
import psutil
import time
from CarlaClient import CarlaClient, port_carla, ip_carla
from CarlaClientPygame import CarlaClientPygame

CARLA_EXE_PATH = "C:/dev/tools/CARLA_0.9.13/WindowsNoEditor/CarlaUE4.exe"

# Importing Carla
try:
    sys.path.append(glob.glob('dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass
import carla


def check_carla_simulator_running_state():
    for p in psutil.process_iter():
        if "carla" in p.name().lower():
            return True
    return False


def start_carla(carla_exe):
    if not check_carla_simulator_running_state():
        print(" # Carla simulator starting...")
        try:
            subprocess.Popen([carla_exe, "-windowed", "-ResX=640", "-ResY=480", "-fps=30", "-quality-level=Medium"],
                             start_new_session=True)
        except FileNotFoundError:
            print(" # Carla executable path is invalid.")
        while not check_carla_simulator_running_state():
            time.sleep(0.001)
        print(" # Carla started. Please run the script again.")
        exit()
    else:
        print(" # Carla simulator ready.\n")


def load_map(town_name):
    print('Loading the {} map.'.format(town_name))
    client = carla.Client(ip_carla, port_carla)
    if town_name in [elem.split('/')[-1] for elem in client.get_available_maps()]:
        client.set_timeout(999)
        client.load_world(town_name)
    else:
        print('The map {} is invalid or not existing. Exiting...'.format(town_name))
    print('Map loaded')


if __name__ == "__main__":
    start_carla(carla_exe=CARLA_EXE_PATH)

    # Load map
    load_map('Town03')

    if not check_carla_simulator_running_state():
        print(' # Carla simulator could not be reached. Exiting..')
        exit()

    car_01 = CarlaClient(name='Car One',  # Name of the client
                         view_cam=True,  # View car from eagle view
                         position=carla.Location(-77.3, 54.4, 1),
                         orientation=carla.Rotation(0, -90, 0),
                         random_spawn=False,  # Get a random spawn point or spawn at the position above
                         vehicle_type='vehicle.Seat.Leon',  # Vehicle type
                         control='rovis',  # auto / rovis
                         ip_rovis='127.0.0.1',
                         port_rovis_actuator=2003,            # 2003
                         port_rovis_cam_front=2004,           # 2004
                         port_rovis_cam_back=None,            # 2005
                         port_rovis_cam_left=None,            # 2006
                         port_rovis_cam_right=None,           # 2007
                         port_rovis_cam_back_left=None,       # 2008
                         port_rovis_cam_back_right=None,      # 2009
                         port_rovis_semseg_front=None,        # 2010
                         port_rovis_semseg_back=None,         # 2011
                         port_rovis_semseg_left=None,         # 2012
                         port_rovis_semseg_right=None,        # 2013
                         port_rovis_semseg_back_left=None,    # 2014
                         port_rovis_semseg_back_right=None,   # 2015
                         port_rovis_state_measurement=2016,   # 2016
                         port_rovis_imu=None,                 # 2017
                         port_rovis_depth=None,               # 2020
                         port_rovis_lidar=None,               # 2021
                         port_rovis_radar=None,               # 2022
                         target_fps=30,
                         view_width=640,
                         view_height=480,
                         view_fov=90)

    # car_02 = CarlaClient(name='Car Two',  # Name of the client
    #                      view_cam=True,  # View car from eagle view
    #                      position=carla.Location(-77.3, 74.4, 1),
    #                      orientation=carla.Rotation(0, -90, 0),
    #                      random_spawn=False,  # Get a random spawn point or spawn at the position above
    #                      vehicle_type='vehicle.bmw.grandtourer',  # Vehicle type
    #                      control='rovis',  # auto / rovis
    #                      ip_rovis='127.0.0.1',
    #                      port_rovis_actuator=2103,
    #                      port_rovis_cam_front=2104,
    #                      port_rovis_cam_back=None,
    #                      port_rovis_cam_left=None,
    #                      port_rovis_cam_right=None,
    #                      port_rovis_cam_back_left=None,
    #                      port_rovis_cam_back_right=None,
    #                      port_rovis_semseg_front=None,
    #                      port_rovis_semseg_back=None,
    #                      port_rovis_semseg_left=None,
    #                      port_rovis_semseg_right=None,
    #                      port_rovis_semseg_back_left=None,
    #                      port_rovis_semseg_back_right=None,
    #                      port_rovis_state_measurement=2116,
    #                      port_rovis_imu=None,
    #                      port_rovis_depth=None,
    #                      port_rovis_lidar=None,
    #                      port_rovis_radar=None,
    #                      target_fps=30,
    #                      view_width=640,
    #                      view_height=480,
    #                      view_fov=90)

    try:
        threading.Thread(target=car_01.game_loop).start()
        # threading.Thread(target=car_02.game_loop).start()
    except RuntimeError as e:
        print("Runtime error encountered.")
        print(e)
    except MemoryError as e:
        print("Memory error encountered.")
        print(e)
