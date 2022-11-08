import os
import sys
import glob
import threading
import subprocess
import time
from CarlaClient import CarlaClient
from CarlaClientPygame import CarlaClientPygame

CARLA_EXE_PATH = "C:/dev/tools/CARLA_0.9.12/WindowsNoEditor/CarlaUE4.exe"

# Importing Carla
try:
    sys.path.append(glob.glob('carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass
import carla


def start_carla(carla_exe):
    if not CarlaClient.check_carla_simulator_running_state():
        print(" # Carla simulator starting...")
        subprocess.Popen([carla_exe, "-windowed", "-ResX=640", "-ResY=480", "-fps=30", "-quality-level=Medium"],
                         start_new_session=True)
        while not CarlaClient.check_carla_simulator_running_state():
            time.sleep(0.001)
        print(" # Carla started. Please run the script again.")
        exit()
    else:
        print(" # Carla simulator ready.\n")


if __name__ == "__main__":
    start_carla(carla_exe=CARLA_EXE_PATH)

    if not CarlaClient.check_carla_simulator_running_state():
        print(' # Carla simulator could not be reached. Exiting..')
        exit()

    client = CarlaClient(name='Client',  # Name of the client
                         view_cam=True,  # View car from eagle view
                         position=carla.Location(-53, 115, 1),  # Initial position
                         orientation=carla.Rotation(0, 90, 0),  # Initial rotation
                         random_spawn=True,  # Get a random spawn point or spawn at the position above
                         vehicle_type='vehicle.Seat.Leon',  # Vehicle type
                         control='auto',  # auto / rovis
                         # town_name='Town03',  # Change the map if needed

                         ip_carla='127.0.0.1',  # Loopback ip for Carla
                         port_carla=2000,       # Carla port

                         ip_rovis='127.0.0.1',
                         port_rovis_actuator=None,            # 2003
                         port_rovis_camera_front=None,        # 2004
                         port_rovis_camera_back=None,         # 2005
                         port_rovis_semseg_camera=None,       # 2006
                         port_rovis_state_measurement=None,   # 2010
                         port_rovis_imu=None,                 # 2011
                         port_rovis_depth=None,               # 2020
                         port_rovis_lidar=None,               # 2021
                         port_rovis_radar=None,               # 2022

                         target_fps=30,
                         view_width=640,
                         view_height=480,
                         view_fov=90)

    # client_2 = CarlaClient(name='Client Two',
    #                        view_cam=True,
    #                        position=carla.Location(-73, 103, 1),
    #                        orientation=carla.Rotation(0, -89, 0),
    #                        random_spawn=False,
    #                        vehicle_type='vehicle.tesla.model3',
    #                        control='auto',
    #
    #                        ip_carla='127.0.0.1',
    #                        port_carla=2000,
    #
    #                        ip_rovis='127.0.0.1',
    #                        port_rovis_actuator=None,            # 2103
    #                        port_rovis_camera_front=2104,        # 2104
    #                        port_rovis_camera_back=2105,         # 2105
    #                        port_rovis_semseg_camera=None,       # 2106
    #                        port_rovis_state_measurement=2110,   # 2110
    #                        port_rovis_imu=None,                 # 2111
    #                        port_rovis_depth=None,               # 2120
    #                        port_rovis_lidar=None,               # 2121
    #                        port_rovis_radar=None,               # 2122
    #
    #                        target_fps=30,
    #                        view_width=640,
    #                        view_height=480,
    #                        view_fov=90)

    try:
        threading.Thread(target=client.game_loop).start()
        # threading.Thread(target=client_2.game_loop).start()
    except RuntimeError as e:
        print("Runtime error encountered.")
        print(e)
    except MemoryError as e:
        print("Memory error encountered.")
        print(e)
