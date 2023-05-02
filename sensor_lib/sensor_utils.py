import sys
import glob
import threading
import numpy as np
import os

import config_utils as conf

try:
    sys.path.append(glob.glob('%s/PythonAPI/carla/dist/carla-*%d.%d-%s.egg' % (
        conf.CARLA_PATH,
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass
import carla


def get_transform(sensor_type):
    # Cameras
    if sensor_type in ['front']:
        return carla.Transform(carla.Location(x=1.8, z=1.2), carla.Rotation())
    elif sensor_type in ['back', 'rear']:
        return carla.Transform(carla.Location(x=-2.1, z=1.3), carla.Rotation(yaw=180))
    elif sensor_type in ['front_left']:
        return carla.Transform(carla.Location(x=1, y=-0.6, z=1.2), carla.Rotation(yaw=-60))
    elif sensor_type in ['front_right']:
        return carla.Transform(carla.Location(x=1, y=0.6, z=1.2), carla.Rotation(yaw=60))
    elif sensor_type in ['back_left']:
        return carla.Transform(carla.Location(x=-1.8, y=-0.6, z=1.3), carla.Rotation(yaw=-120))
    elif sensor_type in ['back_right']:
        return carla.Transform(carla.Location(x=-1.8, y=0.6, z=1.3), carla.Rotation(yaw=120))
    elif sensor_type in ['eagle', 'bird']:
        return carla.Transform(carla.Location(z=30), carla.Rotation(pitch=-90))
    elif sensor_type in ['wheel']:
        return carla.Transform(carla.Location(x=-0.8, y=-1.1, z=0.5), carla.Rotation())
    elif sensor_type in ['driver']:
        return carla.Transform(carla.Location(x=0.7, y=-0.38, z=1.25), carla.Rotation())
    elif sensor_type in ['above']:
        return carla.Transform(carla.Location(z=7.5), carla.Rotation(pitch=-90))
    elif sensor_type in ['default']:
        return carla.Transform(carla.Location(x=-5.5, z=2.8), carla.Rotation(pitch=-15))

    # Others
    elif sensor_type in ['depth']:
        return carla.Transform(carla.Location(x=1.5, z=2.4))
    elif sensor_type in ['lidar']:
        return carla.Transform(carla.Location(z=2), carla.Rotation())
    elif sensor_type in ['radar']:
        return carla.Transform(carla.Location(x=2.5, z=1.0), carla.Rotation(pitch=5))

    elif sensor_type in ['empty', 'null', '']:
        return carla.Transform()

    # else return default
    return carla.Transform(carla.Location(x=-5.5, z=2.8), carla.Rotation(pitch=-15))


def get_intrinsics(width, height, fov):
    """
    Sources:
        https://github.com/carla-simulator/carla/issues/56
        https://github.com/carla-simulator/carla/blob/master/PythonAPI/examples/lidar_to_camera.py - line 113
    """
    f_length = width / (2.0 * np.tan(fov * np.pi / 360.0))  # Focal Length
    cx = width / 2  # Center X
    cy = height / 2  # Center Y

    mat = [[f_length, 0,        cx],
           [0,        f_length, cy],
           [0,        0,        1]]

    return mat


CARLA_OBJECT_TYPES = {  # label - CarlaLabel, cls - sem_cls, tag - for actor filtering
    'pedestrians': {'label': carla.CityObjectLabel.Pedestrians, 'cls': 4, 'tag': 'walker.*'},
    'vehicles': {'label': carla.CityObjectLabel.Vehicles, 'cls': 10, 'tag': 'vehicle.*'},
    'signs': {'label': carla.CityObjectLabel.TrafficSigns, 'cls': 12, 'tag': 'vehicle.*'},
    'lights': {'label': carla.CityObjectLabel.TrafficLight, 'cls': 18, 'tag': 'vehicle.*'},
}


if __name__ == '__main__':
    print('NOT runnable. Check \'Run_Carla.py\'')
