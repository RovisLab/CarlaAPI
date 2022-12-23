import glob
import os
import sys
import weakref
import math
import numpy as np
import cv2
import collections

try:
    sys.path.append(glob.glob('dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
    # sys.path.append(glob.glob('carla-0.9.10-py3.7-win-amd64.egg'))
except IndexError:
    pass

import carla


'''
Sensor positions
'''
def get_transform(sensor_type):
    # Cameras
    if sensor_type in ['front', 'f']:
        return carla.Transform(carla.Location(x=1.8, z=1.2), carla.Rotation())
    elif sensor_type in ['back', 'rear', 'r', 'b']:
        return carla.Transform(carla.Location(x=-2.1, z=1.3), carla.Rotation(yaw=180))
    elif sensor_type in ['front_left']:
        return carla.Transform(carla.Location(x=1, y=-0.6, z=1.2), carla.Rotation(yaw=-60))
    elif sensor_type in ['front_right']:
        return carla.Transform(carla.Location(x=1, y=0.6,  z=1.2), carla.Rotation(yaw=60))
    elif sensor_type in ['back_left']:
        return carla.Transform(carla.Location(x=-1.8, y=-0.6, z=1.3), carla.Rotation(yaw=-120))
    elif sensor_type in ['back_right']:
        return carla.Transform(carla.Location(x=-1.8, y=0.6, z=1.3), carla.Rotation(yaw=120))
    elif sensor_type in ['eagle', 'bird', 'e']:
        return carla.Transform(carla.Location(z=30), carla.Rotation(pitch=-90))
    elif sensor_type in ['wheel', 'w']:
        return carla.Transform(carla.Location(x=-0.8, y=-1.1, z=0.5), carla.Rotation())
    elif sensor_type in ['driver', 'd']:
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

    elif sensor_type in ['empty', 'null']:
        return carla.Transform()

    # else return default
    return carla.Transform(carla.Location(x=-5.5, z=2.8), carla.Rotation(pitch=-15))


'''
Ultrasonic sensor implementation
'''
# class UltrasonicSensor(object):
#     def __init__(self, parent_actor):
#         self.sensor = None
#         self._parent = parent_actor
#         world = self._parent.get_world()
#         self.ultrasonics = None
#
#         us_bp = world.get_blueprint_library().find('sensor.other.ultrasonic_sensor')
#         us_bp.set_attribute('num_us_front', str(NUM_US_FRONT))
#         us_bp.set_attribute('num_us_back', str(NUM_US_BACK))
#         us_bp.set_attribute('front_us_fov', str(FRONT_US_FOV))
#         us_bp.set_attribute('back_us_fov', str(BACK_US_FOV))
#         us_bp.set_attribute('max_range', str(US_MAX_RANGE))
#
#         sensor_transform = carla.Transform(carla.Location(x=0.0, z=0.5), carla.Rotation(pitch=0))
#         self.sensor = world.spawn_actor(
#             us_bp, sensor_transform, attach_to=self._parent
#         )
#         weak_self = weakref.ref(self)
#         self.sensor.listen(
#             lambda x: UltrasonicSensor._us_callback(weak_self, x)
#         )
#
#     def get_front_sensors(self):
#         front_us = list()
#         for idx in range(0, NUM_US_FRONT):
#             front_us.append(self.ultrasonics[idx])
#         return [x.range for x in front_us]
#
#     def get_back_sensors(self):
#         back_us = list()
#         for idx in range(NUM_US_FRONT, len(self.ultrasonics)):
#             back_us.append(self.ultrasonics[idx])
#         back_us = [x.range for x in back_us]
#         back_us.reverse()
#         return back_us
#
#     @staticmethod
#     def _us_callback(weak_ref, us_measurement):
#         self = weak_ref()
#         self.ultrasonics = us_measurement


class MultiCameraSensors(object):
    def __init__(self):
        self.sensors = list()
        self.images = list()
        self.capture = list()

    def add_sensor(self, parent):
        world = parent.get_world()
        camera_bp = world.get_blueprint_library().find('sensor.camera.rgb')
        camera_bp.set_attribute('image_size_x', str(VIEW_WIDTH))
        camera_bp.set_attribute('image_size_y', str(VIEW_HEIGHT))
        camera_bp.set_attribute('fov', str(VIEW_FOV))
        camera_transform = carla.Transform(carla.Location(x=-5.5, z=2.8), carla.Rotation(pitch=-15))
        self.sensors.append(
            world.spawn_actor(
                camera_bp, camera_transform, attach_to=parent
            )
        )

    def add_first_image(self, image):
        self.images[0] = image
        self.capture[0] = False

    def add_second_image(self, image):
        self.images[1] = image
        self.capture[1] = False

    def init(self):
        self.images = [None, None]
        self.capture = [None, None]

        self.sensors[0].listen(
            lambda x: self.add_first_image(x)
        )

        self.sensors[1].listen(
            lambda x: self.add_second_image(x)
        )


'''
Camera sensor implementation
'''
class CollisionSensor(object):
    def __init__(self, parent_actor):
        self.sensor = None
        self.history = []
        self._parent = parent_actor
        world = self._parent.get_world()
        bp = world.get_blueprint_library().find('sensor.other.collision')
        self.sensor = world.spawn_actor(bp, carla.Transform(), attach_to=self._parent)
        # We need to pass the lambda a weak reference to self to avoid circular
        # reference.
        weak_self = weakref.ref(self)
        self.sensor.listen(lambda event: CollisionSensor._on_collision(weak_self, event))

    def get_collision_history(self):
        history = collections.defaultdict(int)
        for frame, intensity in self.history:
            history[frame] += intensity
        return history

    @staticmethod
    def _on_collision(weak_self, event):
        self = weak_self()
        if not self:
            return
        impulse = event.normal_impulse
        intensity = math.sqrt(impulse.x**2 + impulse.y**2 + impulse.z**2)
        self.history.append((event.frame, intensity))
        if len(self.history) > 4000:
            self.history.pop(0)
