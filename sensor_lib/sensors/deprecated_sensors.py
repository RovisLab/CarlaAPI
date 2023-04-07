"""
    DEPRECATED SENSORS
"""

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


'''
Multicamera sensor implementation
'''
# class MultiCameraSensors(object):
#     def __init__(self):
#         self.sensors = list()
#         self.images = list()
#         self.capture = list()
# 
#     def add_sensor(self, parent):
#         world = parent.get_world()
#         camera_bp = world.get_blueprint_library().find('sensor.camera.rgb')
#         camera_bp.set_attribute('image_size_x', str(VIEW_WIDTH))
#         camera_bp.set_attribute('image_size_y', str(VIEW_HEIGHT))
#         camera_bp.set_attribute('fov', str(VIEW_FOV))
#         camera_transform = carla.Transform(carla.Location(x=-5.5, z=2.8), carla.Rotation(pitch=-15))
#         self.sensors.append(
#             world.spawn_actor(
#                 camera_bp, camera_transform, attach_to=parent
#             )
#         )
# 
#     def add_first_image(self, image):
#         self.images[0] = image
#         self.capture[0] = False
# 
#     def add_second_image(self, image):
#         self.images[1] = image
#         self.capture[1] = False
# 
#     def init(self):
#         self.images = [None, None]
#         self.capture = [None, None]
# 
#         self.sensors[0].listen(
#             lambda x: self.add_first_image(x)
#         )
# 
#         self.sensors[1].listen(
#             lambda x: self.add_second_image(x)
#         )


'''
Collision sensor implementation
'''
# class CollisionSensor(object):
#     def __init__(self, parent_actor):
#         self.sensor = None
#         self.history = []
#         self._parent = parent_actor
#         world = self._parent.get_world()
#         bp = world.get_blueprint_library().find('sensor.other.collision')
#         self.sensor = world.spawn_actor(bp, carla.Transform(), attach_to=self._parent)
#         # We need to pass the lambda a weak reference to self to avoid circular
#         # reference.
#         weak_self = weakref.ref(self)
#         self.sensor.listen(lambda event: CollisionSensor._on_collision(weak_self, event))
# 
#     def get_collision_history(self):
#         history = collections.defaultdict(int)
#         for frame, intensity in self.history:
#             history[frame] += intensity
#         return history
# 
#     @staticmethod
#     def _on_collision(weak_self, event):
#         self = weak_self()
#         if not self:
#             return
#         impulse = event.normal_impulse
#         intensity = math.sqrt(impulse.x**2 + impulse.y**2 + impulse.z**2)
#         self.history.append((event.frame, intensity))
#         if len(self.history) > 4000:
#             self.history.pop(0)
