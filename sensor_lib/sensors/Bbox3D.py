import os
import sys
import glob
import time
import numpy as np
import math
import cv2
import weakref

import config_utils as conf
from sensor_lib.sensor_utils import CARLA_OBJECT_TYPES
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


class Bbox3DSensor(BaseSensor):
    def __init__(self, name, parent_actor, client_args):
        super().__init__(name, parent_actor, client_args)
        self.has_capture = True
        self.world = None
        self.bbox_handler = None
        self.range = -1  # Ignore range
        self.actor_filters = ("pedestrians", "vehicles")

        self.generate_calib_mat()

    # ==================== General sensor methods ====================
    def parse_args(self):
        super().parse_args()

        for key in self.args.keys():
            if key == 'range':
                self.range = self.args[key]
            if key == 'cls':
                self.actor_filters = self.args[key]

    def setup(self):
        self.world = self._parent.get_world()
        self.bbox_handler = BoundingBoxesHandler(self._parent, self, self.range)

    def get_data(self, optional_data_type='image'):
        data_types = ['rovis', 'image']
        # rovis - data packed for rovis (saving or sending)
        # image - data meant to be displayed from Carla
        if optional_data_type not in data_types:
            optional_data_type = 'image'

        if len(self.actor_filters) == 0:
            return []

        if optional_data_type == 'image':
            return self.bbox_handler.get_image_bboxes(self.actor_filters)
        elif optional_data_type == 'rovis':
            return self.bbox_handler.get_rovis_bboxes(self.actor_filters)

        return None

    @staticmethod
    def sensor_callback(weak_ref, data):
        pass

    # ==================== Viewer methods ====================
    def do_view(self):
        window_name = '{} - {}'.format(self.parent_name, self.name)
        background = np.zeros((self.height, self.width, 3), dtype=np.dtype('uint8'))
        while not self.stop_view:
            bboxes = self.get_data('image')
            if len(bboxes) == 0:
                time.sleep(0.001)
                continue

            image = self.bbox_handler.draw_bboxes(bboxes, background.copy())

            key = None
            if image is not None:
                cv2.imshow(window_name, image)
                key = cv2.waitKey(self.viewer_delay)

            if key is not None and key != -1:
                if key == 27:  # ESC - close everything
                    self.terminate_signal.emit()
                    self.stop_view = True
                elif key == ord('q'):  # Close just the viewer
                    self.stop_view = True

        time.sleep(0.001)
        if cv2.getWindowProperty(window_name, cv2.WND_PROP_VISIBLE) > 1:
            cv2.destroyWindow(window_name)

    # ==================== Server methods ====================
    def encode_data_transfer(self):
        # bboxes format: [{center, cls, size, rot}, ...]
        bboxes = self.get_data('rovis')

        if bboxes is not None:
            # TODO data = ...
            return bytes(str(bboxes), 'utf8')
        return None

    # ==================== Pack data for saving ====================
    def pack_data(self, ts_stop, frame_id):
        bboxes = self.get_data('rovis')
        # bboxes = [ {center, cls, size, rot}, ... ]

        roi_id, cls, x, y, z, w, h, l, roll, pitch, yaw = [], [], [], [], [], [], [], [], [], [], []
        for idx, bbox in enumerate(bboxes):
            roi_id.append(idx + 1)  # Roi id starts at 1
            cls.append(bbox['cls'])
            x.append(bbox['center']['x'])
            y.append(bbox['center']['y'])
            z.append(bbox['center']['z'])
            w.append(bbox['size']['w'])
            h.append(bbox['size']['h'])
            l.append(bbox['size']['l'])
            roll.append(bbox['rot']['roll'])
            pitch.append(bbox['rot']['pitch'])
            yaw.append(bbox['rot']['yaw'])

        packed_data = {
            'frame_id': frame_id,
            'roi_id': roi_id,
            'cls': cls,
            'x': x, 'y': y, 'z': z,
            'w': w, 'h': h, 'l': l,
            'roll': roll,
            'pitch': pitch,
            'yaw': yaw
        }
        return packed_data

    # ==================== Custom methods ====================
    def draw_bboxes(self, bboxes, image, thickness=1):
        if self.bbox_handler is None:
            return image
        return self.bbox_handler.draw_bboxes(bboxes, image, thickness)


# Class to handle the bounding boxes
class BoundingBoxesHandler(object):
    def __init__(self, parent, sensor, max_range):
        self.parent = parent  # Main vehicle
        self.sensor = sensor  # Bbox sensor
        self.world = self.sensor.world  # World
        self.transform = self.sensor.sensor_transform  # Sensor in relation to main vehicle
        self.calib = self.sensor.calib  # Camera calib
        self.range = max_range if max_range > 0 else np.inf  # Max range for bbox detection

        self.veh_sensor_mat = np.linalg.inv(self.get_matrix(self.transform))

    def get_image_bboxes(self, actor_filters):
        result_bboxes = []  # Saves all bboxes pts in sensor cords
        for actor_filter in actor_filters:
            carla_bbox_list = self.world.get_level_bbs(CARLA_OBJECT_TYPES[actor_filter]['label'])
            for carla_bbox in carla_bbox_list:
                bbox = self.get_image_bbox(carla_bbox, cls=CARLA_OBJECT_TYPES[actor_filter]['cls'])
                if len(bbox) == 0:
                    continue
                result_bboxes.append(bbox)

        return result_bboxes

    def get_image_bbox(self, carla_bbox, cls):
        # Check max range
        if carla_bbox.location.distance(self.parent.get_location()) > self.range:
            return []

        # Get bbox 3D corners in the bbox space
        bb_cords = self._create_bb_points(carla_bbox.extent)

        # Transform bboxes from bbox space to world space
        bb_world = np.dot(self.get_matrix(carla_bbox), np.transpose(bb_cords))

        # Transform bboxes from world space to sensor space
        bb_sensor = self._bboxes_to_sensor(bb_world)[:3, :]

        # Change coordinates sys: x y z -> y -z x  (UE4 to a standard)
        bb_points = np.concatenate([bb_sensor[1, :], -bb_sensor[2, :], bb_sensor[0, :]])

        # Project bboxes to image
        bbox = np.transpose(np.dot(self.calib, bb_points))
        camera_bbox = np.concatenate([bbox[:, 0] / bbox[:, 2], bbox[:, 1] / bbox[:, 2], bbox[:, 2]], axis=1)

        if all(camera_bbox[:, 2] > 0):
            return camera_bbox
        return []

    def get_rovis_bboxes(self, actor_filters):
        actors = self.world.get_actors()
        # rovis_bboxes = [ {center, cls, size, rot}, ... ]
        rovis_bboxes = []
        for actor_filter in actor_filters:
            actor_list = actors.filter(CARLA_OBJECT_TYPES[actor_filter]['tag'])
            for actor in actor_list:
                rovis_bbox = self.get_rovis_bbox(actor)
                if rovis_bbox is not None:
                    rovis_bboxes.append(rovis_bbox)

        return rovis_bboxes

    def get_rovis_bbox(self, actor):
        rovis_bbox = {
            'center': {'x': 0, 'y': 0, 'z': 0},
            'cls': -1,
            'size': {'w': 0, 'h': 0, 'l': 0},
            'rot': {'yaw': 0, 'pitch': 0, 'roll': 0}
        }

        # Check max range
        if actor.get_location().distance(self.parent.get_location()) > self.range:
            return None

        # Get bbox size
        extent = actor.bounding_box.extent
        rovis_bbox['size']['w'] = 2 * extent.y
        rovis_bbox['size']['h'] = 2 * extent.z
        rovis_bbox['size']['l'] = 2 * extent.x

        # Check sizes
        if rovis_bbox['size']['w'] < 0.01 or rovis_bbox['size']['h'] < 0.01 or rovis_bbox['size']['l'] < 0.01:
            return None

        # Get center of bbox of actor in world coordinates
        bb_center_veh = actor.bounding_box.location
        bb_center_veh = np.array([bb_center_veh.x, bb_center_veh.y, bb_center_veh.z, 1])
        vehicle_world_matrix = self.get_matrix(actor.get_transform())
        bbox_center_world = np.dot(vehicle_world_matrix, np.transpose(bb_center_veh))

        # Transform bboxes from world space to vehicle space
        world_vehicle_matrix = np.linalg.inv(self.get_matrix(self.parent.get_transform()))
        bbox_center_veh = np.dot(world_vehicle_matrix, np.transpose(bbox_center_world))
        # bbox_center_veh = [x, y, z]

        # Transform bboxes from Carla to Rovis
        carla_rovis_mat = self.get_matrix(carla.Transform(
            carla.Location(x=0.0, y=0.0, z=0.0),
            carla.Rotation(pitch=0.0, yaw=0.0, roll=180.0)  # 0 -90 180
            # Rovis note: pitch-x, yaw-z, roll-y ?
        ))
        bbox_center_rovis = np.dot(carla_rovis_mat, bbox_center_veh)[:3, :]

        # Save bbox center position
        rovis_bbox['center']['x'] = bbox_center_rovis[0, 0]
        rovis_bbox['center']['y'] = bbox_center_rovis[1, 0]
        rovis_bbox['center']['z'] = -bbox_center_rovis[2, 0]

        # Get rotation
        rovis_bbox['rot'] = self.get_bboxes_rotation(actor.get_transform().rotation)

        # Get class
        rovis_bbox['cls'] = self.get_cls(actor)

        return rovis_bbox

    @staticmethod
    def _create_bb_points(actor):
        cords = np.zeros((8, 4))
        extent = actor.bounding_box.extent  # Distance of bbox3D / 2
        cords[0, :] = np.array([extent.x, extent.y, -extent.z, 1])
        cords[1, :] = np.array([-extent.x, extent.y, -extent.z, 1])
        cords[2, :] = np.array([-extent.x, -extent.y, -extent.z, 1])
        cords[3, :] = np.array([extent.x, -extent.y, -extent.z, 1])
        cords[4, :] = np.array([extent.x, extent.y, extent.z, 1])
        cords[5, :] = np.array([-extent.x, extent.y, extent.z, 1])
        cords[6, :] = np.array([-extent.x, -extent.y, extent.z, 1])
        cords[7, :] = np.array([extent.x, -extent.y, extent.z, 1])
        return cords

    def get_bboxes_rotation(self, actor_rot):
        def limit_angle(angle):  # In interval [-180 180]
            while angle >= 180:
                angle -= 360
            while angle <= -180:
                angle += 360
            return angle

        def deg2rad(angle):
            return angle*3.14/180

        rot = {'yaw': 0, 'pitch': 0, 'roll': 0}
        parent_rot = self.parent.get_transform().rotation

        rot['yaw'] = -deg2rad(limit_angle(actor_rot.yaw - parent_rot.yaw))
        rot['pitch'] = deg2rad(limit_angle(actor_rot.pitch - parent_rot.pitch))
        rot['roll'] = deg2rad(limit_angle(actor_rot.roll - parent_rot.roll))

        return rot

    def _bboxes_to_world(self, cords, actor):
        # BBox to vehicle matrix
        # bb_vehicle_matrix = self.get_matrix(carla.Transform(actor.bounding_box.get_transform()))
        bb_vehicle_matrix = self.get_matrix(carla.Transform(actor.bounding_box.location))

        # Vehicle to world matrix
        vehicle_world_matrix = self.get_matrix(actor.get_transform())

        # Apply transformations
        bb_world_matrix = np.dot(vehicle_world_matrix, bb_vehicle_matrix)
        bbox_world_cords = np.dot(bb_world_matrix, np.transpose(cords))
        return bbox_world_cords

    def _bboxes_to_sensor(self, bb_world):
        # World to vehicle matrix
        world_vehicle_matrix = np.linalg.inv(self.get_matrix(self.parent.get_transform()))

        # Vehicle to sensor
        vehicle_sensor_matrix = self.veh_sensor_mat

        # Apply transformations
        world_sensor_matrix = np.dot(vehicle_sensor_matrix, world_vehicle_matrix)
        sensor_cords = np.dot(world_sensor_matrix, bb_world)
        return sensor_cords

    @staticmethod
    def get_cls(actor):
        sem_tags = actor.semantic_tags

        ignored_list = ('spectator', 'traffic.unknown', 'sensor', 'controller')
        if actor.type_id.startswith(ignored_list):
            return 0  # Unlabeled / invalid

        if actor.type_id == 'traffic.traffic_light':
            return sem_tags[0] if sem_tags[0] != 5 else sem_tags[1]

        if actor.type_id.startswith('traffic'):  # Traffic sign
            return 18  # Traffic sign cls

        return sem_tags[0]

    @staticmethod
    def draw_bboxes(bboxes, image, thickness=1):
        for bbox in bboxes:
            points = [(int(bbox[i, 0]), int(bbox[i, 1])) for i in range(8)]

            # base
            image = cv2.line(image, points[0], points[1], (255, 0, 0), thickness)
            image = cv2.line(image, points[1], points[2], (255, 0, 0), thickness)
            image = cv2.line(image, points[2], points[3], (255, 0, 0), thickness)
            image = cv2.line(image, points[3], points[0], (255, 0, 0), thickness)
            # top
            image = cv2.line(image, points[4], points[5], (0, 255, 0), thickness)
            image = cv2.line(image, points[5], points[6], (0, 255, 0), thickness)
            image = cv2.line(image, points[6], points[7], (0, 255, 0), thickness)
            image = cv2.line(image, points[7], points[4], (0, 255, 0), thickness)
            # base-top
            image = cv2.line(image, points[0], points[4], (0, 0, 255), thickness)
            image = cv2.line(image, points[1], points[5], (0, 0, 255), thickness)
            image = cv2.line(image, points[2], points[6], (0, 0, 255), thickness)
            image = cv2.line(image, points[3], points[7], (0, 0, 255), thickness)
        return image

    @staticmethod
    def get_matrix(transform):
        # Generate transformation matrix from transform
        # Transformation matrix: R_yaw * -R_pitch * -R_roll + T
        rotation = transform.rotation
        location = transform.location
        c_y = np.cos(np.radians(rotation.yaw))
        s_y = np.sin(np.radians(rotation.yaw))
        c_r = np.cos(np.radians(rotation.roll))
        s_r = np.sin(np.radians(rotation.roll))
        c_p = np.cos(np.radians(rotation.pitch))
        s_p = np.sin(np.radians(rotation.pitch))
        matrix = np.matrix(np.identity(4))
        matrix[0, 3] = location.x
        matrix[1, 3] = location.y
        matrix[2, 3] = location.z
        matrix[0, 0] = c_p * c_y
        matrix[0, 1] = c_y * s_p * s_r - s_y * c_r
        matrix[0, 2] = -c_y * s_p * c_r - s_y * s_r
        matrix[1, 0] = s_y * c_p
        matrix[1, 1] = s_y * s_p * s_r + c_y * c_r
        matrix[1, 2] = -s_y * s_p * c_r + c_y * s_r
        matrix[2, 0] = s_p
        matrix[2, 1] = -c_p * s_r
        matrix[2, 2] = c_p * c_r
        return matrix


if __name__ == '__main__':
    print('NOT runnable. Check \'Run_Carla.py\'')
