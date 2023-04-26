import os
import sys
import glob
import time
import numpy as np
import math
import cv2
import weakref

import config_utils as conf
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


class Bbox2DSensor(BaseSensor):
    def __init__(self, name, parent_actor, client_args):
        super().__init__(name, parent_actor, client_args)
        self.has_capture = True
        self.world = None
        self.bbox_handler = None
        self.range = -1  # Ignore range
        self.filters = 'vehicle,walker'

        self.generate_calib_mat()

    # ==================== General sensor methods ====================
    def parse_args(self):
        super().parse_args()

        for key in self.args.keys():
            if key == 'range':
                self.range = self.args[key]

    def setup(self):
        self.world = self._parent.get_world()
        self.bbox_handler = BoundingBoxesHandler(self._parent, self, self.range)

    def get_data(self, optional_data_type=''):
        # Get filters
        actor_filters = []
        for data_type in self.filters.split(','):
            actor_filters.append('{}.*'.format(data_type))

        if len(actor_filters) == 0:
            return []

        # Get bounding boxes 2D
        bboxes = []
        try:
            if self.bbox_handler is not None:
                bboxes = self.bbox_handler.get_bboxes(actor_filters)
        except RuntimeError:  # Client closed but still tries to process this
            return []

        return bboxes  # bboxes - [[x_min, y_min, x_max, y_max, cls], ...]

    @staticmethod
    def sensor_callback(weak_ref, data):
        pass

    # ==================== Viewer methods ====================
    def do_view(self):
        window_name = '{} - {}'.format(self.parent_name, self.name)
        background = np.zeros((self.height, self.width, 3), dtype=np.dtype('uint8'))
        while not self.stop_view:
            bboxes = self.get_data()
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
        if self.send:
            bboxes, cls = self.get_data()

            if bboxes is not None:
                # TODO data = ...
                return bytes(str(bboxes), 'utf8')
        return None

    # ==================== Pack data for saving ====================
    def pack_data(self, ts_stop, frame_id):
        bboxes = self.get_data()

        roi_id, x, y, width, height, cls = [], [], [], [], [], []
        for idx, bbox in enumerate(bboxes):
            roi_id.append(idx + 1)  # Roi id starts at 1
            cls.append(bbox[4])
            x.append(bbox[0])
            y.append(bbox[1])
            width.append(bbox[2] - bbox[0])
            height.append(bbox[3] - bbox[1])

        packed_data = {
            'frame_id': frame_id,
            'roi_id': roi_id,
            'cls': cls,
            'x': x,
            'y': y,
            'width': width,
            'height': height
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

        # This matrix stays the same, so it's faster to compute this only once
        self.veh_sensor_mat = np.linalg.inv(self.get_matrix(self.transform))

    def get_bboxes(self, actor_filters):
        actors = self.world.get_actors()
        bboxes = []
        for actor_filter in actor_filters:
            actor_list = actors.filter(actor_filter)
            for actor in actor_list:
                bbox = self.get_bbox(actor)
                if len(bbox) == 0:
                    continue
                bboxes.append(bbox)

        return bboxes

    def get_bbox(self, actor):
        # Check max range
        if actor.get_location().distance(self.parent.get_location()) > self.range:
            return []

        # Get bbox 3D corners in the vehicle space and bbox real size
        bb_cords = self._create_bb_points(actor)

        # Transform bboxes from vehicle space to world space
        bb_world = self._bboxes_to_world(bb_cords, actor)

        # Transform bboxes from world space to sensor space
        bb_sensor = self._bboxes_to_sensor(bb_world)[:3, :]

        # Change cords sys: x y z -> y -z x
        bb_points = np.concatenate([bb_sensor[1, :], -bb_sensor[2, :], bb_sensor[0, :]])

        # Project bboxes to image
        bb_img = np.transpose(np.dot(self.calib, bb_points))
        bb_img = np.concatenate([bb_img[:, 0] / bb_img[:, 2], bb_img[:, 1] / bb_img[:, 2], bb_img[:, 2]], axis=1)

        if not all(bb_img[:, 2] > 0):
            return []

        # Calculate bbox 2D from 3D
        x_min = x_max = int(bb_img[0, 0])
        y_min = y_max = int(bb_img[0, 1])
        for i in range(1, 8):
            x_min = min(int(bb_img[i, 0]), x_min)
            x_max = max(int(bb_img[i, 0]), x_max)
            y_min = min(int(bb_img[i, 1]), y_min)
            y_max = max(int(bb_img[i, 1]), y_max)

        # Check if bbox appears in the image
        x_min = max(min(x_min, self.sensor.width), 0)
        x_max = max(min(x_max, self.sensor.width), 0)
        y_min = max(min(y_min, self.sensor.height), 0)
        y_max = max(min(y_max, self.sensor.height), 0)

        if x_min == x_max or y_min == y_max:
            return []

        return [x_min, y_min, x_max, y_max, self.get_cls(actor)]

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
            # bbox = [x_min, y_min, x_max, y_max]
            image = cv2.line(image, (bbox[0], bbox[1]), (bbox[0], bbox[3]), (255, 0, 0), thickness)
            image = cv2.line(image, (bbox[0], bbox[3]), (bbox[2], bbox[3]), (255, 0, 0), thickness)
            image = cv2.line(image, (bbox[2], bbox[3]), (bbox[2], bbox[1]), (255, 0, 0), thickness)
            image = cv2.line(image, (bbox[2], bbox[1]), (bbox[0], bbox[1]), (255, 0, 0), thickness)
        return image

    @staticmethod
    def get_matrix(transform):
        # Generate transformation matrix from transform
        # Transformation matrix: R_yaw * -R_pitch * -R_roll * T
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
