"""
    RovisDatabaseFormat.py
Author: Tomuta Gabriel

This script creates a Rovis type Database.
"""

import os
import cv2
import csv
import time
from shutil import copy
import pandas as pd
import numpy as np

# Modified for use here
from depend.types_ROVIS_TYPES import RovisDataType, RovisFilterType
__all__ = [
    'RovisDataBase',
    'create_obj_cls_file',
    'create_calib_file'
]

"""
Example:

def create_data_base_example():
    # Create empty database
    db = RovisDataBase(db_path='C:/dev/src/RovisLab/NewDataBase', core_id=1)
    db.add_stream(filter_type='image', filter_id=1, filter_name='CameraFront')
    db.add_stream(filter_type='sem_seg', filter_id=2, filter_name='SemSegFront', input_sources=[1])
    # db.show_packing_info()
    db.create_db()

    # Example of timestamp generation
    timestamp_start = int(time.time())
    timestamp_stop = timestamp_start
    sampling_time = 5
    
    # Create object classes and add it to ds 2
    classes = [{'name': _, 'countable': True}, {_}]
    obj_cls = create_obj_cls_file(classes)
    db.add_custom(filter_id=2, data=obj_cls, name='object_classes.conf')

    for _ in _:
        image_path = 'test_image.jpg'
        image = cv2.imread(image_path)

        image_gray_path = 'test_image_gray.png'
        image_gray = cv2.imread(image_gray_path)
        shape_id = 1
        cls = 1
        instance = 0
        points = '[[545.0 243.0][569.0 223.0][639.0 204.0]...]'

        # Pack data ( for the data format, run db.show_packing_info() )
        data = {
            'datastream_1': {  # image
                'name': '{}.jpg'.format(timestamp_stop),
                'image': image / image_path
            },
            'datastream_2': {  # sem_seg
                'name': '{}.png'.format(timestamp_stop),
                'semantic': image_gray / image_gray_path,
                'instances': image_gray / image_gray_path,
                'shape_id': shape_id,
                'cls': cls,
                'instance': instance,
                'points': points
            }
        }

        # Add data to database
        db.add_data(ts_start=timestamp_start, ts_stop=timestamp_stop, data=data)

        # Example advance timestamp
        timestamp_start = timestamp_stop
        timestamp_stop += sampling_time
"""


# Utils ================================================================================================================
def mkdir(path, echo=False):
    try:
        os.mkdir(path=path)
    except FileExistsError:
        if echo:
            print(path + ' already exists.')
        pass


def str2bool(v):
    if isinstance(v, bool):
        return v
    if v.lower() in ('yes', 'True', 'true', 't', 'y', '1'):
        return True
    elif v.lower() in ('no', 'False', 'false', 'f', 'n', '0'):
        return False
    else:
        print('Boolean value expected.')
        return False


def create_obj_cls_file(classes: list) -> str:
    """
    classes = [
        {
            'name': <name: str>,
            'countable': <bool>
        },
        {...}
    ]
    """
    obj_cls = 'ObjectClasses:\n{\n'
    for idx, cls in enumerate(classes):
        obj_cls += '    ' + cls['name']
        obj_cls += ':\n    {\n'
        obj_cls += '        ID = ' + str(idx) + '\n'
        obj_cls += '        Countable = ' + ('True' if cls['countable'] else 'False')
        obj_cls += '\n    }\n'
        time.sleep(0.01)
    obj_cls += '}'
    return obj_cls


def create_calib_file(name: str, calib: dict) -> str:
    """
    calib = {
        'width': 0,  # Width of image
        'height': 0,  # Height of image

        'rx': 0,  # Rotation X
        'ry': 0,  # Rotation Y
        'rz': 0,  # Rotation Z
        'tx': 0,  # Translation X
        'ty': 0,  # Translation Y
        'tz': 0,  # Translation Z

        'ch': 0,  # Channels
        'fx': 0,  # Focal length X
        'fy': 0,  # Focal length Y
        'cx': 0,  # Optical center X
        'cy': 0,  # Optical center Y
        'px': 0,  # Pixel size X
        'py': 0,  # Pixel size Y
        'dist0': 0,  # Distance coef 0
        'dist1': 0,  # Distance coef 1
        'dist2': 0,  # Distance coef 2
        'dist3': 0,  # Distance coef 3
        'dist4': 0,  # Distance coef 4

        'bline': 0,  # Baseline
    }
    """

    cal_file = '//{}\n\n'.format(name)
    cal_file += 'image_width = {};\n'.format(calib['width'])
    cal_file += 'image_height = {};\n\n'.format(calib['height'])
    cal_file += 'Pose =\n{\n'
    cal_file += '   Rotation = \n   {\n'
    cal_file += '      x = {};\n'.format(calib['rx'])
    cal_file += '      y = {};\n'.format(calib['ry'])
    cal_file += '      z = {};\n'.format(calib['rz'])
    cal_file += '   }\n'
    cal_file += '   Translation = \n   {\n'
    cal_file += '      x = {};\n'.format(calib['tx'])
    cal_file += '      y = {};\n'.format(calib['ty'])
    cal_file += '      z = {};\n'.format(calib['tz'])
    cal_file += '   }\n}\n'
    cal_file += 'LeftSensor =\n{\n'
    cal_file += '   channels = {};\n'.format(calib['ch'])
    cal_file += '   focal_length_x = {};\n'.format(calib['fx'])
    cal_file += '   focal_length_y = {};\n'.format(calib['fy'])
    cal_file += '   optical_center_x = {};\n'.format(calib['cx'])
    cal_file += '   optical_center_y = {};\n'.format(calib['cy'])
    cal_file += '   pixel_size_x = {};\n'.format(calib['px'])
    cal_file += '   pixel_size_y = {};\n'.format(calib['py'])
    cal_file += '   dist_coeff_0 = {};\n'.format(calib['dist0'])
    cal_file += '   dist_coeff_1 = {};\n'.format(calib['dist1'])
    cal_file += '   dist_coeff_2 = {};\n'.format(calib['dist2'])
    cal_file += '   dist_coeff_3 = {};\n'.format(calib['dist3'])
    cal_file += '   dist_coeff_4 = {};\n'.format(calib['dist4'])
    cal_file += '}\n\n'
    cal_file += 'baseline = {};'.format(calib['bline'])

    return cal_file


class TsSync:
    def __init__(self, path, sync_data):
        self.path = path
        self.sync_data = sync_data

    def create_csv(self, dstreams):
        with open(self.path, 'w', newline='') as f:
            writer = csv.writer(f)
            header = ['timestamp_stop'] + dstreams
            writer.writerow(header)

    def add_ds(self, filter_id, input_src):
        df = pd.read_csv(self.path)

        if 'datastream_{}'.format(filter_id) in df:
            print('Datastream already exists.')
            print('Exiting..')

        if len(input_src):
            idx = 0
            for elem in input_src:
                if 'datastream_{}'.format(elem) in df:
                    idx = elem
                    break

            if idx != 0:
                df['datastream_{}'.format(filter_id)] = df['datastream_{}'.format(idx)]
            else:
                df['datastream_{}'.format(filter_id)] = -1
        else:
            df['datastream_{}'.format(filter_id)] = -1

        df.to_csv(self.path, index=False)

    def add_data(self, ts_stop, values):
        if self.sync_data:
            self.add_data_pd(ts_stop=ts_stop, values=values)
        else:
            self.add_data_fast(ts_stop=ts_stop, values=values)

    def add_data_pd(self, ts_stop, values):
        in_df = pd.read_csv(self.path)

        start_df = in_df[in_df["timestamp_stop"] < ts_stop]
        line_df = in_df[in_df["timestamp_stop"] == ts_stop]
        end_df = in_df[in_df["timestamp_stop"] > ts_stop]

        if line_df.empty:
            line_df = pd.DataFrame(data=[[ts_stop] + values], columns=start_df.columns)
            out_df = pd.DataFrame(np.concatenate([start_df.values, line_df.values, end_df.values]),
                                  columns=start_df.columns)
        else:
            for i in range(0, len(values)):
                if line_df.values[0][i+1] == -1:
                    if values[i] != -1:
                        line_df.values[0][i+1] = values[i]

            out_df = pd.DataFrame(np.concatenate([start_df.values, line_df.values, end_df.values]),
                                  columns=start_df.columns)

        out_df.to_csv(self.path, index=False)

    def add_data_fast(self, ts_stop, values):
        with open(self.path, 'a', newline='') as f:
            writer = csv.writer(f)
            row = [ts_stop] + values

            writer.writerow(row)


# Main Database class ==================================================================================================
class RovisDataBase:
    # Class used by the Stream classes
    class RovisStream:
        def __init__(self, db_path, filter_id, filter_name, input_sources):
            self.id = filter_id
            self.name = filter_name
            self.filter_type = RovisFilterType.ROVIS_UNDEFINED_FILTER_TYPE
            self.output_type = RovisDataType.ROVIS_UNDEFINED
            self.input_sources = input_sources
            self.dir = db_path + '/datastream_{}'.format(self.id)
            self.created = False

        def create_stream(self):
            raise NotImplementedError()

        def add_to_stream(self, ts_start, ts_stop, data):
            raise NotImplementedError()

        def get_packing_info(self):
            return '    \'datastream_{}\': '.format(self.id) + '{  #' + self.add_to_stream.__doc__ + '}'

        def get_input_sources_str(self, core_id):
            """
                Description:
                    Method used for converting the numeral input sources to string for datablock_descriptor
                Parameters:
                    @param: core_id: int
                Returns:
                    string: input sources representation
                Example:
                    core_id = 1
                    self.input_sources = [1, 2, 4]
                    => {1-1;1-2;1-4}
            """

            if not len(self.input_sources):
                return ''

            string = '{'
            for in_src in self.input_sources:
                string += '{0}-{1}'.format(core_id, in_src)
            string += '}'

            return string

        def is_created(self):
            return self.created

    def __init__(self, db_path, core_id, sync_data=False):
        self.db_path = db_path
        self.core_id = core_id
        self.datastreams = {}

        # If this is True, the program will be able to handle multiple data added separately on the same timestamp
        # but it will take longer to compute. Make it True only if needed.
        self.sync_data = sync_data
        self.ts_sync = None

        self.created = False

    # Database creation methods
    def create_db(self):
        if self.created:
            print('Database already created once. Exiting..')
            return

        # Create database folder
        mkdir(self.db_path)

        # Create csvs
        self.create_datablock_descriptor()
        self.create_sampling_timestamps_sync()

        # Create streams
        self.create_streams()

        # Create batch
        bat = open(self.db_path + '/run_viz.bat', 'w+')
        bat.write('python %ROVISLAB_DIR%/RovisDojo/tools/Annotation/main_annotation.py %cd%')
        bat.close()

        self.created = True
        print('Database successfully created at {}/.\n'.format(self.db_path))

    def update_db(self):
        # Update csvs
        self.update_datablock_descriptor()
        self.update_sampling_timestamps_sync()

        # Create new streams
        self.create_streams()

    def create_datablock_descriptor(self):
        with open(self.db_path + '/datablock_descriptor.csv', 'w', newline='') as f:
            writer = csv.writer(f)
            header = ['vision_core_id', 'filter_id', 'name', 'type', 'output_data_type', 'input_sources']
            writer.writerow(header)

            for key in sorted(self.datastreams, key=lambda dict_key: int(dict_key.split('_')[-1])):
                row = [str(self.core_id), self.datastreams[key].id, self.datastreams[key].name,
                       self.datastreams[key].filter_type, self.datastreams[key].output_type]
                if self.datastreams[key].input_sources is not None:
                    row.append(str(self.datastreams[key].get_input_sources_str(self.core_id)))

                writer.writerow(row)

    def update_datablock_descriptor(self):
        self.create_datablock_descriptor()

    def create_sampling_timestamps_sync(self):
        if self.ts_sync is not None:
            print('Ts Synk was created already.')

        self.ts_sync = TsSync(path=self.db_path + '/sampling_timestamps_sync.csv',
                              sync_data=self.sync_data)
        self.ts_sync.create_csv(dstreams=list(sorted(self.datastreams,
                                                     key=lambda dict_key: int(dict_key.split('_')[-1]))))

    def update_sampling_timestamps_sync(self):
        for key in sorted(self.datastreams, key=lambda dict_key: int(dict_key.split('_')[-1])):
            if not self.datastreams[key].is_created():
                self.ts_sync.add_ds(filter_id=self.datastreams[key].id,
                                    input_src=self.datastreams[key].input_sources)

    def add_data(self, ts_start, ts_stop, data):
        """
            Description:
                Method used for adding data to the created Rovis Database
            Parameters:
                @param: ts_start, ts_stop: int, timestamps
                @param: data: dict, for format, run show_packing_info() method
            Returns:
                True if data added successfully, False otherwise
        """

        # Check if the database is created
        if not self.created:
            print('Database not created. Could not add data.')
            print('Use function self.create_db() in order to create it.')
            print('Exiting..')
            return False

        # Check if all the streams are created
        for key in self.datastreams.keys():
            if not self.datastreams[key].is_created():
                print('For data to be added to the database, all streams must be created.')
                print('Use function self.update_db() in order to create missing streams.')
                print('Exiting..')
                return False

        # Prepare sampling_timestamps to add to csv
        sampling = {}
        for key in sorted(self.datastreams, key=lambda dict_key: int(dict_key.split('_')[-1])):
            sampling[key] = -1

        # Add data to datastreams
        for key in data.keys():
            if key not in self.datastreams.keys():
                print('Data of {} could not be added. Stream does not exist'.format(key))
                continue
            self.datastreams[key].add_to_stream(ts_start, ts_stop, data[key])
            sampling[key] = ts_stop

        # Add data to sampling_timestamps_sync.csv
        self.ts_sync.add_data(ts_stop=ts_stop, values=list(sampling.values()))

        return True

    # Stream methods
    def add_stream(self, filter_type, filter_id, filter_name, input_sources=None):
        if 'datastream_{}'.format(filter_id) in self.datastreams.keys():
            print('Filter with id {} already exists.'.format(filter_id))
            print('Exiting..')
            return

        stream = None
        if filter_type in ['ROVIS_MONO_CAMERA_FILTER_TYPE', 'image', 'camera']:
            stream = RovisImageStream(self.db_path, filter_id, filter_name)
        elif filter_type in ['ROVIS_LIDAR_FILTER_TYPE', 'lidar']:
            stream = RovisLidarStream(self.db_path, filter_id, filter_name)
        elif filter_type in ['ROVIS_RADAR_FILTER_TYPE', 'radar']:
            stream = RovisRadarStream(self.db_path, filter_id, filter_name)
        elif filter_type in ['ROVIS_VEHICLE_STATE_ESTIMATION_FILTER_TYPE', 'veh_state']:
            stream = RovisVehStateStream(self.db_path, filter_id, filter_name)
        elif filter_type in ['ROVIS_OBJECT_DETECTOR_2D_FILTER_TYPE', '2D_obj_det', 'det2d']:
            stream = Rovis2DObjDetStream(self.db_path, filter_id, filter_name, input_sources)
        elif filter_type in ['ROVIS_OBJECT_DETECTOR_3D_FILTER_TYPE', '3D_obj_det', 'det3d']:
            stream = Rovis3DObjDetStream(self.db_path, filter_id, filter_name, input_sources)
        elif filter_type in ['ROVIS_IMU_FILTER_TYPE', 'imu']:
            stream = RovisIMUStream(self.db_path, filter_id, filter_name)
        elif filter_type in ['ROVIS_ULTRASONICS_FILTER_TYPE', 'ultrasonic']:
            stream = RovisUltrasonicStream(self.db_path, filter_id, filter_name)
        elif filter_type in ['ROVIS_LANE_DETECTION_FILTER_TYPE', 'lane', 'lanes']:
            stream = RovisLaneStream(self.db_path, filter_id, filter_name, input_sources)
        elif filter_type in ['ROVIS_SEMANTIC_SEGMENTATION_FILTER_TYPE', 'sem_seg', 'semseg', 'semantic']:
            stream = RovisSemSegStream(self.db_path, filter_id, filter_name, input_sources)
        elif filter_type in ['ROVIS_ROTARY_ENCODER_FILTER_TYPE', 'wheels_encoder', 'wheels_enc', 'encoder']:
            stream = RovisWheelsEncoderStream(self.db_path, filter_id, filter_name)
        else:
            print('Stream {} could not be added to database.'.format(filter_type))

        if stream is not None:
            self.datastreams['datastream_{}'.format(filter_id)] = stream

    def create_streams(self):
        for key in sorted(self.datastreams, key=lambda dict_key: int(dict_key.split('_')[-1])):
            if not self.datastreams[key].is_created():
                self.datastreams[key].create_stream()

    def add_custom(self, filter_id: int, data: any, name: str):
        if filter_id == 0:  # Add to database root
            save_path = self.db_path
        else:  # Add to a datastream folder
            if 'datastream_{}'.format(filter_id) not in self.datastreams.keys():
                print('Datastream {} does not exist.'.format(filter_id))
                return
            if not self.datastreams['datastream_{}'.format(filter_id)].is_created():
                print('Datastream {} is not created yet.'.format(filter_id))
                return
            save_path = self.datastreams['datastream_{}'.format(filter_id)].dir

        if os.path.isfile(data):  # Copy existing file
            copy(data, save_path + '/' + name)
        elif type(data) == str:  # If formatted string
            with open(save_path + '/' + name, 'w') as file:
                file.write(data)

    def show_packing_info(self, stream_ids=()):
        if len(stream_ids) == 0:
            streams = list(sorted(self.datastreams, key=lambda dict_key: int(dict_key.split('_')[-1])))
        else:
            aux = list()
            for stream_id in stream_ids:
                if 'datastream_{}'.format(stream_id) in self.datastreams.keys():
                    aux.append('datastream_{}'.format(stream_id))
            if len(aux) == 0:
                print('There are no streams with those ids.')
                return
            streams = aux

        print('Packing data format code: ')
        print('data = {')
        last_key = streams[-1]
        for key in streams:
            if key == last_key:
                print(self.datastreams[key].get_packing_info())
            else:
                print(self.datastreams[key].get_packing_info() + ',')
        print('}')


# Different Rovis Filter Streams =======================================================================================
class RovisImageStream(RovisDataBase.RovisStream):
    def __init__(self, db_path, filter_id, filter_name, input_sources=()):
        super().__init__(db_path, filter_id, filter_name, input_sources)
        self.filter_type = RovisFilterType.ROVIS_MONO_CAMERA_FILTER_TYPE
        self.output_type = RovisDataType.ROVIS_IMAGE

    def create_stream(self):
        # create dir and image folder
        mkdir(self.dir)
        mkdir(self.dir + '/samples')
        mkdir(self.dir + '/samples/0')
        mkdir(self.dir + '/samples/0/left')
        mkdir(self.dir + '/samples/0/right')

        # create csv file
        with open(self.dir + '/data_descriptor.csv', 'w', newline='') as f:
            writer = csv.writer(f)
            header = ['timestamp_start', 'timestamp_stop', 'sampling_time', 'left_file_path_0', 'right_file_path_0']
            writer.writerow(header)

        self.created = True

    def add_to_stream(self, ts_start, ts_stop, data):
        """ image
            'name': !str,
            'image': !array[][][] / str,
            'image_right': array[][][] / str
        """
        # Save image (left)
        image_path = 'samples/0/left' + '/' + data['name']
        if isinstance(data['image'], str):
            copy(data['image'], self.dir + '/' + image_path)
        else:
            cv2.imwrite(self.dir + '/' + image_path, data['image'])

        # Save image (right)
        right_image_path = '-1'
        if 'image_right' in data:
            right_image_path = 'samples/0/right' + '/' + data['name']
            if isinstance(data['image_right'], str):
                copy(data['image_right'], self.dir + '/' + right_image_path)
            else:
                cv2.imwrite(self.dir + '/' + right_image_path, data['image_right'])

        # Update csv
        with open(self.dir + '/data_descriptor.csv', 'a', newline='') as f:
            writer = csv.writer(f)
            row = [ts_start, ts_stop, ts_stop - ts_start, image_path, right_image_path]
            writer.writerow(row)


class RovisLidarStream(RovisDataBase.RovisStream):
    def __init__(self, db_path, filter_id, filter_name, input_sources=()):
        super().__init__(db_path, filter_id, filter_name, input_sources)
        self.filter_type = RovisFilterType.ROVIS_LIDAR_FILTER_TYPE
        self.output_type = RovisDataType.ROVIS_VOXELS

    def create_stream(self):
        # create dir and lidar files folder
        mkdir(self.dir)
        mkdir(self.dir + '/samples')

        # create csv file
        with open(self.dir + '/data_descriptor.csv', 'w', newline='') as f:
            writer = csv.writer(f)
            header = ['timestamp_start', 'timestamp_stop', 'sampling_time', 'lidar_file_path_0']
            writer.writerow(header)

        self.created = True

    @staticmethod
    def write_lidar_file(file_path, points):
        if len(points[0]) == 3:  # x y z
            header_xyz = '''ply
            format ascii 1.0
            element vertex %(vert_num)d
            property float x
            property float y
            property float z
            end_header
            '''
            with open(file_path, 'wb') as f:
                f.write((header_xyz % dict(vert_num=len(points))).encode('utf-8'))
                np.savetxt(f, points, fmt='%f %f %f')
        elif len(points[0]) == 4:  # x y z r  (r: reflectance / intensity?)
            header_xyzr = '''ply
            format ascii 1.0
            element vertex %(vert_num)d
            property float x
            property float y
            property float z
            property float r
            end_header
            '''
            with open(file_path, 'wb') as f:
                f.write((header_xyzr % dict(vert_num=len(points))).encode('utf-8'))
                np.savetxt(f, points, fmt='%f %f %f %f')

    def add_to_stream(self, ts_start, ts_stop, data):
        """ lidar
            'name': !str,
            'lidar_data': !array[][] / str
        """
        # Save lidar file
        if not data['name'].endswith('ply'):
            data['name'] += '.ply'
        lidar_path = 'samples' + '/' + data['name']

        if isinstance(data['lidar_data'], str):
            copy(data['lidar_data'], self.dir + '/' + lidar_path)
        else:
            self.write_lidar_file(self.dir + '/' + lidar_path, data['lidar_data'])

        # Update csv
        with open(self.dir + '/data_descriptor.csv', 'a', newline='') as f:
            writer = csv.writer(f)
            row = [ts_start, ts_stop, ts_stop - ts_start, lidar_path]
            writer.writerow(row)


class RovisRadarStream(RovisDataBase.RovisStream):
    def __init__(self, db_path, filter_id, filter_name, input_sources=()):
        super().__init__(db_path, filter_id, filter_name, input_sources)
        self.filter_type = RovisFilterType.ROVIS_RADAR_FILTER_TYPE
        self.output_type = RovisDataType.ROVIS_RADAR

    def create_stream(self):
        # create dir and radar files folder
        mkdir(self.dir)
        mkdir(self.dir + '/samples')

        # create data_descriptor
        with open(self.dir + '/data_descriptor.csv', 'w', newline='') as f:
            writer = csv.writer(f)
            header = ['timestamp_start', 'timestamp_stop', 'sampling_time', 'radar_file_path_0']
            writer.writerow(header)

        # # create framebased_data_descriptor
        # with open(self.dir + '/framebased_data_descriptor.csv', 'w', newline='') as f:
        #     writer = csv.writer(f)
        #     header = ['timestamp_stop']
        #     writer.writerow(header)

        self.created = True

    @staticmethod
    def write_radar_file(file_path, points):
        if len(points[0]) == 3:  # x y z
            radar_xyz_header = '''ply
                    format ascii 1.0
                    element vertex %(vert_num)d
                    property float x
                    property float y
                    property float z
                    end_header
                    '''
            with open(file_path, 'wb') as f:
                f.write((radar_xyz_header % dict(vert_num=len(points))).encode('utf-8'))
                np.savetxt(f, points, fmt='%f %f %f')
        elif len(points[0]) == 4:  # vel, altitude, azimuth, depth
            radar_carla_header = '''ply
                    format ascii 1.0
                    element vertex %(vert_num)d
                    property float vel
                    property float altitude
                    property float azimuth
                    property float depth
                    end_header
                    '''
            with open(file_path, 'wb') as f:
                f.write((radar_carla_header % dict(vert_num=len(points))).encode('utf-8'))
                np.savetxt(f, points, fmt='%f %f %f %f')
        elif len(points[0]) == 18:  # default radar - header from nuscenes_converter
            radar_default_header = '''ply
                    format ascii 1.0
                    element vertex %(vert_num)d
                    property float x
                    property float y
                    property float z
                    property int dyn_prop
                    property int id
                    property float rcs
                    property float vx
                    property float vy
                    property float vx_comp
                    property float vy_comp
                    property int is_quality_valid
                    property int ambig_state
                    property int x_rms
                    property int y_rms
                    property int invalid_state
                    property int pdh0
                    property int vx_rms
                    property int vy_rms
                    end_header
                    '''
            with open(file_path, 'wb') as f:
                f.write((radar_default_header % dict(vert_num=len(points))).encode('utf-8'))
                np.savetxt(f, points, fmt='%f %f %f %d %d %f %f %f %f %f %d %d %d %d %d %d %d %d')

    def add_to_stream(self, ts_start, ts_stop, data):
        """ radar
            'name': !str,
            'radar_data': !array[][] / str
        """
        # Save radar file
        if not data['name'].endswith('ply'):
            data['name'] += '.ply'
        radar_path = 'samples' + '/' + data['name']
        if isinstance(data['radar_file'], str):
            copy(data['radar_file'], self.dir + '/' + radar_path)
        else:
            write_radar_file(self.dir + '/' + radar_path, data['radar_data'])

        # Update data_descriptor
        with open(self.dir + '/data_descriptor.csv', 'a', newline='') as f:
            writer = csv.writer(f)
            row = [ts_start, ts_stop, ts_stop - ts_start, radar_path]
            writer.writerow(row)

        # # Update framebased_data_descriptor
        # with open(self.dir + '/framebased_data_descriptor.csv', 'a', newline='') as f:
        #     writer = csv.writer(f)
        #     row = [ts_stop]
        #     writer.writerow(row)


class RovisVehStateStream(RovisDataBase.RovisStream):
    def __init__(self, db_path, filter_id, filter_name, input_sources=()):
        super().__init__(db_path, filter_id, filter_name, input_sources)
        self.filter_type = RovisFilterType.ROVIS_VEHICLE_STATE_ESTIMATION_FILTER_TYPE
        self.output_type = RovisDataType.ROVIS_STATE

    def create_stream(self):
        # Create dir
        mkdir(self.dir)

        # create csv file
        with open(self.dir + '/data_descriptor.csv', 'w', newline='') as f:
            writer = csv.writer(f)
            header = ['timestamp_start', 'timestamp_stop', 'sampling_time', 'state_variable_0',
                      'state_variable_1', 'state_variable_2', 'state_variable_3']
            writer.writerow(header)

        self.created = True

    def add_to_stream(self, ts_start, ts_stop, data):
        """ veh_state
            's_var_0': !float,
            's_var_1': !float,
            's_var_2': !float,
            's_var_3': !float
        """
        # Update csv
        with open(self.dir + '/data_descriptor.csv', 'a', newline='') as f:
            writer = csv.writer(f)
            row = [ts_start, ts_stop, ts_stop - ts_start,
                   data['s_var_0'], data['s_var_1'], data['s_var_2'], data['s_var_3']]
            writer.writerow(row)


class Rovis2DObjDetStream(RovisDataBase.RovisStream):
    def __init__(self, db_path, filter_id, filter_name, input_sources=()):
        super().__init__(db_path, filter_id, filter_name, input_sources)
        self.filter_type = RovisFilterType.ROVIS_OBJECT_DETECTOR_2D_FILTER_TYPE
        self.output_type = RovisDataType.ROVIS_2D_ROIS

    def create_stream(self):
        # create dir
        mkdir(self.dir)

        # create data_descriptor
        with open(self.dir + '/data_descriptor.csv', 'w', newline='') as f:
            writer = csv.writer(f)
            header = ['timestamp_start', 'timestamp_stop', 'sampling_time', 'frame_id']
            writer.writerow(header)

        # create framebased_data_descriptor
        with open(self.dir + '/framebased_data_descriptor.csv', 'w', newline='') as f:
            writer = csv.writer(f)
            header = ['frame_id', 'roi_id', 'cls', 'x', 'y', 'width', 'height']
            writer.writerow(header)

        self.created = True

    def add_to_stream(self, ts_start, ts_stop, data):
        """ 2D_det
            'frame_id': !int,
            'roi_id': ![int],
            'cls': ![int],
            'x': ![int],
            'y': ![int],
            'width': ![int],
            'height': ![int]
        """
        # Update data_descriptor
        with open(self.dir + '/data_descriptor.csv', 'a', newline='') as f:
            writer = csv.writer(f)
            row = [ts_start, ts_stop, ts_stop - ts_start, data['frame_id']]
            writer.writerow(row)

        # Update framebased_data_descriptor
        with open(self.dir + '/framebased_data_descriptor.csv', 'a', newline='') as f:
            writer = csv.writer(f)
            for i in range(len(data['roi_id'])):
                row = [data['frame_id'], data['roi_id'][i], data['cls'][i], data['x'][i],
                       data['y'][i], data['width'][i], data['height'][i]]
                writer.writerow(row)


class Rovis3DObjDetStream(RovisDataBase.RovisStream):
    def __init__(self, db_path, filter_id, filter_name, input_sources=()):
        super().__init__(db_path, filter_id, filter_name, input_sources)
        self.filter_type = RovisFilterType.ROVIS_OBJECT_DETECTOR_3D_FILTER_TYPE
        self.output_type = RovisDataType.ROVIS_3D_BBOXES

    def create_stream(self):
        # create dir
        mkdir(self.dir)

        # create data_descriptor
        with open(self.dir + '/data_descriptor.csv', 'w', newline='') as f:
            writer = csv.writer(f)
            header = ['timestamp_start', 'timestamp_stop', 'sampling_time', 'frame_id']
            writer.writerow(header)

        # create framebased_data_descriptor
        with open(self.dir + '/framebased_data_descriptor.csv', 'w', newline='') as f:
            writer = csv.writer(f)
            header = ['frame_id', 'roi_id', 'cls', 'x', 'y', 'z', 'w', 'h', 'l', 'roll', 'pitch', 'yaw']
            writer.writerow(header)

        self.created = True

    def add_to_stream(self, ts_start, ts_stop, data):
        """ 3D_det
            'frame_id': !int,
            'roi_id': ![int],
            'cls': ![int],
            'x': ![float],
            'y': ![float],
            'z': ![float],
            'w': ![float],
            'h': ![float],
            'l': ![float],
            'roll': ![float],
            'pitch': ![float],
            'yaw': ![float]
        """
        # Update data_descriptor
        with open(self.dir + '/data_descriptor.csv', 'a', newline='') as f:
            writer = csv.writer(f)
            row = [ts_start, ts_stop, ts_stop - ts_start, data['frame_id']]
            writer.writerow(row)

        # Update framebased_data_descriptor
        with open(self.dir + '/framebased_data_descriptor.csv', 'a', newline='') as f:
            writer = csv.writer(f)
            for i in range(len(data['roi_id'])):
                row = [data['frame_id'], data['roi_id'][i], data['cls'][i], data['x'][i], data['y'][i], data['z'][i],
                       data['w'][i], data['h'][i], data['l'][i], data['roll'][i], data['pitch'][i], data['yaw'][i]]
                writer.writerow(row)


class RovisIMUStream(RovisDataBase.RovisStream):
    def __init__(self, db_path, filter_id, filter_name, input_sources=()):
        super().__init__(db_path, filter_id, filter_name, input_sources)
        self.filter_type = RovisFilterType.ROVIS_IMU_FILTER_TYPE
        self.output_type = RovisDataType.ROVIS_IMU

    def create_stream(self):
        # Create dir
        mkdir(self.dir)

        # create csv file
        with open(self.dir + '/data_descriptor.csv', 'w', newline='') as f:
            writer = csv.writer(f)
            header = ['timestamp_start', 'timestamp_stop', 'sampling_time', 'acc_x', 'acc_y', 'acc_z',
                      'gyro_x', 'gyro_y', 'gyro_z', 'pitch', 'yaw', 'roll']
            writer.writerow(header)

        self.created = True

    def add_to_stream(self, ts_start, ts_stop, data):
        """ imu
            'acc_x': !float,
            'acc_y': !float,
            'acc_z': !float,
            'gyro_x': !float,
            'gyro_y': !float,
            'gyro_z': !float,
            'pitch': !float,
            'yaw': !float,
            'roll': !float
        """
        # Update csv
        with open(self.dir + '/data_descriptor.csv', 'a', newline='') as f:
            writer = csv.writer(f)
            row = [ts_start, ts_stop, ts_stop - ts_start, data['acc_x'], data['acc_y'], data['acc_z'],
                   data['gyro_x'], data['gyro_y'], data['gyro_z'], data['pitch'], data['yaw'], data['roll']]
            writer.writerow(row)


class RovisUltrasonicStream(RovisDataBase.RovisStream):
    def __init__(self, db_path, filter_id, filter_name, input_sources=()):
        super().__init__(db_path, filter_id, filter_name, input_sources)
        self.filter_type = RovisFilterType.ROVIS_ULTRASONICS_FILTER_TYPE
        self.output_type = RovisDataType.ROVIS_ULTRASONICS

    def create_stream(self):
        # Create dir
        mkdir(self.dir)

        # create csv file
        with open(self.dir + '/data_descriptor.csv', 'w', newline='') as f:
            writer = csv.writer(f)
            header = ['timestamp_start', 'timestamp_stop', 'sampling_time', 'sonic_0', 'sonic_1', 'sonic_2',
                      'sonic_3', 'sonic_4', 'sonic_5', 'sonic_6', 'sonic_7', 'sonic_8', 'sonic_9', 'sonic_10',
                      'sonic_11', 'sonic_12', 'sonic_13', 'sonic_14', 'sonic_15', 'sonic_16', 'sonic_17', 'sonic_18',
                      'sonic_19', 'sonic_20', 'sonic_21', 'sonic_22', 'sonic_23', 'sonic_24', 'sonic_25', 'sonic_26',
                      'sonic_27', 'sonic_28', 'sonic_29', 'sonic_30', 'sonic_31', 'sonic_32', 'sonic_33']
            writer.writerow(header)

        self.created = True

    def add_to_stream(self, ts_start, ts_stop, data):
        """ ultrasonics
            'sonics': !array[34 float]
        """
        # Update csv
        with open(self.dir + '/data_descriptor.csv', 'a', newline='') as f:
            writer = csv.writer(f)
            row = [ts_start, ts_stop, ts_stop - ts_start] + data['sonics']
            writer.writerow(row)


class RovisLaneStream(RovisDataBase.RovisStream):
    def __init__(self, db_path, filter_id, filter_name, input_sources=()):
        super().__init__(db_path, filter_id, filter_name, input_sources)
        self.filter_type = RovisFilterType.ROVIS_LANE_DETECTION_FILTER_TYPE
        self.output_type = RovisDataType.ROVIS_LANES_MODEL

    def create_stream(self):
        # create dir and image folders
        mkdir(self.dir)
        mkdir(self.dir + '/samples')
        mkdir(self.dir + '/samples/0')
        mkdir(self.dir + '/samples/0/left')  # for semantic
        mkdir(self.dir + '/samples/1')
        mkdir(self.dir + '/samples/1/left')  # for instance

        # create data_descriptor
        with open(self.dir + '/data_descriptor.csv', 'w', newline='') as f:
            writer = csv.writer(f)
            header = ['timestamp_start', 'timestamp_stop', 'sampling_time', 'semantic', 'instances']
            writer.writerow(header)

        # create framebased_data_descriptor
        with open(self.dir + '/framebased_data_descriptor.csv', 'w', newline='') as f:
            writer = csv.writer(f)
            header = ['timestamp_stop', 'lane_id', 'points', 'theta_0', 'theta_1', 'theta_2', 'theta_3']
            writer.writerow(header)

        self.created = True

    def add_to_stream(self, ts_start, ts_stop, data):
        """ lanes
            'name': !str,
            'semantic': !array[][] / str,
            'instances': !array[][] / str,
            'lane_id': !int,
            'points': !array[],
            'theta_0': !float,
            'theta_1': !float,
            'theta_2': !float,
            'theta_3': !float
        """
        # Save semantic image
        semantic_path = 'samples/0/left' + '/' + data['name']
        if isinstance(data['semantic'], str):
            copy(data['semantic'], self.dir + '/' + semantic_path)
        else:
            cv2.imwrite(self.dir + '/' + semantic_path, data['semantic'])

        # Save instance image
        instances_path = 'samples/1/left' + '/' + data['name']
        if isinstance(data['instances'], str):
            copy(data['instances'], self.dir + '/' + instances_path)
        else:
            cv2.imwrite(self.dir + '/' + instances_path, data['instances'])

        # Update data_descriptor
        with open(self.dir + '/data_descriptor.csv', 'a', newline='') as f:
            writer = csv.writer(f)
            row = [ts_start, ts_stop, ts_stop - ts_start, semantic_path, instances_path]
            writer.writerow(row)

        # Update framebased_data_descriptor
        with open(self.dir + '/framebased_data_descriptor.csv', 'a', newline='') as f:
            writer = csv.writer(f)
            row = [ts_stop, data['lane_id'], str(data['points']).replace(",", ""),
                   data['theta_0'], data['theta_1'], data['theta_2'], data['theta_3']]
            writer.writerow(row)


class RovisSemSegStream(RovisDataBase.RovisStream):
    def __init__(self, db_path, filter_id, filter_name, input_sources=()):
        super().__init__(db_path, filter_id, filter_name, input_sources)
        self.filter_type = RovisFilterType.ROVIS_SEMANTIC_SEGMENTATION_FILTER_TYPE
        self.output_type = RovisDataType.ROVIS_IMAGE

    def create_stream(self):
        # create dir and image folders
        mkdir(self.dir)
        mkdir(self.dir + '/samples')
        mkdir(self.dir + '/samples/0')
        mkdir(self.dir + '/samples/0/left')  # for semantic
        mkdir(self.dir + '/samples/1')
        mkdir(self.dir + '/samples/1/left')  # for instance

        # create data_descriptor
        with open(self.dir + '/data_descriptor.csv', 'w', newline='') as f:
            writer = csv.writer(f)
            header = ['timestamp_start', 'timestamp_stop', 'sampling_time', 'semantic', 'instances']
            writer.writerow(header)

        # create framebased_data_descriptor
        with open(self.dir + '/framebased_data_descriptor.csv', 'w', newline='') as f:
            writer = csv.writer(f)
            header = ['timestamp_stop', 'shape_id', 'cls', 'instance', 'points']
            writer.writerow(header)

        self.created = True

    def add_to_stream(self, ts_start, ts_stop, data):
        """ sem_seg
            'name': !str,
            'semantic': array[][] / str,
            'instances': array[][] / str,
            'shape_id': [int],
            'cls': [int],
            'instance': [int],
            'points': array[][][2 float]
        """
        # Save semantic image
        if 'semantic' in data:
            semantic_path = 'samples/0/left' + '/' + data['name']
            if isinstance(data['semantic'], str):
                copy(data['semantic'], self.dir + '/' + semantic_path)
            else:
                cv2.imwrite(self.dir + '/' + semantic_path, data['semantic'])
        else:
            semantic_path = ''

        # Save instance image
        if 'instances' in data:
            instances_path = 'samples/1/left' + '/' + data['name']
            if isinstance(data['instances'], str):
                copy(data['instances'], self.dir + '/' + instances_path)
            else:
                cv2.imwrite(self.dir + '/' + instances_path, data['instances'])
        else:
            instances_path = ''

        # Update data_descriptor
        if 'semantic' in data or 'instances' in data:
            with open(self.dir + '/data_descriptor.csv', 'a', newline='') as f:
                writer = csv.writer(f)
                row = [ts_start, ts_stop, ts_stop - ts_start, semantic_path, instances_path]
                writer.writerow(row)

        # Update framebased_data_descriptor
        if not ('shape_id' in data and 'cls' in data and 'instance' in data and 'points' in data):
            return

        if not len(data['shape_id']):
            return

        with open(self.dir + '/framebased_data_descriptor.csv', 'a', newline='') as f:
            writer = csv.writer(f)
            for i in range(len(data['shape_id'])):
                row = [ts_stop, data['shape_id'][i], data['cls'][i],
                       data['instance'][i], data['points'][i]]
                writer.writerow(row)


class RovisWheelsEncoderStream(RovisDataBase.RovisStream):
    def __init__(self, db_path, filter_id, filter_name, input_sources=()):
        super().__init__(db_path, filter_id, filter_name, input_sources)
        self.filter_type = RovisFilterType.ROVIS_ROTARY_ENCODER_FILTER_TYPE
        self.output_type = RovisDataType.ROVIS_VECTOR_INT  # TODO check

    def create_stream(self):
        # Create dir
        mkdir(self.dir)

        # create csv file
        with open(self.dir + '/data_descriptor.csv', 'w', newline='') as f:
            writer = csv.writer(f)
            header = ['timestamp_start', 'timestamp_stop', 'sampling_time', 'data_0', 'data_1']
            writer.writerow(header)

        self.created = True

    def add_to_stream(self, ts_start, ts_stop, data):
        """ wheels_encoder
            'data_0': !int,
            'data_1': !int
        """
        # Update csv
        with open(self.dir + '/data_descriptor.csv', 'a', newline='') as f:
            writer = csv.writer(f)
            row = [ts_start, ts_stop, ts_stop - ts_start, data['data_0'], data['data_1']]
            writer.writerow(row)
