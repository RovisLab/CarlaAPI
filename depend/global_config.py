"""
Define global configuration parameters used across the project.
Both RovisAI and RovisVision must be placed in the same folder:
    e.g.: c:/dev/src/RovisLab

Users can get the global config by calling:
    from config import cfg
"""
import numpy as np
from easydict import EasyDict as edict
import os

__C = edict()

cfg = __C
__C.BASE = edict()
# RovisLab path
# RovisAI and RovisVision must be placed in the same folder
try:
    __C.BASE.PATH = os.environ['ROVISLAB_DIR']
except:
    print("Global config: ROVISLAB_DIR is undefined. Using default path \"C:/dev/src/RovisLab\".")
    __C.BASE.PATH = r'C:/dev/src/RovisLab'

# ===================================
# Set ROVIS VISION parameters
__C.ROVIS_VISION = edict()
__C.ROVIS_VISION.TYPES_FILE = __C.BASE.PATH + r'/RovisVision/src/rovis_toolkit/src/ROVIS_TYPES.h'
__C.ROVIS_VISION.PIPELINE_FILE = __C.BASE.PATH + r'/RovisVision/etc/pipelines/control/rovis_nuscenes_pipeline.conf'
__C.ROVIS_VISION.DNN_MODELS_FOLDER = __C.BASE.PATH + r'/RovisVision/models/'
# __C.ROVIS_VISION.OBJECT_CLASSES_PATH = __C.BASE.PATH + r'/RovisDojo/etc/env/object_classes_3.conf'
__C.ROVIS_VISION.OBJECT_CLASSES_PATH = __C.BASE.PATH + r'/RovisDojo/etc/env/object_classes_carla_13.conf'
#__C.ROVIS_VISION.OBJECT_CLASSES_PATH = __C.BASE.PATH + r'/RovisVision/etc/env/object_classes_coco.conf'
#__C.ROVIS_VISION.SEMSEG_CLASSES_PATH = __C.BASE.PATH + r'/RovisVision/etc/env/object_classes_camvid.conf'
#__C.ROVIS_VISION.OBJECT_CLASSES_PATH = __C.BASE.PATH + r'/RovisVision/etc/env/object_classes_road_driving_objdet.conf'
#__C.ROVIS_VISION.OBJECT_CLASSES_PATH = __C.BASE.PATH + r'/RovisVision/etc/env/object_classes_taco.conf'

# Database configuration parameters
# ===================================
# Set database path
__C.DB = edict()
__C.DB.BASE_PATH = __C.BASE.PATH + r'/RovisDojo/data/fake_dataset/' # default test dataset
#__C.DB.BASE_PATH = r'C:/data/scout_data/rovis_test_3' # default test dataset
#__C.DB.BASE_PATH = r'C:\data\RovisDatabases\Scout\03_institut_outside\01'
#__C.DB.BASE_PATH = r'D:\Datasets\rovis_test_drumul_vechi_3'
# __C.DB.BASE_PATH = r'D:/Datasets/Road2Rovis/2014-06-25-16-45-34_stereo_centre_02'
#__C.DB.BASE_PATH = r'C:\data\RovisDatabases\FaceDet\02'
#__C.DB.BASE_PATH = r'C:\data\RovisDatabases\NuScenes\scene-0103'
#__C.DB.BASE_PATH = r'C:\data\RovisDatabases\KITTI\2011_09_26_drive_0005'
#__C.DB.BASE_PATH = r'D:\Datasets\semseg'
