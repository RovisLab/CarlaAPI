"""
Define global configuration parameters used across the project.
Both RovisAI and RovisVision must be placed in the same folder:
    e.g.: c:/dev/src/RovisLab

Users can get the global config by calling:
    from config import cfg

This script was modified to be used inside CarlaAPI.
"""
import numpy as np
from easydict import EasyDict as edict
import os

__C = edict()

cfg = __C
__C.BASE = edict()
__C.BASE.PATH = os.getcwd()  # <>/<>/CarlaAPI
__C.ROVIS_VISION = edict()

# Look for RovisTypes
found_RovisTypes = False
if "ROVISLAB_DIR" in os.environ:
    RovisTypes_path = os.environ['ROVISLAB_DIR'] + r'/RovisVision/src/rovis_toolkit/src/ROVIS_TYPES.h'
    if os.path.exists(RovisTypes_path):
        __C.ROVIS_VISION.TYPES_FILE = RovisTypes_path
        found_RovisTypes = True
if not found_RovisTypes:
    __C.ROVIS_VISION.TYPES_FILE = __C.BASE.PATH + r'/depend/ROVIS_TYPES.h'

__C.ROVIS_VISION.CONF_FILE = __C.BASE.PATH + r'/depend/config.json'
