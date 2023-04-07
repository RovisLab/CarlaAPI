from sensor_lib.sensors.Camera import CameraSensor
from sensor_lib.sensors.SemSeg import SemSegSensor
from sensor_lib.sensors.Depth import DepthSensor
from sensor_lib.sensors.Lidar import LidarSensor
from sensor_lib.sensors.Radar import RadarSensor
from sensor_lib.sensors.Imu import ImuSensor
from sensor_lib.sensors.Gnss import GnssSensor
from sensor_lib.sensors.OpticalFlow import OpticalFlowSensor
from sensor_lib.sensors.Bbox2D import Bbox2DSensor
from sensor_lib.sensors.Bbox3D import Bbox3DSensor
from sensor_lib.sensors.VehicleState import VehicleStateEstimation
from sensor_lib.sensors.Actuator import ActuatorControl

__all__ = [
    'sensor_types',

    'CameraSensor',
    'SemSegSensor',
    'DepthSensor',
    'LidarSensor',
    'RadarSensor',
    'ImuSensor',
    'GnssSensor',
    'OpticalFlowSensor',
    'Bbox2DSensor',
    'Bbox3DSensor',
    'VehicleStateEstimation',
    'ActuatorControl'
]

sensor_types = {
    'camera': CameraSensor,
    'semseg': SemSegSensor,
    'depth': DepthSensor,
    'lidar': LidarSensor,
    'radar': RadarSensor,
    'imu': ImuSensor,
    'gnss': GnssSensor,
    'oflow': OpticalFlowSensor,
    'det2d': Bbox2DSensor,
    'det3d': Bbox3DSensor,
    'veh_state': VehicleStateEstimation,
    'actuator': ActuatorControl
}
