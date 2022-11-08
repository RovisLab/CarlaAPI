# CarlaAPI for Rovis

Custom Ultrasonic Sensor for Carla.

## Modifications for adding Ultrasonic Sensing to Carla Simulator (built from source)

Inside carla_src_us, you will find the modified source files, needed to add ultrasonic sensing.

This has been tested with Carla 0.9.10 that can be downloaded from this [link](https://github.com/carla-simulator/carla/tree/ba812e75a387e97bf48a0c4b6108ac4e0fe5dee0).

Direct download of the Carla 0.9.10 repository: [here](https://github.com/carla-simulator/carla/archive/ba812e75a387e97bf48a0c4b6108ac4e0fe5dee0.zip)

Each of these files needs to be added to the correct folder from your carla source structure:
* LibCarla/source/carla/sensor/SensorRegistry.h
* PythonAPI/carla/source/libcarla/SensorData.cpp
* LibCarla/source/carla/sensor/data/UltrasonicData.h
* LibCarla/source/carla/sensor/data/UltrasonicMeasurement.h
* LibCarla/source/carla/sensor/s11n/UltrasonicSerializer.cpp
* LibCarla/source/carla/sensor/s11n/UltrasonicSerializer.h
* Unreal/CarlaUE4/Plugins/Carla/Source/Carla/Sensor/UltrasonicSensor.h
* Unreal/CarlaUE4/Plugins/Carla/Source/Carla/Sensor/UltrasonicSensor.cpp
* Unreal/CarlaUE4/Content/Carla/Static/Laser/*

After pasting and overwriting these files, run:
* make launch
* make PythonAPI

For further help on building: 
[windows build](https://carla.readthedocs.io/en/latest/build_windows/) / 
[linux build](https://carla.readthedocs.io/en/latest/build_linux/)

After succesfully building Carla, there will be found in Carla_root\PythonAPI\carla\dist an .egg file.
This file must be copied and pasted into Rovis in the root folder.

Make sure Carla is built and converted to an .exe file before further use.