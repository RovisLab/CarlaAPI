# CarlaAPI v2.0

Carla API (Application Programming Interface) for Rovis usage.

## Features:

 - Spawn multiple cars that can handle different sensors and controls.
 - Various control methods: manual, automatic, rovis
 - Save data locally inside a Rovis type database
 - Send data across the network
 - Easy configuration of every feature from the vehicles, sensors and interface

## Download Carla:

The latest Carla game engine release can be downloaded directly from the link below:

[Download CARLA](https://github.com/carla-simulator/carla/releases)

The recommended and tested version is [Carla 0.9.13](https://github.com/carla-simulator/carla/releases/tag/0.9.13).

## Other dependencies:

Install python dependency packages using the `requirements.txt` file.

## Configuring Carla:

First step is editing the `CARLA_PATH` variable inside `config_utils.py` at line `10` with the root of your carla folder.
The folder must contain `CarlaUE4.exe`.

For the actual configuration, you must use a config file. Examples can be found inside `configs` folder.
To see if the config file is valid, you can run `config_utils.py`.

## Running Carla:

Run `Run_Carla.py` with the path towards a valid config file as parameter.

## Stopping Carla:

You can terminate carla by clicking `Esc` in any viewer window. If you want to close only one viewer, you can press `Q` inside it.

If you are running a client that has manual control (using pygame), it is recommended to terminate by pressing `Esc` only from the control window.

If you are saving data and you have set a positive value for the `samples` field, the simulation will terminate itself after it saves the required data.

## Possible errors:

### `ModuleNotFoundError: No module named 'carla'`
.egg file compiled for another python version or not found. Check `CARLA_PATH` variable.
The module is implemented by using an egg file that can be found inside `<CARLA_PATH>/PythonAPI/carla/dist`. 
Make sure it is present.

### `RuntimeError: time-out of ()ms while waiting for the simulator`
If the Simulator is still running, close it from Task manager and then try again. 
If it is not, check Task Manager for possible processes related to `CarlaUE4.exe` and terminate them.
