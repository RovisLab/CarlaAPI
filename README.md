# CarlaAPI for Rovis

Carla Simulator for Rovis

## Download Carla:

The latest Carla game engine release can be downloaded directly from the link below:

[Download CARLA](https://github.com/carla-simulator/carla/releases)

Copy the carla .egg file from 'Carla_root/PythonAPI/carla/dist' into 'CarlaAPI/dist'

## Other dependencies:

Python version: according to the .egg file distribution.

Install python dependecy packages using the requirements.txt file.

## Configuring Carla:

First step is writing the path toward the Carla exe. Edit the CARLA_EXE_PATH variable in "Run_CarlaClients.py".

## Running Carla:

Carla is executed by running the "Run_CarlaClients.py" script.

## Others:

If you want to populate the simulation with pedestrians and vehicles, run in parallel "spawn_npc.py".

To run using the Pygame control, modify line 42 of "Run_CarlaClients.py" from CarlaClient to CarlaClientPygame. 
In this way, you can write for the control param: 'manual'.

## Possible errors:

#### `ModuleNotFoundError: No module named 'carla'` 

.egg file compiled for another python version or not found
