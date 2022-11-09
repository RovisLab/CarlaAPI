# CarlaAPI for Rovis

Carla Simulator for Rovis

## Download Carla:

The engine can be downloaded directly from the link below:

[Download CARLA_0.9.9.4.zip](https://carla-releases.s3.eu-west-3.amazonaws.com/Windows/CARLA_0.9.9.4.zip)

Once that is downloaded and extracted, in the 'Carla_root\PythonAPI\carla\dist' folder will be an .egg file. 
This file must be copied and pasted into the root of this project.

## Other dependencies:

Python version: 3.7

Others:
```shell
$ pip install glob2 python-time threaded psutil subprocess.run random2 numpy opencv-python pygame pyenet libconf
```

## Configuring Carla:

First step is writing the path toward the Carla exe. Edit it in "Run_CarlaClients.py" at line 9.

The client will be created at line 42. By modifying the params there, you configure the client.

## Running Carla:

Carla is executed by running the "Run_CarlaClients.py" script.

## Others:

If you want to populate the simulation with pedestrians and vehicles, run in parallel "spawn_npc.py".

To run using the Pygame control, modify line 42 of "Run_CarlaClients.py" from CarlaClient to CarlaClientPygame. 
In this way, you can write for the control param: 'manual'.

