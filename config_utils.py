import os
import io
import libconf

__all__ = [
    'CARLA_PATH', 'General', 'Clients', 'config_setup'
]

# Global variables - will be used by all the scripts
CARLA_PATH = "C:/dev/Carla/CARLA_0.9.13"  # Carla install path

# Configuration of the simulator
General = {}
Clients = {}
Database = {}


def read_config(conf_path):
    with io.open(conf_path) as f:
        conf_file = libconf.load(f)

    return conf_file


def check_config():
    def check_key(d, k, val_type):
        if k in d.keys():
            if type(d[k]) != val_type:
                return 1  # Elem is not the right type
        else:
            return 2  # Elem does not exist
        return 0  # OK

    # Define keys and types
    general_key_type = {
        'carla_ip': str, 'carla_port': int, 'res_x': int, 'res_y': int,
        'fps': int, 'quality': str, 'rovis_ip': str, 'town': str, 'populate_sim': bool,
        'population': tuple, 'sync_mode': bool
    }
    client_key_type = {  # Each client
        'position': tuple, 'orientation': tuple, 'random_spawn': bool,
        'actor_type': str, 'control': str, 'cam_width': int,
        'cam_height': int, 'cam_fov': int, 'Comm': libconf.AttrDict
    }
    comm_key_type = {
        'type': str, 'pos': str, 'width': int, 'height': int,
        'fov': int, 'view': bool, 'port': int, 'send': bool,
        'gamma': int, 'range': int, 'channels': int, 'pps': int,
        'rot_freq': int, 'h_fov': int, 'v_fov': int, 'vel_range': int,
        'lower_fov': int, 'upper_fov': int, 'filter': str,
        'cls': tuple
    }
    req_comm_keys = ['type', 'pos']
    db_key_type = {
        'database_path': str, 'save_data': bool, 'sampling_time': int,
        'Datastreams': tuple, 'samples': int
    }
    # Copy from sensor_lib.sensors.__init__.py
    sensor_types = ['camera', 'semseg', 'depth', 'lidar', 'radar', 'imu',
                    'gnss', 'oflow', 'det2d', 'det3d', 'veh_state', 'actuator']

    # Check General keys
    for key in general_key_type:
        ret = check_key(General, key, general_key_type[key])
        if ret == 1:
            print(' - Conf check: General[{}] is not the right type: {}.'.
                  format(key, str(general_key_type[key])))
            return False
        elif ret == 2:
            print(' - Conf check: General does not contain the following key: {}.'.
                  format(key))
            return False

    # Check Clients keys
    if len(Clients) == 0:
        print(' - Conf check: No client in \'Clients\'. Must have at least one.')
        return False
    for cl_key in Clients.keys():
        for key in client_key_type:
            ret = check_key(Clients[cl_key], key, client_key_type[key])
            if ret == 1:
                print(' - Conf check: Client {} - {} is not the right type: {}.'.
                      format(cl_key, key, str(client_key_type[key])))
                return False
            elif ret == 2:
                print(' - Conf check: Client {} does not contain the following key: {}.'.
                      format(cl_key, key))
                return False

    # Check clients comm keys
    for cl_key in Clients.keys():
        for comm_key in Clients[cl_key].Comm.keys():
            for sensor_key in Clients[cl_key].Comm[comm_key]:
                if sensor_key not in comm_key_type:
                    print(' - Conf check: {} - {}: field {} is not a valid field.'.format(cl_key, comm_key, sensor_key))
                    return False
                val = Clients[cl_key].Comm[comm_key][sensor_key]
                if type(val) != comm_key_type[sensor_key]:
                    print(' - Conf check: {} - {}: field {} has type {} instead of {}'.
                          format(cl_key, comm_key, sensor_key, type(val), comm_key_type[sensor_key]))
                    return False

    # Additional Clients checks
    pygame_client = ''
    used_ports = []
    for cl_key in Clients.keys():
        has_actuator = False
        for comm_key in Clients[cl_key].Comm.keys():
            sensor = Clients[cl_key].Comm[comm_key]
            if not all(k in sensor.keys() for k in req_comm_keys):
                print(' - Conf check: {} - {}: Missing required fields from config: {}.'.
                      format(cl_key, comm_key, ','.
                             join([elem for elem in req_comm_keys if elem not in sensor.keys()])))
                return False
            if sensor['type'] not in sensor_types:
                print(' - Conf check: {} - {}: Sensor type {} is invalid.'.
                      format(cl_key, comm_key, sensor['type']))
                return False
            if sensor['type'] == "actuator":
                if has_actuator:
                    print(' - Conf check: {} - {}: One client can have only one Actuator.'.
                          format(cl_key, comm_key))
                    return False
                if "port" not in sensor.keys():
                    print(' - Conf check: {} - {}: Required field \'port\' for receiving data..'.
                          format(cl_key, comm_key))
                    return False
                if sensor['port'] <= 0:
                    print(' - Conf check: {} - {}: Port must be a positive integer.'.
                          format(cl_key, comm_key))
                    return False
                if sensor['port'] in used_ports:
                    print(' - Conf check: {} - {}: Port already in use: {}.'.
                          format(cl_key, comm_key, sensor['port']))
                    return False
                used_ports.append(sensor['port'])
                has_actuator = True
            if "send" in sensor.keys() and sensor['type'] != 'actuator':
                if sensor['send']:
                    if "port" not in sensor.keys():
                        print(' - Conf check: {} - {}: Required field \'port\' for sending data..'.
                              format(cl_key, comm_key))
                        return False
                    if sensor['port'] <= 0:
                        print(' - Conf check: {} - {}: Port must be a positive integer.'.
                              format(cl_key, comm_key))
                        return False
                    if sensor['port'] in used_ports:
                        print(' - Conf check: {} - {}: Port already in use: {}.'.
                              format(cl_key, comm_key, sensor['port']))
                        return False
                    used_ports.append(sensor['port'])
        if has_actuator and Clients[cl_key]['control'] != 'rovis':
            print(' - Conf check: {}: In order to use Actuators, you must select \'rovis\' control.'.format(cl_key))
            return False
        if not has_actuator and Clients[cl_key]['control'] == 'rovis':
            print(' - Conf check: {}: \'rovis\' control does not work without enabling Actuator filter.'.format(cl_key))
            return False
        if Clients[cl_key].control == 'manual':
            if pygame_client is '':
                pygame_client = cl_key
            else:
                print('Because of Pygame, there can be only one manual controlled vehicle at a time: {} / {}.'.
                      format(pygame_client, cl_key))
                return False

    # Check Database keys
    for db_key in Database.keys():
        ret = check_key(Database, db_key, db_key_type[db_key])
        if ret == 1:
            print(' - Conf check: Database {} is not the right type: {}.'.
                  format(db_key, str(db_key_type[db_key])))
            return False
        elif ret == 2:
            print(' - Conf check: Database {} does not contain the following key: {}.'.
                  format(db_key, db_key))
            return False

    # Check Database datastreams
    for ds in Database['Datastreams']:
        if type(ds['name']) != str:
            print(' - Conf check: Datastream {}: name is not the right type: {}.'.format(ds['name'], 'str'))
            return False
        cl = ds['cl']
        if cl not in Clients.keys():
            if Database['save_data']:
                print(' - Conf check: Datastream {}: Client is not valid.'.format(ds['name']))
                return False
            else:
                print(' - Conf check warning: Datastream {}: Client is not valid.'.format(ds['name']))
        sen = ds['sen']
        if sen not in Clients[cl].Comm.keys():
            if Database['save_data']:
                print(' - Conf check: Datastream {}: Sensor is not valid.'.format(ds['name']))
                return False
            else:
                print(' - Conf check warning: Datastream {}: Sensor is not valid.'.format(ds['name']))
        if 'input' in ds.keys():
            if type(ds['input']) != list:
                if Database['save_data']:
                    print(' - Conf check: Datastream {}: input is not the right type: {}.'.format(ds['name'], list))
                    return False
                else:
                    print(' - Conf check warning: Datastream {}: input is not the right type: {}.'.
                          format(ds['name'], list))
        try:
            sen_type = Clients[cl].Comm[sen].type
        except KeyError:
            pass
        else:
            if sen_type == 'actuator':
                print(' - Conf check: Datastream {}: Cannot send actuator data.'.format(ds['name']))
                return False

    # Recommendations
    if not General['sync_mode']:
        print(' - Conf check: It is recommended to run sync mode.')
    if Database['save_data'] and os.path.isdir(Database['database_path']):
        print(' - Conf check: It is recommended for the database to be in a new folder.')

    return True


def config_setup(conf_path):
    # Read config file
    config = read_config(conf_path)

    # Check main keys
    if not all(k in config.keys() for k in ['General', 'Clients', 'Database']):
        print(' - Conf check: Missing main keys: {}'.
              format(','.join([elem for elem in ['General', 'Clients', 'Database'] if elem not in config.keys()])))
        exit(1)

    # Set global variables
    global General, Clients, Database
    General = config.General
    Clients = config.Clients
    Database = config.Database

    # Check config
    return check_config()


if __name__ == '__main__':
    # Test config file
    config_path = r"configs/config_example.conf"

    if config_setup(config_path):
        print(' # Configuration is valid.')
    else:
        print(' # Problem with the setup of configuration.')
