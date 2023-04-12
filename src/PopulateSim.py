import glob
import os
import sys
import time
import numpy as np

import config_utils as conf
try:
    sys.path.append(glob.glob('%s/PythonAPI/carla/dist/carla-*%d.%d-%s.egg' % (
        conf.CARLA_PATH,
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass
import carla


# Global vars
VEH_FILTER = 'vehicle.*'
PED_FILTER = 'walker.pedestrian.*'
TM_PORT = 8000  # Traffic manager port


class PopulateSim(object):
    def __init__(self, sim_client):
        self.sim_client = sim_client
        self.world = sim_client.get_world()
        self.traffic_manager = None

        self.veh_num = conf.General['population'][0] if conf.General['population'][0] >= 0 else 100
        self.ped_num = conf.General['population'][1] if conf.General['population'][1] >= 0 else 50

        self.vehicles = []
        self.walkers = []

        self.all_ids = []  # ped + controllers
        self.all_actors = []  # ped + controllers

    def start(self):
        # Traffic manager
        self.traffic_manager = self.sim_client.get_trafficmanager(TM_PORT)
        self.traffic_manager.set_global_distance_to_leading_vehicle(2.5)
        self.traffic_manager.set_synchronous_mode(conf.General['sync_mode'])
        if not conf.General['sync_mode']:
            print(' - It is recommended to run sync_mode with traffic manager.')

        # Spawn vehicles ===========================================
        veh_spawn_pts = self.world.get_map().get_spawn_points()
        number_of_spawn_points = len(veh_spawn_pts)

        final_veh_num = self.veh_num
        if final_veh_num < number_of_spawn_points:
            np.random.shuffle(veh_spawn_pts)
        elif args.number_of_vehicles > number_of_spawn_points:
            print('requested {} vehicles, but could only find {} spawn points')
            final_veh_num = number_of_spawn_points

        # Find blueprints
        veh_bps = self.world.get_blueprint_library().filter(VEH_FILTER)

        veh_batch = []
        for veh_id in range(final_veh_num):
            bp = np.random.choice(veh_bps)

            if bp.has_attribute('color'):
                color = np.random.choice(bp.get_attribute('color').recommended_values)
                bp.set_attribute('color', color)
            if bp.has_attribute('driver_id'):
                driver_id = np.random.choice(bp.get_attribute('driver_id').recommended_values)
                bp.set_attribute('driver_id', driver_id)
            bp.set_attribute('role_name', 'autopilot')

            veh_batch.append(
                carla.command.SpawnActor(bp, veh_spawn_pts[veh_id]).then(
                    carla.command.SetAutopilot(
                        carla.command.FutureActor, True,
                        self.traffic_manager.get_port()))
            )

        for ret in self.sim_client.apply_batch_sync(veh_batch, conf.General['sync_mode']):
            if not ret.error:
                self.vehicles.append(ret.actor_id)

        # Spawn pedestrians ===========================================
        ped_bps = self.world.get_blueprint_library().filter(PED_FILTER)

        ped_spawn_loc = []
        ped_batch = []
        ped_speed = []
        for i in range(self.ped_num):
            loc = self.world.get_random_location_from_navigation()
            tries = 20
            while loc is None or loc in ped_spawn_loc:
                loc = self.world.get_random_location_from_navigation()
                tries -= 1
                if tries == 0:
                    break
            ped_spawn_loc.append(loc)

            bp = np.random.choice(ped_bps)
            if bp.has_attribute('is_invincible'):
                bp.set_attribute('is_invincible', 'false')
            if bp.has_attribute('speed'):
                ped_speed.append(bp.get_attribute('speed').recommended_values[1])  # Walks
            else:
                ped_speed.append(0.0)
            ped_batch.append(carla.command.SpawnActor(bp, carla.Transform(loc, carla.Rotation())))

        ped_speed_final = []
        for idx, ret in enumerate(self.sim_client.apply_batch_sync(ped_batch, True)):
            if not ret.error:
                self.walkers.append({"id": ret.actor_id})
                ped_speed_final.append(ped_speed[idx])

        # Spawn pedestrian controllers ===========================================
        con_batch = []
        con_bp = self.world.get_blueprint_library().find('controller.ai.walker')

        for i in range(len(self.walkers)):
            con_batch.append(carla.command.SpawnActor(con_bp, carla.Transform(), self.walkers[i]['id']))

        for idx, ret in enumerate(self.sim_client.apply_batch_sync(con_batch, True)):
            if not ret.error:
                self.walkers[idx]["con"] = ret.actor_id

        # Start pedestrians and controllers ===========================================
        for i in range(len(self.walkers)):
            self.all_ids.append(self.walkers[i]["con"])
            self.all_ids.append(self.walkers[i]["id"])
        self.all_actors = self.world.get_actors(self.all_ids)

        # wait for a tick to ensure client receives the last transform of the walkers we have just created
        if conf.General['sync_mode']:
            self.world.tick()
        else:
            self.world.wait_for_tick()

        for i in range(0, len(self.all_ids), 2):
            # start walker
            self.all_actors[i].start()
            # set walk to random point
            self.all_actors[i].go_to_location(self.world.get_random_location_from_navigation())
            # max speed
            self.all_actors[i].set_max_speed(float(ped_speed_final[int(i/2)]))

        print(' # Spawned {} vehicles and {} pedestrians.'.format(len(self.vehicles), len(self.walkers)))

    def terminate(self):
        # Destroy vehicles
        self.sim_client.apply_batch([carla.command.DestroyActor(x) for x in self.vehicles])

        # Stop walker controllers (list is [controller, actor, controller, actor ...])
        for i in range(0, len(self.all_ids), 2):
            self.all_actors[i].stop()

        # Destroy pedestrians
        self.sim_client.apply_batch([carla.command.DestroyActor(x) for x in self.all_ids])
        print(' # Population terminated.')

        # Shut down traffic manager
        self.traffic_manager.shut_down()
        time.sleep(0.5)
