import carla
import random
from my_utils import *
import time
import constants as const
from states import State
import math
from typing import List
import traceback
import numpy as np
import copy
import json
import os
from cluster import my_umap, load_vectors
import pandas as pd
from scipy.spatial.distance import cdist, pdist

class Scenario(object):
    def __init__(self, carla_client: carla.Client, state: State, args, ads=const.APOLLO):
        self.ads_type = ads
        self.client = carla_client
        self.world = self.client.get_world()
        self.blueprint_library = self.world.get_blueprint_library()
        self.state = state
        self.seed = round(time.time())
        self.map = self.world.get_map()
        self.ego_sp = carla.Transform()
        self.ego_dp = carla.Transform()
        self.weather = carla.WeatherParameters()
        self.npc_list: List[NPC] = []
        self.score = 0
        self.mutation_history: List[Mutation] = []
        self.is_corner_case = False
        self.selected_count = 0
        self.actor_traj_list: List[ActorTrajectory] = []
        self.umap_corner_case = None
        self.diameter_umap_corner_case = 1
        self.umap_distance_ratio = 0
        self.output_dir = args.output
        self.vector = None

    def dump(self) -> dict:
        ret = {
            "seed": self.seed,
            "map": self.map.name,
            "sp": get_dict_transform(self.ego_sp),
            "dp": get_dict_transform(self.ego_dp),
            "weather": {
                "cloud": self.weather.cloudiness,
                "precipitation": self.weather.precipitation,
                "precipitation_deposits": self.weather.precipitation_deposits,
                "wind": self.weather.wind_intensity,
                "fog": self.weather.fog_density,
                "wetness": self.weather.wetness,
                "sun_azimuth_angle": self.weather.sun_azimuth_angle,
                "sun_altitude_angle": self.weather.sun_altitude_angle,
            },
            "npc_list": [
                {
                    "type": npc.type,
                    "nav_type": npc.nav_type,
                    "blueprint": npc.blueprint.id,
                    "speed": npc.speed,
                    "sp": get_dict_transform(npc.sp),
                    "dp": get_dict_transform(npc.dp),
                }
                for npc in self.npc_list
            ],
            "score": self.score,
            "collision": 1 if self.state.collision_event is not None else 0,
            "collision_actor": self.state.collision_event.other_actor.type_id if self.state.collision_event is not None else "None",
            "stuck": 1 if self.state.stuck else 0,
            "vector": self.vector.tolist()
        }
        ret["trajectory"] = []
        for t in self.actor_traj_list:
            ret["trajectory"].append(t.traj_to_list())
        return ret

    def load(self, scenario_dict: dict):
        self.seed = round(scenario_dict["seed"])
        current_map = self.world.get_map()
        if scenario_dict["map"].lower() == current_map.name.lower():
            self.map = self.world.get_map()
        else:
            self.map = self.world.get_map()  # TODO: change map
        self.ego_sp = get_carla_transform(scenario_dict["sp"])
        self.ego_dp = get_carla_transform(scenario_dict["dp"])
        self.weather.cloudiness = scenario_dict["weather"]["cloud"]
        self.weather.precipitation = scenario_dict["weather"]["precipitation"]
        self.weather.precipitation_deposits = scenario_dict["weather"][
            "precipitation_deposits"
        ]
        self.weather.wind_intensity = scenario_dict["weather"]["wind"]
        self.weather.fog_density = scenario_dict["weather"]["fog"]
        self.weather.wetness = scenario_dict["weather"]["wetness"]
        self.weather.sun_azimuth_angle = scenario_dict["weather"]["sun_azimuth_angle"]
        self.weather.sun_altitude_angle = scenario_dict["weather"]["sun_altitude_angle"]

        self.npc_list = []
        for npc_dict in scenario_dict["npc_list"]:
            npc = NPC()
            npc.type = npc_dict["type"]
            npc.nav_type = npc_dict["nav_type"]
            npc.blueprint = self.blueprint_library.find(npc_dict["blueprint"])
            npc.speed = npc_dict["speed"]
            npc.sp = get_carla_transform(npc_dict["sp"])
            npc.dp = get_carla_transform(npc_dict["dp"])
            # npc.bbox = npc.get_bbox(npc.bounding_box.extent)
            self.npc_list.append(npc)

    def is_valid_spawn_point(self, new_sp: carla.Transform) -> bool:
        distance_from_ego = new_sp.location.distance(self.ego_sp.location)
        if (
            distance_from_ego < const.MIN_DIST_FROM_EGO
            or distance_from_ego > const.MAX_DIST_FROM_EGO
        ):
            return False

        for npc in self.npc_list:
            if new_sp.location.distance(npc.sp.location) < const.MIN_DIST_BETWEEN_NPC:
                return False

        return True

    def add_random_npc(self, type=const.VEHICLE) -> Mutation:
        spawn_points = self.map.get_spawn_points()
        npc = NPC()
        npc.sp = random.choice(spawn_points)
        while not self.is_valid_spawn_point(npc.sp):
            npc.sp = random.choice(spawn_points)
        npc.dp = self.ego_dp

        if type == const.VEHICLE:
            npc.type = type
            npc.nav_type = const.AUTOPILOT
            npc.blueprint = random.choice(self.blueprint_library.filter("vehicle.*"))
            # npc.blueprint = random.choice(self.blueprint_library.filter("vehicle.*"))
            npc.speed = random.uniform(const.VEHICLE_MIN_SPEED, const.VEHICLE_MAX_SPEED)

        self.npc_list.append(npc)
        m = Mutation()
        m.type = const.MUT_NPC_ADD
        m.new_value = npc
        m.index = len(self.npc_list) - 1
        self.mutation_history.append(m)
        return m

    def add_npc(self, npc: NPC) -> Mutation:
        self.npc_list.append(npc)
        m = Mutation()
        m.type = const.MUT_NPC_ADD
        m.new_value = npc
        m.index = len(self.npc_list) - 1
        self.mutation_history.append(m)
        return m

    def update_umap(self, dir):
        vectors, case_labels = load_vectors(os.path.join(os.path.dirname((os.path.abspath(__file__))), dir))
        
        if len(case_labels) > 15 and (case_labels.count("collision") + case_labels.count("stuck") >= 2):
            embedding, self.umap_model = my_umap(vectors)
            df_umap = pd.DataFrame(embedding, columns=['UMAP Dimension 1', 'UMAP Dimension 2'])
            df_umap["case_label"] = case_labels
            self.umap_corner_case = df_umap[df_umap["case_label"] != "safe"][["UMAP Dimension 1", "UMAP Dimension 2"]].values
            
            self.diameter_umap_corner_case = np.max(pdist(self.umap_corner_case, metric='euclidean'))
    
    def umap_distance_to_corner_case(self):
        self.vector = self.get_vector()
        if self.diameter_umap_corner_case == 1:
            # no enough corner case for analysis
            self.umap_distance_ratio = 0
        else:
            new_embedding = self.umap_model.transform(self.vector.reshape(1, -1))
            avg_umap_distance = np.mean(cdist(new_embedding, self.umap_corner_case, metric='euclidean')[0])
            self.umap_distance_ratio = min(1, avg_umap_distance / self.diameter_umap_corner_case)
        
    def objective_function(self, total_sim, w=const.W) -> float:
        # range: [0,1]
        # greater function value -> more dangerous and more diverse scenario
        
        if self.state.num_frames == 0:
            self.score = 0
        else:
            d_min = max(self.state.min_dist - 2.5, 0.05)
            sigmoid_d_inverse = 1 / (1 + math.exp(-1/d_min))
            
            self.score = \
                w[0] * sigmoid_d_inverse \
                + w[1] *  max(0, self.state.route_distace_to_dst / max(5,self.state.route_distance_sp_to_dp))\
                + w[2] * (1 - self.selected_count/total_sim) \
                + w[3] * self.umap_distance_ratio
            
        return self.score

    def print_simulation_result(self):
        print(
            "[*] objective_function_value = {:.2f}, num_frames = {}, change_lane = {}, min_distance = {:.2f}".format(
                self.score,
                self.state.num_frames,
                self.state.change_lane,
                self.state.min_dist,
            )
        )
        print("-"*70 + "\n")

    def mutate_npc_speed(self) -> Mutation:
        m = Mutation()
        m.type = const.MUT_NPC_SPEED
        m.index = random.randint(0, len(self.npc_list) - 1)
        m.old_value = self.npc_list[m.index].speed

        m.new_value = m.old_value + (1 - self.score) * random.uniform(-16, 16)
        m.new_value = min(const.VEHICLE_MAX_SPEED, m.new_value)
        m.new_value = max(const.VEHICLE_MIN_SPEED, m.new_value)

        self.npc_list[m.index].speed = m.new_value
        self.mutation_history.append(m)
        return m

    def mutate_npc_blueprint(self, vehicle_list=None) -> Mutation:
        m = Mutation()
        m.type = const.MUT_NPC_BP
        m.index = random.randint(0, len(self.npc_list) - 1)
        m.old_value = self.npc_list[m.index].blueprint
        if vehicle_list is not None:
            m.new_value = self.blueprint_library.find(random.choice(vehicle_list))
        else:
            vehicle_bps = self.blueprint_library.filter("vehicle.*")
            m.new_value = random.choice(vehicle_bps)
        self.npc_list[m.index].blueprint = m.new_value
        self.mutation_history.append(m)
        return m

    def mutate_npc_dp(self) -> Mutation:
        m = Mutation()
        m.type = const.MUT_NPC_DST
        m.index = random.randint(0, len(self.npc_list) - 1)
        m.old_value = self.npc_list[m.index].dp
        
        spawn_points = self.map.get_spawn_points()
        m.new_value = random.choice(spawn_points)
        while not self.is_valid_spawn_point(m.new_value):
            m.new_value = random.choice(spawn_points)
        self.npc_list[m.index].dp = m.new_value
        self.mutation_history.append(m)
        return m
    
    def mutate_weather(self) -> List[Mutation]:
        m_list = []
        if random.random() < (1-self.score):
            # cloudiness
            m = Mutation()
            m.type = const.MUT_WEATHER_CLOUD
            m.old_value = self.weather.cloudiness
            m.new_value = m.old_value + (1-self.score) * random.uniform(-50, 50)
            m.new_value = min(100, m.new_value)
            m.new_value = max(0, m.new_value)
            self.weather.cloudiness = m.new_value
            m_list.append(m)
        
        if random.random() < (1-self.score):
            # precipitation
            m = Mutation()
            m.type = const.MUT_WEATHER_RAIN
            m.old_value = self.weather.precipitation
            m.new_value = m.old_value + (1-self.score) * random.uniform(-50, 50)
            m.new_value = min(100, m.new_value)
            m.new_value = max(0, m.new_value)
            self.weather.precipitation = m.new_value
            m_list.append(m)
        
        if random.random() < (1-self.score):
            # precipitation deposit
            m = Mutation()
            m.type = const.MUT_WEATHER_PD
            m.old_value = self.weather.precipitation_deposits
            m.new_value = m.old_value + (1-self.score) * random.uniform(-50, 50)
            m.new_value = min(100, m.new_value)
            m.new_value = max(0, m.new_value)
            self.weather.precipitation_deposits = m.new_value
            m_list.append(m)
        
        if random.random() < (1-self.score):
            # wind
            m = Mutation()
            m.type = const.MUT_WEATHER_WIND
            m.old_value = self.weather.wind_intensity
            m.new_value = m.old_value + (1-self.score) * random.uniform(-50, 50)
            m.new_value = min(100, m.new_value)
            m.new_value = max(0, m.new_value)
            self.weather.wind_intensity = m.new_value
            m_list.append(m)
            
        if random.random() < (1-self.score):
            # sun azimuth angle
            m = Mutation()
            m.type = const.MUT_WEATHER_SUN_AZ
            m.old_value = self.weather.sun_azimuth_angle
            m.new_value = m.old_value + (1-self.score) * random.uniform(-180, 180)
            m.new_value = min(360, m.new_value)
            m.new_value = max(0, m.new_value)
            self.weather.sun_azimuth_angle = m.new_value
            m_list.append(m)
        
        if random.random() < (1-self.score):
            # sun altitude angle
            m = Mutation()
            m.type = const.MUT_WEATHER_SUN_AL
            m.old_value = self.weather.sun_altitude_angle
            m.new_value = m.old_value + (1-self.score) * random.uniform(-90, 90)
            m.new_value = min(90, m.new_value)
            m.new_value = max(-90, m.new_value)
            self.weather.sun_altitude_angle = m.new_value
            m_list.append(m)
            
        if random.random() < (1-self.score):
            # wetness
            m = Mutation()
            m.type = const.MUT_WEATHER_WET
            m.old_value = self.weather.wetness
            m.new_value = m.old_value + (1-self.score) * random.uniform(-50, 50)
            m.new_value = min(100, m.new_value)
            m.new_value = max(0, m.new_value)
            self.weather.wetness = m.new_value
            m_list.append(m)

        for m in m_list:
            self.mutation_history.append(m)
        return m_list

    # def mutate(self, score, T=const.INIT_TEMPERATURE, factor=Factor()) -> Mutation:
    #     if factor.rear_end:
    #         npc = NPC()
    #         npc.type = const.VEHICLE
    #         npc.nav_type = const.AUTOPILOT
    #         npc.speed = random.uniform(2, 10)
    #         npc.sp = get_rare_end_npc_sp(self.ego_sp, random.uniform(8, 20))
    #         npc.dp = self.ego_dp
    #         npc.blueprint = random.choice(self.blueprint_library.filter("vehicle.*"))
    #         m = self.add_npc(npc)

    #     elif factor.standing_water:
    #         m = Mutation()
    #         m.type = const.MOD_PD
    #         m.old_value = self.weather.precipitation_deposits
    #         m.new_value = random.uniform(70, 100)
    #         self.weather.precipitation_deposits = m.new_value
    #         self.mutation_history.append(m)

    #     elif factor.small_target:
    #         m = self.mutate_npc_blueprint(const.SMALL_NPC_LIST)

    #     elif factor.large_target:
    #         m = self.mutate_npc_blueprint(const.LARGE_NPC_LIST)

    #     elif factor.slope:
    #         pass

    #     else:
    #         r = random.randint(0, 99)
    #         if r < max(0.3 * T, 10):
    #             m = self.add_random_npc()
    #         elif r < max(0.5 * T, 25):
    #             m = self.mutate_npc_blueprint()
    #         elif r < max(0.9 * T, 60):
    #             m = self.mutate_npc_speed(T)
    #         else:
    #             m = self.mutate_weather(T)

    #     return m
    
    def mutate_npc(self) -> Mutation:
        r = random.random()
        if r < 0.6 * self.score:
            m = self.mutate_npc_dp()
        elif r < self.score:
            m = self.mutate_npc_blueprint()
        else:
            m = self.mutate_npc_speed()
        return m
    
    def mutate(self) -> List[Mutation]:
        m_list = []
        if random.random() < (1 - 0.8*self.score) and len(self.npc_list) < 3:
            m = self.add_random_npc()
        else:
            m = self.mutate_npc()
        m_list.append(m)
        
        m_list += self.mutate_weather()
        return m_list
    
    def rollback(self, m: Mutation):
        if m.type == const.MUT_NPC_SPEED:
            self.npc_list[m.index].speed = m.old_value
        elif m.type == const.MUT_NPC_BP:
            self.npc_list[m.index].blueprint = m.old_value
        elif m.type == const.MUT_WEATHER_PD:
            self.weather.precipitaton_deposits = m.old_value
        elif m.type == const.MUT_WEATHER_SUN_AL:
            self.weather.sun_altitude_angle = m.old_value
        elif m.type == const.MUT_NPC_ADD:
            self.npc_list.pop(m.index)
        else:
            # TODO: error
            pass

        for i in range(len(self.mutation_history)):
            if self.mutation_history[i] is m:
                self.mutation_history.pop(i)

    def run_simulation(self, total_sim, check_route=True):
        self.state.reset()
        self.state.start_point = (
            self.ego_sp.location.x,
            self.ego_sp.location.y,
        )  # Carla coordinate
        self.state.dst_point = (
            self.ego_dp.location.x,
            -self.ego_dp.location.y,
        )  # Apollo coordinate

        retval = const.INIT
        # print("-" * 50)
        try:
            tm = self.client.get_trafficmanager()
            fixed_delta_seconds = self.world.get_settings().fixed_delta_seconds
            self.world.wait_for_tick()
            print("[*] #sim = {}".format(total_sim))
            # set weather
            print(
                "[*] precipitation_deposits = {}, sun_altitude_angle = {}, fog_density = {}".format(
                    self.weather.precipitation_deposits,
                    self.weather.sun_altitude_angle,
                    self.weather.fog_density,
                )
            )
            self.world.set_weather(self.weather)

            sensors = []
            actor_vehicles = []
            actor_walkers = []
            actor_controllers = []

            self.world.wait_for_tick()  # sync once with simulator

            # spawn ego_vehicle
            if self.ads_type == const.APOLLO:
                ego = get_ego_vehicle(self.world)
                ego_light_state = ego.get_light_state()
                ego_light_state |= carla.VehicleLightState.HighBeam
                ego_light_state |= carla.VehicleLightState.Position
                ego_light_state |= carla.VehicleLightState.Fog
                ego.set_light_state(carla.VehicleLightState(ego_light_state))

                ego.set_transform(self.ego_sp)
                while abs(ego.get_velocity().z) > 0.0001:
                    # print(player.get_velocity())
                    self.world.wait_for_tick()
                    time.sleep(1)
                ego.set_transform(self.ego_sp)
                # initialize trajectory of ego
                self.actor_traj_list.append(ActorTrajectory(ego.id))
                self.actor_traj_list[0].append_transform(self.ego_sp, 0)
                
                localization_channel = "/apollo/localization/pose"
                self.state.node.create_reader(
                    localization_channel,
                    LocalizationEstimate,
                    lambda data: check_arrival(data, self.state),
                )

                # clean history route
                # apollo_routing_request(self.state.node, (0, 0, 0), (0, 0, 0), 0)

                # check apollo obstacle detection
                # perception_obstacle_channel = "/apollo/perception/obstacles"
                # self.state.node.create_reader(
                #     perception_obstacle_channel,
                #     PerceptionObstacles,
                #     lambda data: check_obstacle_detection(
                #         data, self.state, self.map, ego
                #     ),
                # )

                # check lane change
                planning_channel = "/apollo/planning"
                self.state.node.create_reader(
                    planning_channel,
                    ADCTrajectory,
                    lambda data: check_planning(data, self.state),
                )
                
        
            # Attach collision detector
            collision_bp = self.blueprint_library.find("sensor.other.collision")
            sensor_collision = self.world.spawn_actor(
                collision_bp, carla.Transform(), attach_to=ego
            )
            sensor_collision.listen(lambda event: on_collision(event, self.state))
            sensors.append(sensor_collision)

            # Attach lane invasion sensor
            # lanesensor_bp = blueprint_library.find("sensor.other.lane_invasion")
            # sensor_lane = world.spawn_actor(lanesensor_bp, carla.Transform(), attach_to=ego)
            # sensor_lane.listen(lambda event: _on_invasion(event, state))
            # sensors.append(sensor_lane)
            
            # Attach Fog LiDAR
            # fake_lidar_bp = self.blueprint_library.find("sensor.lidar.ray_cast_with_fog")
            # fake_lidar_bp.set_attribute('range', '100')
            # fake_lidar_bp.set_attribute('rotation_frequency', str(20))
            # fake_lidar_bp.set_attribute('upper_fov', '2.0')
            # fake_lidar_bp.set_attribute('lower_fov', '-26.8')
            # fake_lidar_bp.set_attribute('points_per_second', '10000')
            # fake_lidar_tf = carla.Transform(carla.Location(z=2.4))
            # fake_lidar = self.world.spawn_actor(fake_lidar_bp, fake_lidar_tf, attach_to=ego)
            # fake_lidar.listen(lambda raw_data: calculate_fog_noise_rate(raw_data, self.state))
            # sensors.append(fake_lidar)

            self.world.wait_for_tick()  # sync with simulator

            # spawn npc actors
            walker_controller_bp = self.blueprint_library.find("controller.ai.walker")
            for npc in self.npc_list:
                if npc.type == const.VEHICLE:
                    actor_vehicle = self.world.try_spawn_actor(npc.blueprint, npc.sp)
                    npc.get_bbox(actor_vehicle.bounding_box.extent)
                    if actor_vehicle is None:
                        print(
                            "[-] Failed spawning {} vehicle at ({}, {})".format(
                                const.NAV_TYPE_NAME_LIST[npc.nav_type],
                                npc.sp.location.x,
                                npc.sp.location.y,
                            )
                        )

                        self.state.spawn_failed = True
                        self.state.spawn_failed_object = npc
                        retval = const.FAIL
                        return retval

                    tm.update_vehicle_lights(actor_vehicle, True)
                    actor_vehicles.append(actor_vehicle)
                    npc.get_bbox(actor_vehicle.bounding_box.extent)
                    self.actor_traj_list.append(ActorTrajectory(actor_vehicle.id))
                    self.actor_traj_list[-1].append_transform(npc.sp, 0)
                    if npc.nav_type == const.AUTOPILOT:
                        print(
                            "[+] New {} vehicle [{}] [{}] [{:.1f} km/h] ({:.2f},{:.2f})->({:.2f},{:.2f})".format(
                                const.NAV_TYPE_NAME_LIST[npc.nav_type],
                                actor_vehicle.id,
                                npc.blueprint.id,
                                npc.speed,
                                npc.sp.location.x,
                                npc.sp.location.y,
                                npc.dp.location.x,
                                npc.dp.location.y,
                            )
                        )
                    else:
                        print(
                            "[+] New {} vehicle [{}] [{}] [{:.1f} km/h] ({:.2f},{:.2f})".format(
                                const.NAV_TYPE_NAME_LIST[npc.nav_type],
                                actor_vehicle.id,
                                npc.blueprint.id,
                                npc.speed,
                                npc.sp.location.x,
                                npc.sp.location.y,
                            )
                        )

                elif npc.type == const.WALKER:  # walker
                    actor_walker = self.world.try_spawn_actor(npc.blueprint, npc.sp)
                    npc.get_bbox(actor_walker.bounding_box.extent)
                    if actor_walker is None:
                        print(
                            "[-] Failed spawning {} walker at ({}, {})".format(
                                const.NAV_TYPE_NAME_LIST[npc.nav_type],
                                npc.sp.location.x,
                                npc.sp.location.y,
                            )
                        )

                        self.state.spawn_failed = True
                        self.state.spawn_failed_object = npc
                        retval = const.FAIL
                        return retval

                    actor_walkers.append(actor_walker)
                    npc.get_bbox(actor_walker.bounding_box.extent)
                    self.actor_traj_list.append(ActorTrajectory(actor_walker.id))
                    self.actor_traj_list[-1].append_transform(npc.sp, 0)
                    if npc.nav_type == const.AUTOPILOT:
                        print(
                            "[+] New {} walker [{}] [{}] [{:.1f} km/h] ({:.2f},{:.2f})->({:.2f},{:.2f})".format(
                                const.NAV_TYPE_NAME_LIST[npc.nav_type],
                                actor_walker.id,
                                npc.blueprint.id,
                                npc.speed,
                                npc.sp.location.x,
                                npc.dp.location.y,
                                npc.dp.location.x,
                                npc.dp.location.y,
                            )
                        )
                    else:
                        print(
                            "[+] New {} walker [{}] [{}] [{:.1f} km/h] ({:.2f},{:.2f},yaw={:.2f})".format(
                                const.NAV_TYPE_NAME_LIST[npc.nav_type],
                                actor_walker.id,
                                npc.blueprint.id,
                                npc.speed,
                                npc.sp.location.x,
                                npc.sp.location.y,
                                npc.sp.rotation.yaw,
                            )
                        )

            if self.ads_type == const.APOLLO:
                # Set Apollo routing
                start_point = (
                    self.ego_sp.location.x,
                    (-1) * float(self.ego_sp.location.y),
                    self.ego_sp.location.z,
                )
                destination_point = (
                    self.ego_dp.location.x,
                    (-1) * float(self.ego_dp.location.y),
                    self.ego_dp.location.z,
                )
                apollo_routing_request(
                    self.state.node,
                    start_point,
                    destination_point,
                    self.ego_sp.rotation.yaw,
                )
                time.sleep(2)  # give some time
                if self.state.route_too_long and check_route:
                    retval = const.ROUTE_TOO_LONG
                    print("[*] Route too long!")
                    return retval
                self.world.wait_for_tick()

            # move npc actors
            self.world.reset_all_traffic_lights()
            cnt_v = 0
            cnt_w = 0
            for npc in self.npc_list:
                if npc.type == const.VEHICLE:
                    actor_vehicle = actor_vehicles[cnt_v]
                    cnt_v += 1
                    if npc.nav_type == const.IMMOBILE:
                        brake_control = carla.VehicleControl(brake=1.0)
                        actor_vehicle.apply_control(brake_control)

                    elif npc.nav_type == const.LINEAR:
                        forward_vec = npc.sp.rotation.get_forward_vector()
                        actor_vehicle.set_target_velocity(forward_vec * npc.speed)

                    elif npc.nav_type == const.AUTOPILOT:
                        # if npc.get("ignore_traffic_light") is not None:
                        #     if npc["ignore_traffic_light"]:
                        #         tm.ignore_lights_percentage(actor_vehicle, 100)
                        # else:
                        #     npc["ignore_traffic_light"] = False
                        tm.set_desired_speed(actor_vehicle, npc.speed)
                        actor_vehicle.set_autopilot(True, tm.get_port())
                        tm.set_path(
                            actor_vehicle,
                            [
                                npc.dp.location,
                            ],
                        )
                    actor_vehicle.set_simulate_physics(True)
                elif npc.type == const.WALKER:
                    actor_walker = actor_walkers[cnt_w]
                    cnt_w += 1
                    if npc.nav_type == const.LINEAR:
                        forward_vec = npc.sp.rotation.get_forward_vector()
                        controller_walker = carla.WalkerControl()
                        controller_walker.direction = forward_vec
                        controller_walker.speed = npc.speed
                        actor_walker.apply_control(controller_walker)

                    elif npc.nav_type == const.AUTOPILOT:
                        controller_walker = self.world.spawn_actor(
                            walker_controller_bp, npc.sp, actor_walker
                        )

                        # world.tick() # without this, walker vanishes
                        self.world.wait_for_tick()
                        controller_walker.start()
                        controller_walker.set_max_speed(float(npc.speed))
                        controller_walker.go_to_location(npc.dp.location)

                    elif npc.nav_type == const.IMMOBILE:
                        controller_walker = None

                    if controller_walker:  # can be None if immobile walker
                        actor_controllers.append(controller_walker)

            try:
                # actual monitoring of the driving simulation begins here
                first_snapshot = self.world.get_snapshot()
                first_frame_id = first_snapshot.frame
                first_sim_time = first_snapshot.timestamp.elapsed_seconds

                last_frame_id = first_frame_id

                self.state.first_frame_id = first_frame_id
                self.state.sim_start_time = first_snapshot.timestamp.platform_timestamp
                self.state.num_frames = 0
                self.state.elapsed_time = 0

                while True:
                    if self.ads_type == const.APOLLO:
                        self.world.wait_for_tick()

                    snapshot = self.world.get_snapshot()
                    cur_frame_id = snapshot.frame
                    cur_sim_time = snapshot.timestamp.elapsed_seconds

                    if cur_frame_id <= last_frame_id:
                        # skip if we got the same frame data as last
                        continue

                    last_frame_id = cur_frame_id  # update last
                    self.state.num_frames = cur_frame_id - first_frame_id
                    self.state.elapsed_time = cur_sim_time - first_sim_time
                    if self.state.num_frames == 5:
                        self.state.route_distance_sp_to_dp = self.state.route_distace_to_dst

                    ego_tf = ego.get_transform()
                    ego_location = ego_tf.location

                    # Get speed
                    speed = calculate_velocity(ego.get_velocity())
                    speed_limit = ego.get_speed_limit()

                    print(
                        "({:.2f},{:.2f})>({:.2f},{:.2f})>({:.2f},{:.2f}) {:.2f} m left, {:.2f}/{} km/h   \r".format(
                            self.ego_sp.location.x,
                            self.ego_sp.location.y,
                            ego_location.x,
                            ego_location.y,
                            self.ego_dp.location.x,
                            self.ego_dp.location.y,
                            self.state.distance_to_dst,
                            speed,
                            speed_limit,
                        ),
                        end="",
                    )
                    
                    if self.state.num_frames % const.RECORD_TIMESTEP_INTERVAL == 0:
                        # record trajectory of ego
                        self.actor_traj_list[0].append_transform(ego_tf, speed)
                        
                        # record trajectory of each NPC vehicle
                        for i in range(len(actor_vehicles)):
                            npc_tf = actor_vehicles[i].get_transform()
                            npc_speed = calculate_velocity(actor_vehicles[i].get_velocity())
                            self.actor_traj_list[1+i].append_transform(npc_tf, npc_speed)
                            
                        # record trajectory of each NPC pedestrian
                        for i in range(len(actor_walkers)):
                            npc_tf = actor_walkers[i].get_transform()
                            npc_speed = calculate_velocity(actor_walkers[i].get_velocity())
                            self.actor_traj_list[1+len(actor_vehicles)+i].append_transform(npc_tf, npc_speed)
                            
                        # debug info
                        # print(self.state.fog_noise_rate)
                        
                    for v in actor_vehicles:
                        dist = ego_location.distance(v.get_location())
                        if dist < self.state.min_dist:
                            self.state.min_dist = dist
                        # if (
                        #     dist > const.FOG_RING_RANGE[0]
                        #     and dist < const.FOG_RING_RANGE[1]
                        # ):
                            # self.state.in_fog_num += 1

                    for w in actor_walkers:
                        dist = ego_location.distance(w.get_location())
                        if dist < self.state.min_dist:
                            self.state.min_dist = dist
                        # if (
                        #     dist > const.FOG_RING_RANGE[0]
                        #     and dist < const.FOG_RING_RANGE[1]
                        # ):
                        #     self.state.in_fog_num += 1

                    if self.ads_type == const.APOLLO:
                        if self.state.arrival:
                            print("\n[*] (Apollo) Reached the destination.")
                            break

                    # Check collision
                    if self.state.collision_event is not None:
                        print(
                            "\n[*] Collision detected: elapsed_time: {:.2f}, collision_actor: [{}]{}".format(
                                self.state.elapsed_time,
                                self.state.collision_event.other_actor.id,
                                self.state.collision_event.other_actor.type_id,
                            )
                        )
                        retval = const.CORNER_CASE
                        self.is_corner_case = True
                        # self.record_corner_case()
                        break

                    # Check stuck
                    if speed < 1:  # km/h
                        self.state.stuck_duration += 1
                    else:
                        self.state.stuck_duration = 0

                    if self.state.stuck_duration >= (
                        const.TIMEOUT_SIM_SEC / fixed_delta_seconds
                    ):
                        self.state.stuck = True
                        self.state.stuck_xy = (ego_location.x, ego_location.y)
                        print(
                            "\n[*] Stuck: {} seconds. Ego location: ({:.2f}, {:.2f})".format(
                                const.TIMEOUT_SIM_SEC,
                                ego_location.x, 
                                ego_location.y
                            )
                        )
                        retval = const.CORNER_CASE
                        self.is_corner_case = True
                        # self.record_corner_case()
                        break

                    # Check apollo obstacles detection
                    # if self.ads_type == const.APOLLO:
                    #     npc_list = actor_vehicles + actor_walkers
                    #     apollo_obs_list = self.state.apollo_obs_list
                    #     apollo_obs_fn, apollo_obs_fp = calculate_false_results(
                    #         apollo_obs_list, npc_list
                    #     )
                    #     self.state.apollo_fn_num += apollo_obs_fn
                    #     self.state.apollo_fp_num += apollo_obs_fp

            except KeyboardInterrupt:
                print("Keyboard Interrupt")
                retval = const.INTERRUPT

        except Exception as e:
            print("[-] Runtime error:")
            traceback.print_exc()
            retval = const.RUNTIME_ERROR

        finally:
            # Finalize simulation
            if retval != const.ROUTE_TOO_LONG:
                self.update_umap(self.output_dir)
                self.umap_distance_to_corner_case()
                self.objective_function(total_sim+1)
                self.record_scenario()
            else:
                self.score = 0
            
            destroy_commands = []
            for v in actor_vehicles:
                # v.set_autopilot(False)
                destroy_commands.append(carla.command.DestroyActor(v))
            for w in actor_walkers:
                destroy_commands.append(carla.command.DestroyActor(w))
            for s in sensors:
                destroy_commands.append(carla.command.DestroyActor(s))
                s.destroy()
            if self.ads_type == const.APOLLO:
                self.client.apply_batch(destroy_commands)
            
            # Don't reload and exit if user requests so
            if retval == const.INTERRUPT:
                return retval

            else:
                if not self.ads_type == const.APOLLO:
                    self.client.reload_world()
                return retval

    def static_conf_to_vector(self) -> List:
        # static configuration includes: 
        #   ego sp and dp, 
        #   weather parameters, 
        #   for each NPC:
        #       sp and dp, 
        #       boudingbox (x,y,z), 
        #       speed.
        static_vector = []
        # 1. ego sp and dp
        static_vector.append(sum_tf(self.ego_sp))
        static_vector.append(sum_tf(self.ego_dp) - static_vector[0])
        
        # 2. weather parameters
        static_vector.append(self.weather.cloudiness)
        static_vector.append(self.weather.precipitation)
        static_vector.append(self.weather.precipitation_deposits)
        static_vector.append(self.weather.wind_intensity)
        static_vector.append(self.weather.sun_azimuth_angle)
        static_vector.append(self.weather.sun_altitude_angle)
        static_vector.append(self.weather.wetness)
        
        # 3. NPC
        npc_list = self.npc_list[:3]
        # if collision, only record the collision npc
        if self.state.collision_event is not None:
            for npc in npc_list:
                if npc.blueprint.id == self.state.collision_event.other_actor.id:
                    npc_list = [npc]
                    break
        
        while len(npc_list) < 3:
            npc_list.append(NPC(speed=0))
        for npc in npc_list:
            if npc.speed == 0:
                static_vector += [0] * 4
            else:
                static_vector.append(sum_tf(npc.sp) - static_vector[0])
                static_vector.append(sum_tf(npc.dp) - static_vector[0])
                static_vector.append(npc.bbox.x + npc.bbox.y + npc.bbox.z)
                static_vector.append(npc.speed)
        
        return static_vector
    
    def ego_traj_to_vector(self) -> List:
        traj = self.actor_traj_list[0] # ego
        length = len(traj.x)
        delta_x = find_max_delta(traj.x)
        delta_y = find_max_delta(traj.y)
        delta_z = find_max_delta(traj.z)
        delta_yaw = find_max_delta(traj.yaw)
        variance_v = np.std(traj.v)
        low_speed = sum(1 for speed in traj.v if speed < const.LOW_SPEED_THRESHOLD) / length
        npc_in_fov = count_npcs_in_fov(self.actor_traj_list) / length
        ret = [delta_x, delta_y, delta_z, delta_yaw, variance_v, low_speed, npc_in_fov]
        
        # if self.state.stuck:
        #     ret.append(1)
        # else:
        #     ret.append(0)
        # if self.state.collision_event is not None:
        #     ret.append(1)
        # else:
        #     ret.append(0)
        return ret
    
    def get_vector(self):
        return np.array(self.static_conf_to_vector() + self.ego_traj_to_vector())
    
    def record_scenario(self):
        with open(
            "{}/{}.json".format(self.output_dir, time.strftime("%Y-%m-%d-%H-%M")), "w"
        ) as f:
            json.dump(self.dump(), f)
        
    def record_corner_case(self):
        if self.state.stuck:
            with open(
                "corner_case/stuck_x{:.2f}_y{:.2f}_{}.json".format(self.state.stuck_xy[0], self.state.stuck_xy[1], time.strftime("%Y-%m-%d-%H-%M")), "w"
            ) as f:
                json.dump(self.dump(), f)
        elif self.state.collision_event is not None:
            with open(
                "corner_case/collision_{}_{}.json".format(
                    self.state.collision_event.other_actor.type_id,
                    time.strftime("%Y-%m-%d-%H-%M"),
                ),
                "w",
            ) as f:
                json.dump(self.dump(), f)
    
def init_unique_scenario(
    carla_client,
    state: State,
    args,
    scenario_list: List[Scenario],
    factor=Factor(),
    ads_type=const.APOLLO,
) -> Scenario:
    spawn_points = carla_client.get_world().get_map().get_spawn_points()

    # generate spawn point
    unique_flag = False
    while not unique_flag:
        if factor.slope:
            sp_x, sp_y, sp_z, sp_yaw = random.choice(const.TOWN03_SLOP)
            sp = carla.Transform(
                carla.Location(x=sp_x, y=sp_y, z=sp_z),
                carla.Rotation(pitch=0, yaw=sp_yaw, roll=0),
            )
        else:
            sp = random.choice(spawn_points)
        unique_flag = True
        for scenario in scenario_list:
            if sp.location.distance(scenario.ego_sp.location) <= 1:
                unique_flag = False
                break
    # generate destination point
    dp = random.choice(spawn_points)
    distance = sp.location.distance(dp.location)
    while distance <= const.MIN_DIST_FROM_SP or distance >= const.MAX_DIST_FROM_SP:
        dp = random.choice(spawn_points)
        distance = sp.location.distance(dp.location)

    new_scenario = Scenario(carla_client, state, args, ads_type)
    new_scenario.ego_sp = sp
    new_scenario.ego_dp = dp
    new_scenario.weather.cloudiness = random.randint(0, 100)
    new_scenario.weather.precipitation = random.randint(0, 100)

    new_scenario.weather.wind_intensity = random.randint(0, 100)
    new_scenario.weather.sun_azimuth_angle = random.randint(0, 180)

    if factor.standing_water:
        new_scenario.weather.precipitation_deposits = random.randint(80, 100)
        new_scenario.weather.sun_altitude_angle = random.randint(-90, 60)
        new_scenario.weather.wetness = random.randint(50, 100)
    else:
        new_scenario.weather.precipitation_deposits = random.randint(0, 100)
        new_scenario.weather.sun_altitude_angle = random.randint(-90, 90)
        new_scenario.weather.wetness = random.randint(0, 100)

    new_scenario.weather.fog_density = 70
    new_scenario.weather.fog_distance = 0.750000
    new_scenario.weather.fog_falloff = 0.100000
    new_scenario.weather.scattering_intensity = 1.000000
    new_scenario.weather.mie_scattering_scale = 0.030000

    npc = NPC()

    # type, nav_type
    npc.type = const.VEHICLE
    npc.nav_type = const.AUTOPILOT

    # speed, sp, dp
    if factor.rear_end:
        npc.speed = random.uniform(2, 10)
        npc.sp = get_rare_end_npc_sp(new_scenario.ego_sp, random.uniform(8, 12))
        if factor.slope:
            npc.sp.location.z += 3
        npc.dp = new_scenario.ego_dp
    else:
        npc.speed = random.uniform(const.VEHICLE_MIN_SPEED, const.VEHICLE_MAX_SPEED)
        npc.sp = random.choice(spawn_points)
        while not new_scenario.is_valid_spawn_point(npc.sp):
            npc.sp = random.choice(spawn_points)
        npc.dp = random.choice(spawn_points)

    # blueprint
    if factor.large_target and factor.small_target:
        print("factor3, 4")
        npc.blueprint = new_scenario.blueprint_library.find(
            random.choice(const.LARGE_NPC_LIST + const.SMALL_NPC_LIST)
        )
    elif factor.large_target:
        print("factor4")
        npc.blueprint = new_scenario.blueprint_library.find(
            random.choice(const.LARGE_NPC_LIST)
        )
    elif factor.small_target:
        print("factor3")
        npc.blueprint = new_scenario.blueprint_library.find(
            random.choice(const.SMALL_NPC_LIST)
        )
    else:
        npc.blueprint = random.choice(
            new_scenario.blueprint_library.filter("vehicle.*")
        )

    new_scenario.add_npc(npc)

    return new_scenario

def crossover(parent1: Scenario, parent2: Scenario):
    # child1 = copy.deepcopy(parent1)
    # child2 = copy.deepcopy(parent2)
    child1 = parent1
    child2 = parent2
    
    if random.random() < 0.5:
        child1.weather.cloudiness, child2.weather.cloudiness = (
            parent2.weather.cloudiness, parent1.weather.cloudiness)
    
    if random.random() < 0.5:
        child1.weather.precipitation, child2.weather.precipitation = (
            parent2.weather.precipitation, parent1.weather.precipitation)

    if random.random() < 0.5:
        child1.weather.precipitation_deposits, child2.weather.precipitation_deposits = (
            parent2.weather.precipitation_deposits, parent1.weather.precipitation_deposits)

    if random.random() < 0.5:
        child1.weather.wind_intensity, child2.weather.wind_intensity = (
            parent2.weather.wind_intensity, parent1.weather.wind_intensity)

    if random.random() < 0.5:
        child1.weather.sun_azimuth_angle, child2.weather.sun_azimuth_angle = (
            parent2.weather.sun_azimuth_angle, parent1.weather.sun_azimuth_angle)

    if random.random() < 0.5:
        child1.weather.sun_altitude_angle, child2.weather.sun_altitude_angle = (
            parent2.weather.sun_altitude_angle, parent1.weather.sun_altitude_angle)

    if random.random() < 0.5:
        child1.weather.wetness, child2.weather.wetness = (
            parent2.weather.wetness, parent1.weather.wetness)

    min_npcs = min(len(parent1.npc_list), len(parent2.npc_list))

    for i in range(min_npcs):
        if random.random() < 0.5:
            child1.npc_list[i].blueprint, child2.npc_list[i].blueprint = (
                parent2.npc_list[i].blueprint, parent1.npc_list[i].blueprint)
        
        if random.random() < 0.5:
            child1.npc_list[i].speed, child2.npc_list[i].speed = (
                parent2.npc_list[i].speed, parent1.npc_list[i].speed)

    return child1, child2