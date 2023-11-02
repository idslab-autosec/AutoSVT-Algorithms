import carla
import random
from my_utils import *
import time
import constants as const
from states import State
import math
from typing import List
import traceback


class Scenario(object):
    def __init__(self, carla_client: carla.Client, state: State, ads=const.APOLLO):
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
        self.npc_list = []  # List[NPC]
        self.objective_function_value = 0
        self.modify_history = []  # List[Modification]
        self.is_corner_case = False

    def dump(self) -> dict:
        return {
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
        }

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

    def add_random_npc(self, type=const.VEHICLE) -> Modification:
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
        m = Modification()
        m.type = const.MOD_ADD_NPC
        m.new_value = npc
        m.index = len(self.npc_list) - 1
        self.modify_history.append(m)
        return m

    def add_npc(self, npc: NPC) -> Modification:
        self.npc_list.append(npc)
        m = Modification()
        m.type = const.MOD_ADD_NPC
        m.new_value = npc
        m.index = len(self.npc_list) - 1
        self.modify_history.append(m)
        return m

    def objective_function(self) -> float:
        self.objective_function_value = random.random()
        return self.objective_function_value
        # if self.state.num_frames == 0:
        #     self.objective_function_value = 0
        # else:
        #     self.objective_function_value = -(
        #         (
        #             self.state.apollo_fp_num
        #             + self.state.apollo_fn_num
        #             + self.state.in_fog_num
        #         )
        #         / self.state.num_frames
        #         + 1 / max(self.state.min_dist - 2.5, 0.1)
        #     )
        # return self.objective_function_value

    def print_simulation_result(self):
        print(
            "[*] objective_function_value = {:.2f}, num_frames = {}, num_fp = {}, num_fn = {}, change_lane = {}, min_distance = {:.2f}, in_fog_num = {}".format(
                self.objective_function_value,
                self.state.num_frames,
                self.state.apollo_fp_num,
                self.state.apollo_fn_num,
                self.state.change_lane,
                self.state.min_dist,
                self.state.in_fog_num,
            )
        )

    def random_modify_npc_speed(self, T=const.INIT_TEMPERATURE) -> Modification:
        m = Modification()
        m.type = const.MOD_NPC_SPEED
        m.index = random.randint(0, len(self.npc_list) - 1)
        m.old_value = self.npc_list[m.index].speed

        m.new_value = m.old_value + T / const.INIT_TEMPERATURE * random.uniform(-8, 8)
        m.new_value = min(const.VEHICLE_MAX_SPEED, m.new_value)
        m.new_value = max(const.VEHICLE_MIN_SPEED, m.new_value)

        self.npc_list[m.index].speed = m.new_value
        self.modify_history.append(m)
        return m

    def random_modify_npc_blueprint(self, vehicle_list=None) -> Modification:
        m = Modification()
        m.type = const.MOD_NPC_BP
        m.index = random.randint(0, len(self.npc_list) - 1)
        m.old_value = self.npc_list[m.index].blueprint
        if vehicle_list is not None:
            m.new_value = self.blueprint_library.find(random.choice(vehicle_list))
        else:
            vehicle_bps = self.blueprint_library.filter("vehicle.*")
            m.new_value = random.choice(vehicle_bps)
        self.npc_list[m.index].blueprint = m.new_value
        self.modify_history.append(m)
        return m

    def random_modify_weather(self, T=const.INIT_TEMPERATURE) -> Modification:
        r = random.randint(0, 1)
        m = Modification()
        if r == 0:
            # modify precipitation deposit
            m.type = const.MOD_PD
            m.old_value = self.weather.precipitation_deposits
            m.new_value = m.old_value + T / const.INIT_TEMPERATURE * random.uniform(
                -50, 50
            )
            m.new_value = min(100, m.new_value)
            m.new_value = max(0, m.new_value)
            self.weather.precipitation_deposits = m.new_value
        elif r == 1:
            # modify sun altitude angle
            # range: [-90, 90]
            m.type = const.MOD_SUN
            m.old_value = self.weather.sun_altitude_angle
            m.new_value = m.old_value + T / const.INIT_TEMPERATURE * random.uniform(
                -90, 90
            )
            m.new_value = min(90, m.new_value)
            m.new_value = max(-90, m.new_value)
            self.weather.sun_altitude_angle = m.new_value

        self.modify_history.append(m)
        return m

    def random_modify(self, T=const.INIT_TEMPERATURE, factor=Factor()) -> Modification:
        if factor.rear_end:
            npc = NPC()
            npc.type = const.VEHICLE
            npc.nav_type = const.AUTOPILOT
            npc.speed = random.uniform(2, 10)
            npc.sp = get_rare_end_npc_sp(self.ego_sp, random.uniform(8, 20))
            npc.dp = self.ego_dp
            npc.blueprint = random.choice(self.blueprint_library.filter("vehicle.*"))
            m = self.add_npc(npc)

        elif factor.standing_water:
            m = Modification()
            m.type = const.MOD_PD
            m.old_value = self.weather.precipitation_deposits
            m.new_value = random.uniform(70, 100)
            self.weather.precipitation_deposits = m.new_value
            self.modify_history.append(m)

        elif factor.small_target:
            m = self.random_modify_npc_blueprint(const.SMALL_NPC_LIST)

        elif factor.large_target:
            m = self.random_modify_npc_blueprint(const.LARGE_NPC_LIST)

        elif factor.slope:
            pass

        else:
            r = random.randint(0, 99)
            if r < max(0.3 * T, 10):
                m = self.add_random_npc()
            elif r < max(0.5 * T, 25):
                m = self.random_modify_npc_blueprint()
            elif r < max(0.9 * T, 60):
                m = self.random_modify_npc_speed(T)
            else:
                m = self.random_modify_weather(T)

        return m

    def rollback(self, m: Modification):
        if m.type == const.MOD_NPC_SPEED:
            self.npc_list[m.index].speed = m.old_value
        elif m.type == const.MOD_NPC_BP:
            self.npc_list[m.index].blueprint = m.old_value
        elif m.type == const.MOD_PD:
            self.weather.precipitaton_deposits = m.old_value
        elif m.type == const.MOD_SUN:
            self.weather.sun_altitude_angle = m.old_value
        elif m.type == const.MOD_ADD_NPC:
            self.npc_list.pop(m.index)
        else:
            # TODO: error
            pass

        for i in range(len(self.modify_history)):
            if self.modify_history[i] is m:
                self.modify_history.pop(i)

    def run_simulation(self, check_route=True):
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

                localization_channel = "/apollo/localization/pose"
                self.state.node.create_reader(
                    localization_channel,
                    LocalizationEstimate,
                    lambda data: check_arrival(data, self.state),
                )

                # clean history route
                # apollo_routing_request(self.state.node, (0, 0, 0), (0, 0, 0), 0)

                # check apollo obstacle detection
                perception_obstacle_channel = "/apollo/perception/obstacles"
                self.state.node.create_reader(
                    perception_obstacle_channel,
                    PerceptionObstacles,
                    lambda data: check_obstacle_detection(
                        data, self.state, self.map, ego
                    ),
                )

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

            self.world.wait_for_tick()  # sync with simulator

            # spawn npc actors
            walker_controller_bp = self.blueprint_library.find("controller.ai.walker")
            for npc in self.npc_list:
                if npc.type == const.VEHICLE:
                    actor_vehicle = self.world.try_spawn_actor(npc.blueprint, npc.sp)
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

                    # player_transform = ego.get_transform()
                    ego_location = ego.get_location()
                    # player_rot = player_transform.rotation

                    # Get speed
                    velocity_vector = ego.get_velocity()
                    speed = 3.6 * math.sqrt(
                        velocity_vector.x**2
                        + velocity_vector.y**2
                        + velocity_vector.z**2
                    )
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

                    for v in actor_vehicles:
                        dist = ego_location.distance(v.get_location())
                        if dist < self.state.min_dist:
                            self.state.min_dist = dist
                        if (
                            dist > const.FOG_RING_RANGE[0]
                            and dist < const.FOG_RING_RANGE[1]
                        ):
                            self.state.in_fog_num += 1

                    for w in actor_walkers:
                        dist = ego_location.distance(w.get_location())
                        if dist < self.state.min_dist:
                            self.state.min_dist = dist
                        if (
                            dist > const.FOG_RING_RANGE[0]
                            and dist < const.FOG_RING_RANGE[1]
                        ):
                            self.state.in_fog_num += 1

                    if self.ads_type == const.APOLLO:
                        if self.state.arrival:
                            print("\n[*] (Apollo) Reached the destination")
                            break

                    # Check crash
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
                        break

                    # Check stuck
                    if speed < 1:  # km/h
                        self.state.stuck_duration += 1
                    else:
                        self.state.stuck_duration = 0

                    if self.state.stuck_duration > (
                        const.TIMEOUT_SIM_SEC / fixed_delta_seconds
                    ):
                        self.state.stuck = True
                        print(
                            "\n[*] Stuck for too long: {}".format(
                                self.state.stuck_duration
                            )
                        )
                        retval = const.CORNER_CASE
                        self.is_corner_case = True
                        break

                    # Check apollo obstacles detection
                    if self.ads_type == const.APOLLO:
                        npc_list = actor_vehicles + actor_walkers
                        apollo_obs_list = self.state.apollo_obs_list
                        apollo_obs_fn, apollo_obs_fp = calculate_false_results(
                            apollo_obs_list, npc_list
                        )
                        self.state.apollo_fn_num += apollo_obs_fn
                        self.state.apollo_fp_num += apollo_obs_fp

            except KeyboardInterrupt:
                print("Keyboard Interrupt")
                retval = const.INTERRUPT

        except Exception as e:
            print("[-] Runtime error:")
            traceback.print_exc()
            retval = const.RUNTIME_ERROR

        finally:
            # Finalize simulation
            self.objective_function()
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


def init_unique_scenario(
    carla_client,
    state: State,
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

    new_scenario = Scenario(carla_client, state, ads_type)
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

    new_scenario.weather.fog_density = 80
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
