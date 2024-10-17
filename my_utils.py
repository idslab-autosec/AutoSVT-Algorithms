#!/usr/bin/env python3
import time
import math
import random
import constants as const
from typing import List
import carla
from scipy.spatial.transform import Rotation

from modules.common_msgs.localization_msgs.localization_pb2 import LocalizationEstimate
# from modules.common_msgs.perception_msgs.perception_obstacle_pb2 import PerceptionObstacles
from modules.common_msgs.planning_msgs.planning_pb2 import ADCTrajectory
from modules.common_msgs.routing_msgs.routing_pb2 import RoutingRequest, LaneWaypoint
from states import State
import numpy as np

class NPC(object):
    def __init__(self, type=const.VEHICLE, nav_type=const.AUTOPILOT, speed=10):
        self.type = type
        self.nav_type = nav_type
        self.speed = speed
        self.blueprint = None  # carla.ActorBlueprint
        self.sp = carla.Transform()
        self.dp = carla.Transform()
        self.bbox = carla.Vector3D()
    def get_bbox(self, bbox_extent: carla.Vector3D) -> None:
        self.bbox.x = bbox_extent.x * 2
        self.bbox.y = bbox_extent.y * 2
        self.bbox.z = bbox_extent.z * 2

class Mutation(object):
    def __init__(self):
        self.type = const.MUT_UNKNOW
        self.index = -1
        self.old_value = None
        self.new_value = None
        self.old_obj = None
        self.new_obj = None

    def print_modification(self):
        m_str_dict = {
            const.MUT_NPC_BP: "npc_list[{}] bluepint".format(self.index),
            const.MUT_NPC_DST: "npc_list[{}] destination".format(self.index),
            const.MUT_NPC_SPEED: "npc_list[{}] speed".format(self.index),
            const.MUT_WEATHER_PD: "precipitation deposits",
            const.MUT_WEATHER_SUN_AL: "sun altitude angle",
            const.MUT_NPC_ADD: "new npc",
        }
        if self.type == const.MUT_NPC_ADD:
            print("[+] Add a new NPC [{}]".format(self.new_value.blueprint.id))
        elif self.type == const.MUT_NPC_BP:
            print(
                "[+] Modify {}: {} -> {}".format(
                    m_str_dict[self.type], self.old_value.id, self.new_value.id
                )
            )
        else:
            print(
                "[+] Modify {}: {} -> {}".format(
                    m_str_dict[self.type], self.old_value, self.new_value
                )
            )

class Factor(object):
    def __init__(self):
        self.rear_end = False
        self.standing_water = False
        self.small_target = False
        self.large_target = False
        self.slope = False

    def reset(self):
        self.rear_end = False
        self.standing_water = False
        self.small_target = False
        self.large_target = False
        self.slope = False

    def random_choice(self)->str:
        tmp = random.randint(1, 4) # do not choose slope
        if tmp == 1:
            self.rear_end = True
            return "rear_end"
        elif tmp == 2:
            self.standing_water = True
            return "standing_water"
        elif tmp == 3:
            self.small_target = True
            return "small_target"
        elif tmp == 4:
            self.large_target = True
            return "large_target"
        elif tmp == 5:
            self.slope = True
            return "slope"

class ActorTrajectory(object):
    def __init__(self, id) -> None:
        self.actor_id = id
        self.x = []
        self.y = []
        self.z = []
        self.yaw = []
        self.v = []
    def append(self, x, y, z, yaw, v) -> None:
        self.x.append(x)
        self.y.append(y)
        self.z.append(z)
        self.yaw.append(yaw)
        self.v.append(v)
    def append_transform(self, tf: carla.Transform, v) -> None:
        self.x.append(tf.location.x)
        self.y.append(tf.location.y)
        self.z.append(tf.location.z)
        self.yaw.append(tf.rotation.yaw)
        self.v.append(v)

def get_ego_vehicle(world):
    world.wait_for_tick()
    actor_list = world.get_actors()
    for actor in actor_list:
        try:
            if (
                "hero" == actor.attributes["role_name"]
                or "ego_vehicle" == actor.attributes["role_name"]
            ):
                return actor
        except:
            pass
    return None


def get_rare_end_npc_sp(ego_sp, distance):
    npc_sp = carla.Transform(carla.Location(), ego_sp.rotation)
    ego_radian_yaw = math.radians(ego_sp.rotation.yaw)
    ego_radian_pitch = math.radians(ego_sp.rotation.pitch)

    npc_sp.location.x = ego_sp.location.x + distance * math.cos(ego_radian_yaw)
    npc_sp.location.y = ego_sp.location.y + distance * math.sin(ego_radian_yaw)
    npc_sp.location.z = ego_sp.location.z + distance * math.sin(ego_radian_pitch)
    return npc_sp


def get_carla_transform(positon_dict):
    if positon_dict is None:
        return None

    tf = carla.Transform(
        carla.Location(x=positon_dict["x"], y=positon_dict["y"], z=positon_dict["z"])
    )
    if "pitch" in positon_dict:
        tf.rotation.pitch = positon_dict["pitch"]
    if "yaw" in positon_dict:
        tf.rotation.yaw = positon_dict["yaw"]
    if "roll" in positon_dict:
        tf.rotation.roll = positon_dict["roll"]
    return tf


def get_dict_transform(tf: carla.Transform):
    return {
        "x": tf.location.x,
        "y": tf.location.y,
        "z": tf.location.z,
        "pitch": tf.rotation.pitch,
        "yaw": tf.rotation.yaw,
        "roll": tf.rotation.roll,
    }


# @param node: pycyber node
# @param start_coordinate: (x, y, z) in apollo coordinate
# @param end_coordinate: (x, y, z) in apollo coordinate
# @param start_yaw: degree in carla coordinate (North->-90, East->0)
def apollo_routing_request(
    node, start_coordinate, end_coordinate, start_yaw=0, sequence_num=0
):
    routing_request_channel = "/apollo/routing_request"
    msg = RoutingRequest()
    msg.header.timestamp_sec = time.time()
    msg.header.module_name = "routing_request"
    msg.header.sequence_num = sequence_num

    point = LaneWaypoint()
    # point.id = ""
    # point.s = ""
    shift = 5
    point.pose.x = start_coordinate[0] - shift * math.cos(math.radians(-start_yaw))
    point.pose.y = start_coordinate[1] - shift * math.sin(math.radians(-start_yaw))
    point.pose.z = start_coordinate[2]
    # print("start: ({}, {})， start_yaw: {}".format(point.pose.x, point.pose.y, start_yaw))
    msg.waypoint.append(point)

    point.pose.x = end_coordinate[0]
    point.pose.y = end_coordinate[1]
    point.pose.z = end_coordinate[2]
    msg.waypoint.append(point)

    # node = cyber.Node("routing request node")
    writer = node.create_writer(routing_request_channel, RoutingRequest)
    time.sleep(1)
    for _ in range(10):
        writer.write(msg)
        time.sleep(0.25)


def orientation_to_euler(apollo_orentation):
    q = apollo_orentation
    # print(type(q), q)
    q = (q.qx, q.qy, q.qz, q.qw)
    r = Rotation.from_quat(q)
    euler = r.as_euler("xyz")
    return euler


def check_arrival(localization_data: LocalizationEstimate, state: State):
    yaw = orientation_to_euler(localization_data.pose.orientation)[2]
    yaw += math.pi / 2
    current_x = localization_data.pose.position.x + state.vehicle_len * math.cos(yaw)
    current_y = localization_data.pose.position.y + state.vehicle_len * math.sin(yaw)
    # print("yaw = {}".format(yaw))
    # print("cx = {:.2f}, cy = {:.2f}, dx = {:.2f}, dy = {:.2f}".format(current_x, current_y, state.dst_point[0], state.dst_point[1]))
    state.distance_to_dst = math.sqrt(
        math.pow(state.dst_point[0] - current_x, 2)
        + math.pow(state.dst_point[1] - current_y, 2)
    )
    # print("state.distance_to_dst = {:.2f}".format(state.distance_to_dst))
    if state.distance_to_dst <= state.check_arrival_threshold:
        state.arrival = True
    else:
        state.arrival = False


# def check_obstacle_detection(
#     perception_obstacle_data: PerceptionObstacles, state: State, carla_map, ego
# ):
#     apollo_obstacles_list = []
#     for obs in perception_obstacle_data.perception_obstacle:
#         obs_location = carla.Location(
#             x=obs.position.x, y=-obs.position.y, z=obs.position.z
#         )
#         if obs_location.distance(ego.get_location()) > const.DETECTION_RANGE:
#             continue
#         obs_waypoint = carla_map.get_waypoint(
#             obs_location,
#             project_to_road=False,
#             lane_type=carla.LaneType.Driving | carla.LaneType.Shoulder,
#         )
#         if obs_waypoint is not None:
#             apollo_obstacles_list.append(obs_location)
#     state.apollo_obs_list = apollo_obstacles_list


def check_planning(planning_data, state: State):
    state.route_distace_to_dst = planning_data.debug.planning_data.routing.measurement.distance
    if planning_data.decision.main_decision.cruise.change_lane_type != 0:
        state.change_lane = True
    if state.route_distace_to_dst >= const.MAX_ROUTE_DISTANCE:
        state.route_too_long = True
    else:
        state.route_too_long = False


def on_collision(event, state: State):
    if event.frame > state.first_frame_id + state.num_frames:
        return

    if event.other_actor.type_id != "staticonst.road":
        state.collision_event = event


def calculate_false_results(detected_obstacles, npc_actors):
    false_negatives = 0
    false_positives = 0
    threshold = 5

    # Detect false negative
    for npc in npc_actors:
        npc_location = npc.get_location()
        is_detected = False
        for obs in detected_obstacles:
            if npc_location.distance(obs) <= threshold:
                is_detected = True
                break
        if not is_detected:
            false_negatives += 1
            # print("[!] false negative: {}, {}".format(npconst.type_id, npconst.get_location()))

    # Detect false positive
    for obs in detected_obstacles:
        is_false_positive = True
        for npc in npc_actors:
            if obs.distance(npc.get_location()) <= threshold:
                is_false_positive = False
                break
        if is_false_positive:
            false_positives += 1
            # print("[!] false positive: {}".format(obs))

    return false_negatives, false_positives

def calculate_velocity(velocity_vector) -> float:
    # return velocity in km/h
    return 3.6 * math.sqrt(
        velocity_vector.x**2
        + velocity_vector.y**2
        + velocity_vector.z**2
    )
    
def find_max_delta(l, window_size=const.WINDOW_SIZE):
    max_delta = float('-inf')
    ret_delta = 0

    for i in range(len(l) - window_size + 1):
        window = l[i:i + window_size]
        delta = max(window) - min(window)
        
        if delta > max_delta:
            max_delta = delta
            max_index = window.index(max(window))
            min_index = window.index(min(window))
            
            if max_index > min_index:
                ret_delta = delta
            else:
                ret_delta = -delta

    return ret_delta

def count_npcs_in_fov(actor_traj_list: List[ActorTrajectory], fov_distance=const.FOV_DISTANCE, fov_angle=const.FOV_ANGLE):
    ego_x_list = actor_traj_list[0].x
    ego_y_list = actor_traj_list[0].y
    ego_yaw_list = actor_traj_list[0].yaw
    count = 0
    # ego_yaw_rad = math.radians(ego_yaw)
    
    for traj in actor_traj_list[1:]:
        for i in range(len(traj.x)):
            npc_x = traj.x[i]
            npc_y = traj.y[i]
        
            distance = math.sqrt((npc_x - ego_x_list[i]) ** 2 + (npc_y - ego_y_list[i]) ** 2)
        
            if distance <= fov_distance:
                npc_angle = math.atan2(npc_y - ego_y_list[i], npc_x - ego_x_list[i])
                npc_angle_deg = math.degrees(npc_angle)
                angle_diff = (npc_angle_deg - ego_yaw_list[i] + 180) % 360 - 180
                if -fov_angle <= angle_diff <= fov_angle:
                    count += 1
    
    return count


def calculate_fog_noise_rate(raw_lidar_data, state: State) -> float:
    lidar_data_dtype = np.dtype([
            ('x', np.float32),
            ('y', np.float32),
            ('z', np.float32),
            ('intensity', np.float32),
            ('object_tag', np.uint32),
        ])
    lidar_data = np.fromstring(bytes(raw_lidar_data.raw_data), dtype=lidar_data_dtype)
    tags = lidar_data["object_tag"]
    fog_noise_num = np.sum(tags == 29)
    state.fog_noise_rate = fog_noise_num / tags.shape[0]
    
def softmax(x):
    e_x = np.exp(x - np.max(x))
    return e_x / e_x.sum(axis=0)

def select_scenario_by_score(scenarios: List, n=1):
    if n > len(scenarios):
        raise ValueError("n must be less than or equal to the number of scenarios.")
    
    scores = np.array([s.score for s in scenarios])
    probabilities = softmax(scores)
    selected_index = np.random.choice(len(scenarios), size=n, replace=False, p=probabilities)
    return selected_index

def subtract_lists(list1, list2):
    if len(list1) != len(list2):
        raise ValueError("两个列表的长度必须相同")
    
    result = [a - b for a, b in zip(list1, list2)]
    return result
