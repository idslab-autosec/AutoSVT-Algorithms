#!/usr/bin/env python3
import time
import math
import random
import constants as const

import carla
from scipy.spatial.transform import Rotation

from modules.common_msgs.localization_msgs.localization_pb2 import LocalizationEstimate
from modules.common_msgs.perception_msgs.perception_obstacle_pb2 import (
    PerceptionObstacles,
)
from modules.common_msgs.planning_msgs.planning_pb2 import ADCTrajectory
from modules.common_msgs.routing_msgs.routing_pb2 import RoutingRequest, LaneWaypoint
from states import State

class NPC(object):
    def __init__(self, type=const.VEHICLE, nav_type=const.AUTOPILOT, speed=10):
        self.type = type
        self.nav_type = nav_type
        self.speed = speed
        self.blueprint = None  # carla.ActorBlueprint
        self.sp = carla.Transform()
        self.dp = carla.Transform()


class Modification(object):
    def __init__(self):
        self.type = const.MOD_UNKNOW
        self.index = -1
        self.old_value = None
        self.new_value = None
        self.old_obj = None
        self.new_obj = None

    def print_modification(self):
        m_str_dict = {
            const.MOD_NPC_BP: "npc_list[{}] bluepint".format(self.index),
            const.MOD_NPC_DST: "npc_list[{}] destination".format(self.index),
            const.MOD_NPC_SPEED: "npc_list[{}] speed".format(self.index),
            const.MOD_PD: "precipitation deposits",
            const.MOD_SUN: "sun altitude angle",
            const.MOD_ADD_NPC: "new npc",
        }
        if self.type == const.MOD_ADD_NPC:
            print("[+] Add a new NPC [{}]".format(self.new_value.blueprint.id))
        elif self.type == const.MOD_NPC_BP:
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


def check_obstacle_detection(
    perception_obstacle_data: PerceptionObstacles, state: State, carla_map, ego
):
    apollo_obstacles_list = []
    for obs in perception_obstacle_data.perception_obstacle:
        obs_location = carla.Location(
            x=obs.position.x, y=-obs.position.y, z=obs.position.z
        )
        if obs_location.distance(ego.get_location()) > const.DETECTION_RANGE:
            continue
        obs_waypoint = carla_map.get_waypoint(
            obs_location,
            project_to_road=False,
            lane_type=carla.LaneType.Driving | carla.LaneType.Shoulder,
        )
        if obs_waypoint is not None:
            apollo_obstacles_list.append(obs_location)
    state.apollo_obs_list = apollo_obstacles_list


def check_planning(planning_data, state: State):
    if planning_data.decision.main_decision.cruise.change_lane_type != 0:
        state.change_lane = True
    if planning_data.debug.planning_data.routing.measurement.distance >= const.MAX_ROUTE_DISTANCE:
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
