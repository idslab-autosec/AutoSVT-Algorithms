""" Global States """

class State:
    def __init__(self):
        self.carla_map = None
        self.sim_start_time = 0
        self.num_frames = 0
        self.elapsed_time = 0
        self.node = None
        self.ego_vehicle = None
        self.seed = None

        # failure states
        self.spawn_failed = False
        self.spawn_failed_object = None

        # error states
        self.arrival = False
        self.collision_event = None
        self.stuck = False
        self.stuck_duration = 0
        self.stuck_xy = (0, 0)
        self.change_lane = False
        self.route_too_long = False

        self.alpha = 0
        self.visibility = float("inf")
        self.vehicle_len = 4.7
        self.check_arrival_threshold = 5
        self.distance_to_dst = float("inf")
        self.route_distace_to_dst = float("inf")
        self.route_distance_sp_to_dp = float("inf")
        self.dst_point = None
        self.start_point = None
        # self.apollo_fp_num = 0
        # self.apollo_fn_num = 0
        # self.in_fog_num = 0
        self.fog_noise_rate = 0

        # self.apollo_obs_list = []
        self.min_dist = float("inf")

        self.total_sim = 0
        self.total_corner_case = 0


    def reset(self):
        self.sim_start_time = 0
        self.num_frames = 0
        self.elapsed_time = 0
        self.ego_vehicle = None
        self.seed = None

        # failure states
        self.spawn_failed = False
        self.spawn_failed_object = None

        # error states
        self.arrival = False
        self.collision_event = None
        self.stuck = False
        self.stuck_duration = 0
        self.stuck_xy = (0, 0)
        self.change_lane = False
        self.route_too_long = False

        self.alpha = 0
        self.visibility = float("inf")
        self.vehicle_len = 4.7
        self.check_arrival_threshold = 5
        self.distance_to_dst = float("inf")
        self.route_distace_to_dst = float("inf")
        self.route_distance_sp_to_dp = float("inf")
        self.dst_point = None
        self.start_point = None
        # self.apollo_fp_num = 0
        # self.apollo_fn_num = 0
        # self.in_fog_num = 0
        self.fog_noise_rate = 0

        # self.apollo_obs_list = []
        self.min_dist = float("inf")


