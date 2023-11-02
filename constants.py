TIMEOUT_SIM_SEC = 180

# Agent Types
APOLLO = 1
OTHER = -1

# Static configurations
MAX_DIST_FROM_SP = 150
MIN_DIST_FROM_SP = 30
MAX_DIST_FROM_EGO = 40
MIN_DIST_FROM_EGO = 5
MIN_DIST_BETWEEN_NPC = 3
MAX_ROUTE_DISTANCE = 300

# Simulation return
FAIL = -1
INIT = 0
CORNER_CASE = 1
RUNTIME_ERROR = 2
INTERRUPT = 3
ROUTE_TOO_LONG = 4

# NPC Type
NULL = -1  # special type for traction testing
VEHICLE = 0
WALKER = 1
STATIC_OBJECT = 2
NPC_TYPE_LIST = [VEHICLE, WALKER, STATIC_OBJECT]
NPC_TYPE_NAME_LIST = ["vehicle", "walker", "static_object"]

# Actor Navigation Type
LINEAR = 0
AUTOPILOT = 1
IMMOBILE = 2

NAV_TYPE_LIST = [LINEAR, AUTOPILOT, IMMOBILE]
NAV_TYPE_NAME_LIST = ["linear", "autopilot", "immobile"]

# Actor Attributes
VEHICLE_MAX_SPEED = 30  # multiplied with forward vector
VEHICLE_MIN_SPEED = 5
WALKER_MAX_SPEED = 8  # m/s
WALKER_CROSS_RATE = 0.5  # how many pedestrians will walk through the road

# Optimization
INIT_TEMPERATURE = 100
MIN_TEMPERATURE = 10
COOLING_FACTOR = 0.5

MIN_DIST_REFRESH = 15

MOD_UNKNOW = -1
MOD_PD = 0
MOD_SUN = 1
MOD_NPC_SPEED = 2
MOD_NPC_BP = 3
MOD_NPC_DST = 4
MOD_ADD_NPC = 5

DETECTION_RANGE = 30  # meters
FOG_RING_RANGE = (4, 5)

LARGE_NPC_LIST = [
    "vehicle.citroen.c3",
    "vehicle.micro.microlino",
    "vehicle.jeep.wrangler_rubicon",
    "vehicle.carlamotors.carlacola",
    "vehicle.mercedes.sprinter",
    "vehicle.audi.etron",
    "vehicle.volkswagen.t2_2021",
    "vehicle.tesla.cybertruck",
    "vehicle.ford.mustang",
    "vehicle.volkswagen.t2",
    "vehicle.mitsubishi.fusorosa",
    "vehicle.nissan.patrol",
    "vehicle.nissan.micra",
    "vehicle.mini.cooper_s",
    "vehicle.ford.ambulance",
    "vehicle.carlamotors.firetruck",
]

SMALL_NPC_LIST = [
    "vehicle.micro.microlino",
    "vehicle.nissan.micra",
    "vehicle.harley-davidson.low_rider",
    "vehicle.kawasaki.ninja",
    "vehicle.vespa.zx125",
    "vehicle.yamaha.yzf",
    "vehicle.bh.crossbike",
    "vehicle.diamondback.century",
    "vehicle.gazelle.omafiets",
]

TOWN03_SLOP = [
    (16.2, -134.43, 0.5, 0),
    (82.67, -185.89, 0.5, 90),
    (80.96, -13.41, 0.5, -90),
    (151.13, -182.17, 1.5, 90),
    (152.13, -16.17, 1, -90),
    (72.13, -136.57, 10, -175),
    (79.53, -65.57, 10, 93),
    (85.53, -144.57, 10, -88),
    (154.53, -140.57, 10, -88),
    (149.13, -65.17, 10, 90),
]
