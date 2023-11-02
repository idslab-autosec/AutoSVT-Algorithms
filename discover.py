from scenario import Scenario, init_unique_scenario
from my_utils import *
import constants as const
from states import State
import random
import math
import json
import time
import carla
from pycyber import cyber
import argparse

def optimize(scenario: Scenario, max_cycle: int):
    # the first time
    print("-" * 80)
    print("[*] Init scenario")
    sim_ret = scenario.run_simulation()
    scenario.state.total_sim += 1
    if sim_ret == const.ROUTE_TOO_LONG:
        print("The destination is too far, reset.")
        scenario.state.total_sim -= 1
        return const.ROUTE_TOO_LONG

    scenario.print_simulation_result()
    if scenario.is_corner_case:
        scenario.state.total_corner_case += 1
        if scenario.state.stuck:
            with open(
                "output/{}_stuck.json".format(time.strftime("%Y-%m-%d-%H-%M")), "w"
            ) as f:
                json.dump(scenario.dump(), f)
        elif scenario.state.collision_event is not None:
            with open(
                "output/collision_{}_{}.json".format(
                    scenario.state.collision_event.other_actor.type_id,
                    time.strftime("%Y-%m-%d-%H-%M"),
                ),
                "w",
            ) as f:
                json.dump(scenario.dump(), f)
        return const.CORNER_CASE

    while scenario.state.min_dist > const.MIN_DIST_REFRESH:
        scenario.rollback(scenario.modify_history[0])
        scenario.add_random_npc(type=const.VEHICLE)
        print("-" * 80)
        print("[*] Init scenario")
        scenario.run_simulation()
        scenario.state.total_sim += 1
        scenario.print_simulation_result()
        if scenario.is_corner_case:
            scenario.state.total_corner_case += 1
            if scenario.state.stuck:
                with open(
                    "output/stuck_{}.json".format(time.strftime("%Y-%m-%d-%H-%M")), "w"
                ) as f:
                    json.dump(scenario.dump(), f)
            elif scenario.state.collision_event is not None:
                with open(
                    "output/collision_{}_{}.json".format(
                        time.strftime("%Y-%m-%d-%H-%M"),
                        scenario.state.collision_event.other_actor.type_id,
                    ),
                    "w",
                ) as f:
                    json.dump(scenario.dump(), f)
            return const.CORNER_CASE

    T = const.INIT_TEMPERATURE
    while T >= const.MIN_TEMPERATURE:
        for i in range(max_cycle):
            print("-" * 80)
            print(
                "[*] T = {}, round = {}, time = {}".format(
                    T, i, time.strftime("%Y-%m-%d %H:%M:%S")
                )
            )

            factor = Factor()
            if random.uniform(0, 1) < 0.008 * T:
                # Randomly select factor
                factor_str = factor.random_choice()
                print("[*] Choose factor: {}".format(factor_str))

            m = scenario.random_modify(T, factor)
            m.old_obj = scenario.objective_function_value
            m.print_modification()

            sim_ret = scenario.run_simulation()
            if sim_ret != const.FAIL:
                scenario.state.total_sim += 1
            scenario.print_simulation_result()
            if scenario.is_corner_case:
                scenario.state.total_corner_case += 1
                if scenario.state.collision_event is not None:
                    print(
                        "collision with {}".format(
                            scenario.state.collision_event.other_actor.type_id
                        )
                    )
                elif scenario.state.stuck:
                    print("stuck")
                # record corner case
                record_dict = scenario.dump()
                record_dict["corner_case"] = {
                    "collision_event": {
                        "collision_object": scenario.state.collision_event.other_actor.type_id,
                    }
                    if (scenario.state.collision_event is not None)
                    else None,
                    "stuck": scenario.state.stuck,
                }
                modify_str_list = [
                    "precipitation_deposits",
                    "sun_altitude_angle",
                    "npc_speed",
                    "npc_blueprint",
                    "npc_destination",
                    "add_new_npc",
                ]

                record_dict["modify_history"] = []
                for modify in scenario.modify_history:
                    modify_dict = {
                        "type": modify_str_list[modify.type],
                        "index": modify.index,
                    }
                    if modify.type == const.MOD_ADD_NPC:
                        modify_dict["old_value"] = None
                        modify_dict["new_value"] = modify.new_value.blueprint.id
                    elif modify.type == const.MOD_NPC_BP:
                        modify_dict["old_value"] = modify.old_value.id
                        modify_dict["new_value"] = modify.new_value.id
                    else:
                        modify_dict["old_value"] = modify.old_value
                        modify_dict["new_value"] = modify.new_value
                    record_dict["modify_history"].append(modify_dict)

                if scenario.state.collision_event is not None:
                    filename_prefix = "collision_{}".format(
                        scenario.state.collision_event.other_actor.type_id
                    )
                elif scenario.state.stuck:
                    filename_prefix = "stuck"
                with open(
                    "output/{}_{}.json".format(
                        filename_prefix, time.strftime("%Y-%m-%d-%H-%M")
                    ),
                    "w",
                ) as f:
                    json.dump(record_dict, f)
                return const.CORNER_CASE
            m.new_obj = scenario.objective_function_value
            if m.new_obj < m.old_obj:
                print("[*] Accept the modification")
            else:
                # the old scenario is better
                p = math.exp((m.old_obj - m.new_obj) / T * 25)
                r = random.uniform(0, 1)
                if r < p:
                    print(
                        "[*] Accept the modification with probability {:.2f}".format(p)
                    )
                else:
                    # rollback to old scenario
                    scenario.rollback(m)
                    print("[*] Refuse the modification (rollback)")
        T = T * const.COOLING_FACTOR


def parse_arguments():
    parser = argparse.ArgumentParser(description="Discovering corner cases.")
    parser.add_argument("--host", help="Carla server ip", type=str, default="127.0.0.1")
    parser.add_argument(
        "-p", "--port", help="Carla server port", type=int, default=2000
    )

    parser.add_argument("-a", "--ads", help="Type of ADS", type=str, default="Apollo")
    parser.add_argument(
        "-n", "--num-scenario", help="Number of scenarios", type=int, default=10
    )
    # parser.add_argument(
    #     "-t", "--init-temperature", help="Initial temperature", type=int, default=100
    # )
    parser.add_argument(
        "-m",
        "--max-cycle",
        help="The maximum number of cycles at a certain temperature",
        type=int,
        default=8,
    )

    parser.add_argument(
        "--rear",
        help="[Factor1] Rear-end collision",
        action="store_true",
        default=False,
    )
    parser.add_argument(
        "--water",
        help="[Factor2] Standing water reflection",
        action="store_true",
        default=False,
    )
    parser.add_argument(
        "--small",
        help="[Factor3] Small targets",
        action="store_true",
        default=False,
    )
    parser.add_argument(
        "--large",
        help="[Factor4] Large targets",
        action="store_true",
        default=False,
    )
    parser.add_argument(
        "--slope",
        help="[Factor5] Uphill and downhill",
        action="store_true",
        default=False,
    )

    arguments = parser.parse_args()
    return arguments


def main():
    args = parse_arguments()
    cyber.init("AutoSVT")
    node = cyber.Node("AutoSVT_node")
    state = State()
    state.node = node
    client = carla.Client(args.host, args.port)

    scenario_list = []
    factor = Factor()
    factor.rear_end = args.rear
    factor.standing_water = args.water
    factor.small_target = args.small
    factor.large_target = args.large
    factor.slope = args.slope

    if args.ads.lower() == "apollo":
        ads_type = const.APOLLO
    else:
        ads_type = const.OTHER

    sim_round = 0
    while sim_round < args.num_scenario:
        scenario = init_unique_scenario(client, state, scenario_list, factor, ads_type)
        scenario_list.append(scenario)
        ret = optimize(scenario, args.max_cycle)
        if ret != const.ROUTE_TOO_LONG:
            sim_round += 1
        
    cyber.shutdown()


if __name__ == "__main__":
    main()
