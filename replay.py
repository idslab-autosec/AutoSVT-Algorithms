import sys
import json
from states import State
from pycyber import cyber
import carla
from scenario import Scenario
import argparse
def parse_arguments():
    parser = argparse.ArgumentParser(description="Discovering corner cases.")
    parser.add_argument("-r", "--record", help="Input json file.", type=str, default=None)
    parser.add_argument("-o", "--output", help="Output directory", type=str, default="output_test")
    arguments = parser.parse_args()
    return arguments

def replay(args):
    with open(args.record, "r") as f:
        record = json.load(f)
    cyber.init("replay")
    node = cyber.Node("replay_node")
    state = State()
    state.node = node
    client = carla.Client("127.0.0.1", 2000)
    scenario = Scenario(client, state, args)
    scenario.load(record)
    ret = scenario.run_simulation(total_sim=1, check_route=False)
    cyber.shutdown()
    return ret


def main():
    args = parse_arguments()
    replay(args)


if __name__ == "__main__":
    main()
