import sys
import json
from states import State
from pycyber import cyber
import carla
from scenario import Scenario


def replay(json_file_name):
    with open(json_file_name, "r") as f:
        record = json.load(f)
    cyber.init("replay")
    node = cyber.Node("replay_node")
    state = State()
    state.node = node
    client = carla.Client("127.0.0.1", 2000)
    scenario = Scenario(client, state)
    scenario.load(record)
    ret = scenario.run_simulation(check_route=False)
    cyber.shutdown()
    return ret


def main():
    replay(sys.argv[1])


if __name__ == "__main__":
    main()
