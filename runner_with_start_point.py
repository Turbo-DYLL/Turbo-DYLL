import csv
import logging
import time
import warnings
from pathlib import Path
from typing import Tuple

import numpy as np
from prettytable import PrettyTable

import utils
from ROAR.agent_module.aaron_pid_agent import PIDFastAgent
from ROAR.agent_module.pure_pursuit_agent \
    import PurePursuitAgent
from ROAR.agent_module.timer_wrapper_agent import TimerWrapperAgent
from ROAR.agent_module.turbo_pid_agent import TurboPIDAgent
from ROAR.configurations.configuration import Configuration as AgentConfig
from ROAR.utilities_module.data_structures_models import Location, Transform
from ROAR_Sim.carla_client.carla_runner import CarlaRunner
from ROAR_Sim.configurations.configuration import Configuration as CarlaConfig


# aaron import


def compute_score(carla_runner: CarlaRunner) -> Tuple[float, int, int]:
    """
    Calculates the score of the vehicle upon completion of the track based on certain metrics
    Args:
        carla_runner ():

    Returns:
        time_elapsed:
        num_collision: number of collisions during simulation
        laps_completed: Number of laps completed

    """
    time_elapsed: float = carla_runner.end_simulation_time - carla_runner.start_simulation_time
    num_collision: int = carla_runner.agent_collision_counter
    laps_completed = 0 if carla_runner.completed_lap_count < 0 else carla_runner.completed_lap_count

    return time_elapsed, num_collision, laps_completed


def run(agent_class,
        start_line: int,
        start_transform: Transform,
        end_location: Location,
        waypoint_path: Path,
        agent_config_file_path: Path,
        carla_config_file_path: Path,
        num_laps: int = 10) -> Tuple[float, int, int]:
    """
    Run the agent along the track and produce a score based on certain metrics
    Args:
        num_laps: int number of laps that the agent should run
        agent_class: the participant's agent
        start_line: the starting line
        start_transform: the starting point of this comparison
        end_location: the end point of this comparison
        waypoint_path: the path to the waypoints
        agent_config_file_path: agent configuration path
        carla_config_file_path: carla configuration path
    Returns:
        float between 0 - 1 representing scores
    """

    agent_config: AgentConfig = AgentConfig.parse_file(agent_config_file_path)
    carla_config = CarlaConfig.parse_file(carla_config_file_path)

    # hard code agent config such that it reflect competition requirements
    agent_config.num_laps = num_laps
    # hard code waypoint path
    agent_config.waypoint_file_path = waypoint_path
    carla_runner = CarlaRunner(carla_settings=carla_config,
                               agent_settings=agent_config,
                               npc_agent_class=PurePursuitAgent,
                               competition_mode=True,
                               start_bbox=[2530, 99, 4064, 2543, 120, 4076],
                               lap_count=num_laps)
    try:
        my_vehicle = carla_runner.set_carla_world()
        agent = TimerWrapperAgent(agent_class, end_location, vehicle=my_vehicle, agent_settings=agent_config)
        from ROAR.control_module import controls
        while controls.controls_sequence.__len__() > 1 and controls.controls_sequence[1].start_line <= start_line:
            controls.controls_sequence.pop(0)
            print("pop")

        print(controls.controls_sequence.__len__().__str__() + " controls left")
        carla_runner.world.player.set_transform(utils.convert_transform_from_agent_to_source(start_transform))
        carla_runner.start_game_loop(agent=agent, use_manual_control=False)
        print(compute_score(carla_runner)[1])
        return compute_score(carla_runner)
    except Exception as e:
        print(f"something bad happened during initialization: {e}")
        carla_runner.on_finish()
        logging.error(f"{e}. Might be a good idea to restart Server")
        raise e


def suppress_warnings():
    logging.basicConfig(format='%(levelname)s - %(asctime)s - %(name)s '
                               '- %(message)s',
                        level=logging.INFO)
    logging.getLogger("matplotlib").setLevel(logging.WARNING)
    warnings.simplefilter("ignore")
    np.set_printoptions(suppress=True)


class TestMode:
    TURBO_ONLY = 0
    PID_ONLY = 1
    BOTH = 2


def main():
    suppress_warnings()
    test_mode = TestMode.TURBO_ONLY
    agent_class_list = []
    waypoint_path_list = []
    if test_mode % 2 == 0:
        agent_class_list.append(TurboPIDAgent)
        waypoint_path_list.append(Path("./ROAR/datasets/segment_waypoint/eric-waypoints-jump.txt"))
    if test_mode > 0:
        agent_class_list.append(PIDFastAgent)
        waypoint_path_list.append(Path("./ROAR/datasets/aaronWaypoint.txt"))

    start_line = 0
    end_line = 12423
    my_waypoint_path = Path("./ROAR/datasets/segment_waypoint/eric-waypoints-jump.txt")
    temp_waypoint_path = Path("./ROAR/datasets/segment_waypoint/waypoints.temp")
    with open(my_waypoint_path, "r") as f:
        lines = f.readlines()
        end_location = utils.convert_location_from_str_to_agent(lines[end_line - 1])

    with open(temp_waypoint_path, "w") as f:
        f.writelines(lines[start_line:])

    start_transform = utils.convert_transform_from_str_to_agent(lines[start_line])

    total_array = []
    num_laps = 1
    table = PrettyTable()
    table.field_names = ["agent_name", "time_elapsed (sec)", "num_collisions", "laps completed"]

    for i in range(len(agent_class_list)):
        scores = run(agent_class=agent_class_list[i],
                     start_line=start_line,
                     start_transform=start_transform,
                     end_location=end_location,
                     waypoint_path=temp_waypoint_path,
                     agent_config_file_path=Path("./ROAR/configurations/carla/carla_agent_configuration.json"),
                     carla_config_file_path=Path("./ROAR_Sim/configurations/configuration.json"),
                     num_laps=num_laps)
        total_array.append(scores)
        table.add_row([agent_class_list[i].__name__, scores[0], scores[1], scores[2]])

    print(table)


if __name__ == "__main__":
    main()
