import logging
import warnings
from enum import Enum
from pathlib import Path
from typing import Tuple, List

import numpy as np
from prettytable import PrettyTable

import utils
from ROAR.agent_module.aaron_pid_agent import PIDFastAgent
from ROAR.agent_module.record_wrapper_agent import RecordWrapperAgent
from ROAR.agent_module.pure_pursuit_agent \
    import PurePursuitAgent
from ROAR.agent_module.turbo_pid_agent import TurboPIDAgent
from ROAR.configurations.configuration import Configuration as AgentConfig
from ROAR.utilities_module.data_structures_models import Location
from ROAR_Sim.carla_client.carla_runner import CarlaRunner
from ROAR_Sim.configurations.configuration import Configuration as CarlaConfig

import csv


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
        end_location: Location,
        waypoint_record_list: List[Location],
        waypoint_path: Path,
        agent_config_file_path: Path,
        carla_config_file_path: Path,
        num_laps: int = 10) -> Tuple[float, float, List]:
    """
    Run the agent along the track and produce a score based on certain metrics
    Args:
        num_laps: int number of laps that the agent should run
        agent_class: the participant's agent
        end_location: the end point of this comparison
        waypoint_record_list: the list of waypoints
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
        agent = RecordWrapperAgent(agent_class, end_location, waypoint_record_list, carla_runner, vehicle=my_vehicle,
                                   agent_settings=agent_config)
        carla_runner.start_game_loop(agent=agent, use_manual_control=False)
        return carla_runner.start_simulation_time, carla_runner.end_simulation_time, agent.time_list
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


def main():
    suppress_warnings()

    class Mode(Enum):
        turbo_pid = 0
        pid = 1

    mode = Mode.turbo_pid
    if mode == Mode.turbo_pid:
        agent_class = TurboPIDAgent
        output_path = Path("./ROAR/datasets/pid_comparison/turbo.csv")
        waypoints_path = Path("./ROAR/datasets/segment_waypoint/eric-waypoints-jump.txt")
    else:
        agent_class = PIDFastAgent
        output_path = Path("./ROAR/datasets/pid_comparison/pid.csv")
        waypoints_path = Path("./ROAR/datasets/aaronWaypoint.txt")

    my_waypoint_list = Path("./ROAR/datasets/segment_waypoint/eric-waypoints-jump.txt")
    with open(my_waypoint_list, "r") as f:
        lines = f.readlines()

    i = 0
    waypoint_record_list = []
    while i < len(lines):
        waypoint_record_list.append(utils.convert_location_from_str_to_agent(lines[i]))
        i += 500
    waypoint_record_list.append(utils.convert_location_from_str_to_agent(lines[-1]))

    end_line = 0
    end_location = utils.convert_location_from_str_to_agent(lines[end_line - 1])

    num_laps = 1
    table = PrettyTable()
    table.field_names = ["agent_name", "time_elapsed (sec)", "num_collisions", "laps completed"]

    start_time, stop_time, time_list = run(agent_class=agent_class,
                                           end_location=end_location,
                                           waypoint_record_list=waypoint_record_list,
                                           waypoint_path=waypoints_path,
                                           agent_config_file_path=Path(
                                               "./ROAR/configurations/carla/carla_agent_configuration.json"),
                                           carla_config_file_path=Path("./ROAR_Sim/configurations/configuration.json"),
                                           num_laps=num_laps)

    with open(output_path, 'w', newline='') as csvfile:
        writer = csv.writer(csvfile)
        for i in range(len(time_list)):
            writer.writerow([i * 500, time_list[i] - start_time])
        if len(lines) % 500 != 0:
            writer.writerow([len(lines), stop_time - start_time])


if __name__ == "__main__":
    main()
