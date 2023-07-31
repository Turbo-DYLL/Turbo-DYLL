import logging
from pathlib import Path
from ROAR_Sim.configurations.configuration import Configuration as CarlaConfig

from ROAR.agent_module.pure_pursuit_agent import PurePursuitAgent
from ROAR.configurations.configuration import Configuration as AgentConfig
import argparse
from misc.utils import str2bool
import carla

from ROAR_Sim.carla_client.carla_runner import CarlaRunner
from ROAR.agent_module.special_agents.segment_waypoint_generating_agent import WaypointGeneratingAgent
from opencv_map import MapViewer
import util


def main(args):
    """Starts game loop"""
    agent_config = AgentConfig.parse_file(Path("./ROAR/configurations/carla/carla_agent_configuration.json"))
    carla_config = CarlaConfig.parse_file(Path("./ROAR_Sim/configurations/configuration.json"))

    carla_runner = CarlaRunner(carla_settings=carla_config,
                               agent_settings=agent_config,
                               npc_agent_class=PurePursuitAgent)
    interactive_map_viewer = MapViewer()
    waypoints_file = Path("./ROAR/datasets/segment_waypoint_test/main.txt")
    while True:
        try:
            spawn_point = util.get_coordinates_from_last_line(waypoints_file)
            my_vehicle = carla_runner.set_carla_world()
            agent = WaypointGeneratingAgent(vehicle=my_vehicle, agent_settings=agent_config)
            print("spawn_point: ", spawn_point)
            #TODO fix the bug where if u copy in a coord the next line that it prints does not start with a \n 
            if spawn_point:
                carla_runner.world.player.set_transform(spawn_point)
            carla_runner.start_game_loop(agent=agent,
                                        use_manual_control=not args.auto)
        except Exception as e:
            logging.error(f"Something bad happened during initialization: {e}")
            carla_runner.on_finish()
            logging.error(f"{e}. Might be a good idea to restart Server")
            raise e

        with open(waypoints_file, "r") as file:
            waypoints = file.readlines()

        interactive_map_viewer.update(waypoints, agent.waypoints_list)

        # 0: save, 1: discard, 2: save&quit 3: discard&quit
        choice = interactive_map_viewer.interactive_map(util.get_coords_from_str(agent.waypoints_list[-1]))
        print(f"last waypoint: {agent.waypoints_list[-1]}")

        if choice % 2 == 0:
            with open(waypoints_file, "a") as file:
                file.writelines(agent.waypoints_list)
            print("waypoint saved")
        else:
            print("waypoint discarded")

        if choice > 1:
            print("quitting")
            break



if __name__ == "__main__":
    logging.basicConfig(format='%(levelname)s - %(asctime)s - %(name)s '
                               '- %(message)s',
                        datefmt="%H:%M:%S",
                        level=logging.DEBUG)
    logging.getLogger(" streaming client").setLevel(logging.WARNING)
    import warnings

    warnings.filterwarnings("ignore", module="carla")
    parser = argparse.ArgumentParser()
    # Manual control param
    parser.add_argument("--auto", type=str2bool, default=False, help="True to use auto control")

    warnings.filterwarnings("ignore", module="carla")
    args = parser.parse_args()
    main(args)
