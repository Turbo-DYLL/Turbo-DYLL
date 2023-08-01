import logging
from pathlib import Path
from ROAR_Sim.configurations.configuration import Configuration as CarlaConfig

from ROAR.agent_module.pure_pursuit_agent import PurePursuitAgent
from ROAR.configurations.configuration import Configuration as AgentConfig
import argparse
from misc.utils import str2bool
import carla

#modified
from ROAR_Sim.carla_client.segment_carla_runner import CarlaRunner #custom CarlaRunner
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
    while not carla_runner.terminate:
    

        try:
            spawn_point = util.get_coordinates_from_last_line(Path("./ROAR/datasets/segment_waypoint_test/main.txt"))
            my_vehicle = carla_runner.set_carla_world()
            agent = WaypointGeneratingAgent(vehicle=my_vehicle, agent_settings=agent_config)
            print("spawn_point: ", spawn_point)
            #TODO fix the bug where if u copy in a coord the next line that it prints does not start with a \n 
            if spawn_point:
                carla_runner.world.player.set_transform(spawn_point)
            carla_runner.start_game_loop(agent=agent,
                                        use_manual_control=not args.auto)
            
            with open(Path("./ROAR/datasets/segment_waypoint_test/main.txt"), "r") as file:
                interactive_map_viewer.update(file.read(), agent.waypoints_list)
                file.close()
            choice = interactive_map_viewer.interactive_map(util.get_coords_from_str(agent.waypoints_list[-1]))
            print(agent.waypoints_list[-1])
            if choice == 0:
                print("waypoint saved")
                with open(Path("./ROAR/datasets/segment_waypoint_test/main.txt"), "a") as file:
                    file.writelines(agent.waypoints_list)
                file.close()
            else:
                print("waypoint discarded")
            
                
        except Exception as e:
            logging.error(f"Something bad happened during initialization: {e}")
            carla_runner.on_finish()
            logging.error(f"{e}. Might be a good idea to restart Server")
            raise e



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
