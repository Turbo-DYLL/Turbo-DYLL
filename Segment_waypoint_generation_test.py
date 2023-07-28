import logging
from pathlib import Path
from ROAR_Sim.configurations.configuration import Configuration as CarlaConfig
from ROAR_Sim.carla_client.carla_runner import CarlaRunner
from ROAR.agent_module.pure_pursuit_agent import PurePursuitAgent
from ROAR.configurations.configuration import Configuration as AgentConfig
import argparse
from misc.utils import str2bool
import carla


from ROAR.agent_module.special_agents.segment_waypoint_generating_agent import WaypointGeneratingAgent

def get_coordinates_from_last_line(file_path):
    file = open(file_path, 'r')
    lines = file.readlines()
    file.close()
    last_line = lines[-1].strip()
    coordinates = last_line.split(',')[:3]
    x, y, z = map(float, coordinates)

    return x, y, z

def main(args):
    """Starts game loop"""
    agent_config = AgentConfig.parse_file(Path("./ROAR/configurations/carla/carla_agent_configuration.json"))
    carla_config = CarlaConfig.parse_file(Path("./ROAR_Sim/configurations/configuration.json"))

    carla_runner = CarlaRunner(carla_settings=carla_config,
                               agent_settings=agent_config,
                               npc_agent_class=PurePursuitAgent)
    try:
        my_vehicle = carla_runner.set_carla_world()
        # agent = ForwardOnlyAgent(vehicle=my_vehicle,
                         # agent_settings=agent_config)
        x1,y1,z1 = get_coordinates_from_last_line(Path("./ROAR/datasets/segment_waypoint_test/main.txt"))
        print(f'x1: {x1} y1: {y1} z1: {z1}')
        agent = WaypointGeneratingAgent(vehicle=my_vehicle, agent_settings=agent_config)
    
        carla_runner.world.player.set_location(carla.Location(x=x1,z=y1,y=z1))
        carla_runner.start_game_loop(agent=agent,
                                     use_manual_control=not args.auto)
        
        ans = input("Do you want to save waypoints to main.txt? [Y/n]")
        if ans == "Y" or ans == "y":
            print("waypoint saved")
            with open(Path("./ROAR/datasets/segment_waypoint_test/main.txt"), "a") as file:
                file.writelines(agent.waypoints_list)
        else:
            print("waypoint discarded")

            
        print("LoopComplete")

    except Exception as e:
        logging.error(f"Something bad happened during initialization: {e}")
        carla_runner.on_finish()
        logging.error(f"{e}. Might be a good idea to restart Server")


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