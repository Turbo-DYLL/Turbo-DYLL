import logging
from pathlib import Path
from ROAR_Sim.configurations.configuration import Configuration as CarlaConfig
from ROAR_Sim.carla_client.carla_runner import CarlaRunner
from ROAR.agent_module.pure_pursuit_agent import PurePursuitAgent
from ROAR.configurations.configuration import Configuration as AgentConfig
import argparse
from misc.utils import str2bool
from ROAR.agent_module.forward_only_agent import ForwardOnlyAgent

# my import
from ROAR.agent_module.pid_agent import PIDAgent
from ROAR.agent_module.pid_fast_agent import PIDFastAgent

def main(args):
    """Starts game loop"""
    agent_config = AgentConfig.parse_file(Path("./ROAR/configurations/carla/carla_agent_configuration.json"))
    carla_config = CarlaConfig.parse_file(Path("./ROAR_Sim/configurations/configuration.json"))

    carla_runner = CarlaRunner(carla_settings=carla_config,
                               agent_settings=agent_config,
                               npc_agent_class=PurePursuitAgent,
                               start_bbox=[2530, 99, 4064, 2543, 120, 4076],
                               competition_mode=True,
                               lap_count=1)

    try:
        my_vehicle = carla_runner.set_carla_world()
        print(get_run_time(carla_runner))
        agent = PIDFastAgent(vehicle=my_vehicle,
                         agent_settings=agent_config)
        carla_runner.start_game_loop(agent=agent,
                                     use_manual_control=not args.auto)
        #carla_runner.agent_collision_counter

    except Exception as e:
        logging.error(f"Something bad happened during initialization: {e}")
        carla_runner.on_finish()
        logging.error(f"{e}. Might be a good idea to restart Server")

    finally:
        print("Time: " + str(carla_runner.end_simulation_time - carla_runner.start_simulation_time)) # based off time.time

def get_run_time(carla_runner):
    # Gets the run time in the sim
    return carla_runner.world.carla_world.get_snapshot().elapsed_seconds

if __name__ == "__main__":
    logging.basicConfig(format='%(levelname)s - %(asctime)s - %(name)s '
                               '- %(message)s',
                        datefmt="%H:%M:%S",
                        level=logging.DEBUG)
    logging.getLogger(" streaming client").setLevel(logging.WARNING)
    import warnings

    warnings.filterwarnings("ignore", module="carla")
    parser = argparse.ArgumentParser()
    parser.add_argument("--auto", type=str2bool, default=True, help="True to use auto control")

    warnings.filterwarnings("ignore", module="carla")
    args = parser.parse_args()
    main(args)
