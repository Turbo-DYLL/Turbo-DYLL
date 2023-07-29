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
    with open(file_path, "r") as file:
        lines = file.readlines()
    if len(lines) == 0:
        return None
    overwrite = input("Do you want to overwrite waypoints? [Y/n]").upper() == "Y"
    if overwrite:
        with open(file_path, "w") as file:
            file.write("")
        return None
    last_line = lines[-1].strip()
    coordinates = last_line.split(',')
    x, y, z, roll, pitch, yaw = map(float, coordinates)
    # TODO: fix this, when uncommented, it will cause the car to spawn in the air
    return carla.Transform(carla.Location(x=x, y=z, z=y), convert_rotation_from_agent_to_source_real(roll, pitch, yaw))


def convert_rotation_from_agent_to_source_real(roll, pitch, yaw) -> carla.Rotation:
    roll, pitch, yaw = roll, pitch, -yaw
    if yaw <= 0:
        yaw = yaw + 270
    else:
        yaw = yaw - 90
    return carla.Rotation(roll=roll, pitch=pitch, yaw=yaw)


def main(args):
    """Starts game loop"""
    agent_config = AgentConfig.parse_file(Path("./ROAR/configurations/carla/carla_agent_configuration.json"))
    carla_config = CarlaConfig.parse_file(Path("./ROAR_Sim/configurations/configuration.json"))

    carla_runner = CarlaRunner(carla_settings=carla_config,
                               agent_settings=agent_config,
                               npc_agent_class=PurePursuitAgent)
    try:
        spawn_point = get_coordinates_from_last_line(Path("./ROAR/datasets/segment_waypoint_test/main.txt"))
        my_vehicle = carla_runner.set_carla_world()
        agent = WaypointGeneratingAgent(vehicle=my_vehicle, agent_settings=agent_config)
        print("spawn_point: ", spawn_point)
        if spawn_point:
            carla_runner.world.player.set_transform(spawn_point)
        carla_runner.start_game_loop(agent=agent,
                                     use_manual_control=not args.auto)
        # carla_runner.on_finish()
        ans = input("Do you want to save waypoints to main.txt? [Y/n]")
        if ans.upper() == "Y":
            print("waypoint saved")
            with open(Path("./ROAR/datasets/segment_waypoint_test/main.txt"), "a") as file:
                file.writelines(agent.waypoints_list)
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
