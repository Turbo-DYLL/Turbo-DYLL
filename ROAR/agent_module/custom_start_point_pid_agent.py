from pathlib import Path

import util
from ROAR.agent_module.agent import Agent
from ROAR.control_module.turbo_pid_controller import TurboPIDController
from ROAR.planning_module.local_planner.turbo_waypoint_following_local_planner import TurboWaypointFollowingLocalPlanner
from ROAR.planning_module.behavior_planner.behavior_planner import BehaviorPlanner
from ROAR.planning_module.mission_planner.waypoint_following_mission_planner import WaypointFollowingMissionPlanner
from ROAR.utilities_module.data_structures_models import SensorsData
from ROAR.utilities_module.vehicle_models import VehicleControl, Vehicle
import ROAR.utilities_module.waypoints as waypoints
import logging


class TurboPIDAgent(Agent):
    def __init__(self, target_speed=40, **kwargs):
        super().__init__(**kwargs)
        self.target_speed = target_speed
        self.logger = logging.getLogger("PID Agent")
        self.route_file_path = Path(self.agent_settings.waypoint_file_path)
        self.pid_controller = TurboPIDController(agent=self, steering_boundary=(-1, 1), throttle_boundary=(0, 1))
        self.mission_planner = WaypointFollowingMissionPlanner(agent=self)
        # initiated right after mission plan

        self.behavior_planner = BehaviorPlanner(agent=self)
        self.local_planner = TurboWaypointFollowingLocalPlanner(
            spawn_point_id=self.agent_settings.spawn_point_id,
            agent=self,
            controller=self.pid_controller,
            mission_planner=self.mission_planner,
            behavior_planner=self.behavior_planner,
            closeness_threshold=1)
        waypoints_path = Path("./ROAR/datasets/segment_waypoint/eric-waypoints-jump.txt")
        with open(waypoints_path, "r") as waypoint_file:
            waypoint_list = waypoint_file.readlines()

        waypoint_list = [util.convert_transform_from_str_to_agent(transform_str) for transform_str in waypoint_list]
        waypoints.waypoints = waypoint_list.copy()
        self.pid_controller.init_controls()
        self.logger.debug(
            f"Waypoint Following Agent Initiated. Reading f"
            f"rom {self.route_file_path.as_posix()}")

    def run_step(self, vehicle: Vehicle,
                 sensors_data: SensorsData) -> VehicleControl:
        super().run_step(vehicle=vehicle, sensors_data=sensors_data)
        self.transform_history.append(self.vehicle.transform)
        # print(self.vehicle.transform, self.vehicle.velocity)
        if self.is_done:
            control = VehicleControl()
            self.logger.debug("Path Following Agent is Done. Idling.")
        else:
            control = self.local_planner.run_in_series()
        return control
