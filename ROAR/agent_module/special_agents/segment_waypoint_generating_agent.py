from ROAR.agent_module.agent import Agent
from ROAR.utilities_module.data_structures_models import SensorsData
from ROAR.utilities_module.vehicle_models import Vehicle, VehicleControl
from ROAR.configurations.configuration import Configuration as AgentConfig
from pathlib import Path


class WaypointGeneratingAgent(Agent):
    
    def __init__(self, vehicle: Vehicle, agent_settings: AgentConfig, **kwargs):
        super().__init__(vehicle=vehicle, agent_settings=agent_settings, **kwargs)
        self.last_waypoint = None
        self.output_file_path: Path = self.output_folder_path / "easy_map_waypoints.txt"
        self.waypoints_list = []

    def run_step(self, sensors_data: SensorsData,
                 vehicle: Vehicle) -> VehicleControl:
        super(WaypointGeneratingAgent, self).run_step(sensors_data=sensors_data, vehicle=vehicle)
        if self.time_counter > 1:
            if not self.last_waypoint:
                location = self.vehicle.transform.location
                if location.x == 0 and location.y == 0 and location.z == 0:
                    return VehicleControl()
                self.last_waypoint = self.vehicle.transform
                self.waypoints_list.append(self.vehicle.transform.record() + "\n")
            distance = self.vehicle.transform.location.distance(self.last_waypoint.location)
            if distance >= 1:
                self.waypoints_list.append(self.vehicle.transform.record() + "\n")
                self.last_waypoint = self.vehicle.transform
                print(f"Writing to array: {self.vehicle.transform}")
        return VehicleControl()
