from ROAR.agent_module.agent import Agent
from ROAR.utilities_module.data_structures_models import SensorsData
from ROAR.utilities_module.vehicle_models import Vehicle, VehicleControl
from ROAR.configurations.configuration import Configuration as AgentConfig
from pathlib import Path
import math


class WaypointGeneratingAgent(Agent):
    
    def __init__(self, vehicle: Vehicle, agent_settings: AgentConfig, **kwargs):
        super().__init__(vehicle=vehicle, agent_settings=agent_settings, **kwargs)
        self.output_file_path: Path = self.output_folder_path / "easy_map_waypoints.txt"
        self.waypoints_list = []
        print(f"Writing to array: {self.vehicle.transform}")
        self.waypoints_list.append(self.vehicle.transform.record() + '\n')

    def get_distance(last_location, current_location):
        #returns distance between last location and current location
        last_coordinates = last_location.split(',')[:3]
        x1, z1, y1 = map(float, last_coordinates)
        current_coordinates = last_location.split(',')[:3]
        x2, z2, y2 = map(float, current_coordinates)
        # distance = self.vehicle.transform.location.distance(self.last_waypoint.location)
        distance = math.sqrt((x1-x2)**2+(z1-z2)**2+(y1-y2)**2) #distance formula of two point
        return distance

    def run_step(self, sensors_data: SensorsData,
                 vehicle: Vehicle) -> VehicleControl:
        super(WaypointGeneratingAgent, self).run_step(sensors_data=sensors_data,
                                                     vehicle=vehicle)
        i = 1000

        # print(str)
        # print(self.waypoints_list[-1])         
        if self.get_distance(str(self.waypoints_list[-1]), str(self.vehicle.transform.record())) > 1:
            print(f"Writing to array: {self.vehicle.transform}")
            self.waypoints_list.append(self.vehicle.transform.record() + '\n')
        return VehicleControl()
    

    


    #TODO Change Run step in to Distance based instead of 