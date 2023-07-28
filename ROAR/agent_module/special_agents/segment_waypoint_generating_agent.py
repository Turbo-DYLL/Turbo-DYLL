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
        self.waypoints_list.append(self.vehicle.transform.record() + "\n")   
        self.last_waypoint = self.vehicle.transform

    def run_step(self, sensors_data: SensorsData,
                 vehicle: Vehicle) -> VehicleControl:
        super(WaypointGeneratingAgent, self).run_step(sensors_data=sensors_data,
                                                     vehicle=vehicle)
        # print(str)
        # print(self.waypoints_list[-1])
        # #self.get_distance(str(self.waypoints_list[-1]), str(self.vehicle.transform.record())) > 1:         
        if self.time_counter > 1:
            #print(f"Writing to [{self.output_file_path}]: {self.vehicle.transform}")
            if self.last_waypoint == None:
                self.last_waypoint = self.vehicle.transform
                if self.vehicle.transform.location.x != 0:
                    self.waypoints_list.append(self.vehicle.transform.record() + "\n")
                print('run_step')
            else:
                distance = self.vehicle.transform.location.distance(self.last_waypoint.location)
                if distance >= 1:
                    self.waypoints_list.append(self.vehicle.transform.record() + "\n")
                    self.last_waypoint = self.vehicle.transform
                    print(f"Writing to array: {self.vehicle.transform}")
        return VehicleControl()
    

    


    #TODO Change Run step in to Distance based instead of 