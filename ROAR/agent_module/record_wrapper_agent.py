from typing import List

from ROAR.agent_module.agent import Agent
from ROAR.utilities_module.data_structures_models import Location


class RecordWrapperAgent(Agent):
    def __init__(self, agent_class, end_location: Location, waypoint_record_list: List[Location], carla_runner, **kwargs):
        super().__init__(**kwargs)
        self.agent = agent_class(**kwargs)
        self.waypoint_record_list = waypoint_record_list
        self.time_list = []
        self.carla_world = carla_runner.world.carla_world
        if end_location is not None:
            self.end_location = end_location
        else:
            self.end_location = waypoint_record_list[-1]

    def run_step(self, vehicle, sensors_data):
        super().run_step(vehicle=vehicle, sensors_data=sensors_data)
        if self.vehicle.transform.location.distance(self.waypoint_record_list[0]) < 10:
            self.waypoint_record_list.pop(0)
            self.time_list.append(self.carla_world.get_snapshot().elapsed_seconds)
        if self.vehicle.transform.location.distance(self.end_location) < 10:
            raise Exception("End of path")

        return self.agent.run_step(vehicle, sensors_data)