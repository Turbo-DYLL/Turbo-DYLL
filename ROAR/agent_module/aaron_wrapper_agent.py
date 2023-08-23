from typing import List

from ROAR.agent_module.agent import Agent
from ROAR.utilities_module.data_structures_models import Location


class ArronWrapperAgent(Agent):
    def __init__(self, agent_class, waypoint_record_list: List[Location], carla_runner, **kwargs):
        super().__init__(**kwargs)
        self.agent = agent_class(**kwargs)
        self.waypoint_record_list = waypoint_record_list
        self.time_list = []
        self.carla_world = carla_runner.world.carla_world

    def run_step(self, vehicle, sensors_data):
        super().run_step(vehicle=vehicle, sensors_data=sensors_data)
        if self.vehicle.transform.location.distance(self.waypoint_record_list[0]) < 10:
            self.waypoint_record_list.pop(0)
            self.time_list.append(self.carla_world.get_snapshot().elapsed_seconds)
            if len(self.waypoint_record_list) == 0:
                raise Exception("End of path")

        return self.agent.run_step(vehicle, sensors_data)