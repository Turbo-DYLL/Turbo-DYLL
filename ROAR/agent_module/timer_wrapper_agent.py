from ROAR.agent_module.agent import Agent
from ROAR.utilities_module.data_structures_models import Location


class TimerWrapperAgent(Agent):
    def __init__(self, agent_class, end_point: Location, **kwargs):
        super().__init__(**kwargs)
        self.agent = agent_class(**kwargs)
        self.end_point = end_point

    def run_step(self, vehicle, sensors_data):
        super().run_step(vehicle=vehicle, sensors_data=sensors_data)
        if self.vehicle.transform.location.distance(self.end_point) < 10:
            raise Exception("End of path")
        return self.agent.run_step(vehicle, sensors_data)