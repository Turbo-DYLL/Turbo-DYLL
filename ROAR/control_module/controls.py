from pathlib import Path

from ROAR.control_module.lat_pid_result import LatPIDResult
from ROAR.utilities_module.data_structures_models import Transform, Location, Rotation
from ROAR.utilities_module.vehicle_models import VehicleControl
from ROAR.utilities_module.waypoints import waypoints


class Control:
    def __init__(self, start_line: int, start_within: float = 10):
        self._start_location = waypoints[start_line - 1].location
        self._start_within = start_within

    def get_start_location(self):
        return self._start_location

    def should_start(self, transform: Transform) -> bool:
        return transform.location.distance(self._start_location) <= self._start_within

    def apply_control(self, transform: Transform, lat_pid_result: LatPIDResult, current_speed: float) -> VehicleControl:
        raise NotImplementedError


class BrakeControl(Control):
    def apply_control(self, transform: Transform, lat_pid_result: LatPIDResult, current_speed: float) -> VehicleControl:
        return VehicleControl(throttle=-1, steering=lat_pid_result.steering, brake=1)


class StraightControl(Control):
    def __init__(self, start_line: int):
        super().__init__(start_line)

    def apply_control(self, transform: Transform, lat_pid_result: LatPIDResult, current_speed: float) -> VehicleControl:
        if lat_pid_result.sharp_error < 0.9 or current_speed <= 90:
            throttle = 1
            brake = 0
        else:
            throttle = -1
            brake = 1

        return VehicleControl(throttle=throttle, steering=lat_pid_result.steering, brake=brake)


class MountainControl(Control):
    def apply_control(self, transform: Transform, lat_pid_result: LatPIDResult, current_speed: float) -> VehicleControl:
        if lat_pid_result.sharp_error >= 0.67 and current_speed > 70:
            throttle = 0
            brake = 0.4
        elif lat_pid_result.wide_error > 0.09 and current_speed > 92:  # wide turn
            throttle = max(0, 1 - 6 * pow(lat_pid_result.wide_error + current_speed * 0.003, 6))
            brake = 0
        else:
            throttle = 1
            brake = 0

        return VehicleControl(throttle=throttle, steering=lat_pid_result.steering, brake=brake)


controls_sequence = [
    StraightControl(0),
    BrakeControl(1037, 27),
    StraightControl(1067),
    MountainControl(1367)
]
