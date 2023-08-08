from pathlib import Path

from ROAR.control_module.lat_pid_result import LatPIDResult
from ROAR.utilities_module.data_structures_models import Transform, Location, Rotation
from ROAR.utilities_module.vehicle_models import VehicleControl
from ROAR.utilities_module.waypoints import waypoints


class Control:
    def __init__(self, start_line: int):
        self._start_location = waypoints[start_line - 1].location

    def get_start_location(self):
        return self._start_location

    def is_arrived(self, transform: Transform) -> bool:
        return transform.location.distance(self._start_location) <= 10

    def apply_control(self, transform: Transform, lat_pid_result: LatPIDResult, current_speed: float) -> VehicleControl:
        raise NotImplementedError


class BrakeControl(Control):
    def apply_control(self, transform: Transform, lat_pid_result: LatPIDResult, current_speed: float) -> VehicleControl:
        return VehicleControl(throttle=-1, steering=lat_pid_result.steering, brake=1)

    def is_arrived(self, transform: Transform) -> bool:
        return transform.location.distance(self._start_location) <= 25


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
    def __init__(self, start_line: int):
        super().__init__(start_line)
        self.waypoint_queue_region = []
        self.brake_counter = 0
        # region_list_path = Path("./ROAR/control_module/region_list.txt")
        region_list_path = Path("./ROAR/datasets/control/region_list.txt")
        # braking_list_path = Path("./ROAR/control_module/braking_list_mod.txt")
        braking_list_path = Path("./ROAR/datasets/control/braking_list.txt")
        with open(region_list_path) as f:
            for line in f:
                raw = line.split(",")
                waypoint = Transform(location=Location(x=raw[0], y=raw[1], z=raw[2]),
                                     rotation=Rotation(pitch=0, yaw=0, roll=0))
                self.waypoint_queue_region.append(waypoint)

        self.waypoint_queue_braking = []
        with open(braking_list_path) as f:
            for line in f:
                raw = line.split(",")
                waypoint = Transform(location=Location(x=raw[0], y=raw[1], z=raw[2]),
                                     rotation=Rotation(pitch=0, yaw=0, roll=0))
                self.waypoint_queue_braking.append(waypoint)

    def apply_control(self, transform: Transform, lat_pid_result: LatPIDResult, current_speed: float) -> VehicleControl:
        waypoint = self.waypoint_queue_braking[0]  # 5012 is weird bump spot
        dist = transform.location.distance(waypoint.location)
        if dist <= 5:
            self.brake_counter = 1
            # print(self.waypoint_queue_braking[0])
            self.waypoint_queue_braking.pop(0)
        if self.brake_counter > 0:
            throttle = -1
            brake = 1
            self.brake_counter += 1
            if self.brake_counter >= 8:
                self.brake_counter = 0
        elif lat_pid_result.sharp_error >= 0.67 and current_speed > 70:
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
    BrakeControl(1037),
    StraightControl(1067),
    MountainControl(1367)
]
