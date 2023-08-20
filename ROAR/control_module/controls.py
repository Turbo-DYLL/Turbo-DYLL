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
        print(transform.record())
        print(lat_pid_result, current_speed)
        if lat_pid_result.sharp_error >= 0.67 and current_speed > 80:
            throttle = 0
            brake = 0.4

            if lat_pid_result.sharp_error > 0.8:
                brake = lat_pid_result.sharp_error / 2

            if lat_pid_result.sharp_error > 1.2:
                brake = 1

        elif lat_pid_result.wide_error > 0.09 and current_speed > 92:  # wide turn
            throttle = max(0, 1 - 6 * pow(lat_pid_result.wide_error + current_speed * 0.003, 6))
            brake = 0
        else:
            throttle = 1
            brake = 0

        return VehicleControl(throttle=throttle, steering=lat_pid_result.steering, brake=brake)


class MountainControl2(Control):
    def apply_control(self, transform: Transform, lat_pid_result: LatPIDResult, current_speed: float) -> VehicleControl:
        print(transform.record())
        print(lat_pid_result, current_speed)
        if lat_pid_result.sharp_error >= 0.67 and current_speed > 90:
            throttle = 0
            brake = lat_pid_result.sharp_error / 1.8

            if lat_pid_result.sharp_error > 1:
                brake = 1
        elif lat_pid_result.wide_error > 0.09 and current_speed > 92:  # wide turn
            throttle = max(0, 1 - 3 * pow(lat_pid_result.wide_error + current_speed * 0.003, 6))
            brake = 0
        else:
            throttle = 1
            brake = 0

        return VehicleControl(throttle=throttle, steering=lat_pid_result.steering, brake=brake)


class MountainControl3(Control):
    def apply_control(self, transform: Transform, lat_pid_result: LatPIDResult, current_speed: float) -> VehicleControl:
        print(transform.record())
        print(lat_pid_result, current_speed)
        if lat_pid_result.sharp_error >= 0.67 and current_speed > 80:
            throttle = 0
            brake = 0.4
            if lat_pid_result.sharp_error > 0.8:
                brake = lat_pid_result.sharp_error / 2
        elif lat_pid_result.wide_error > 0.09 and current_speed > 100:  # wide turn
            throttle = max(0, 1 - 6 * pow(lat_pid_result.wide_error + current_speed * 0.003, 7))
            brake = 0  # min(0.8, 4 * pow(lat_pid_result.wide_error + current_speed * 0.003, 7))
        else:
            throttle = 1
            brake = 0

        return VehicleControl(throttle=throttle, steering=lat_pid_result.steering, brake=brake)


class MountainControl4(Control):
    def apply_control(self, transform: Transform, lat_pid_result: LatPIDResult, current_speed: float) -> VehicleControl:
        print(transform.record())
        print(lat_pid_result, current_speed)
        if lat_pid_result.sharp_error >= 0.5 and current_speed > 80:
            throttle = 0
            brake = 0.4

            if lat_pid_result.sharp_error > 0.8:
                brake = lat_pid_result.sharp_error / 2
        elif lat_pid_result.wide_error > 0.09 and current_speed > 100:  # wide turn
            throttle = max(0, 1 - 6 * pow(lat_pid_result.wide_error + current_speed * 0.003, 7))
            brake = 0  # min(0.8, 4 * pow(lat_pid_result.wide_error + current_speed * 0.003, 7))
        else:
            throttle = 1
            brake = 0

        return VehicleControl(throttle=throttle, steering=lat_pid_result.steering, brake=brake)


class MountainControl5(Control):
    def apply_control(self, transform: Transform, lat_pid_result: LatPIDResult, current_speed: float) -> VehicleControl:
        print(transform.record())
        print(lat_pid_result, current_speed)
        if lat_pid_result.sharp_error >= 0.67 and current_speed > 80:
            throttle = 0
            brake = 0.4

            if lat_pid_result.sharp_error > 0.8:
                brake = lat_pid_result.sharp_error / 2
        elif lat_pid_result.wide_error > 0.09 and current_speed > 90:  # wide turn
            throttle = max(0, 1 - 6 * pow(lat_pid_result.wide_error + current_speed * 0.003, 7))
            brake = 0
        else:
            throttle = 1
            brake = 0

        if transform.rotation.pitch < -10:
            throttle = throttle * 0.8
            brake = brake * 1.2

        return VehicleControl(throttle=throttle, steering=lat_pid_result.steering, brake=brake)


class RingControl(Control):
    def apply_control(self, transform: Transform, lat_pid_result: LatPIDResult, current_speed: float) -> VehicleControl:
        print(transform.record())
        print(lat_pid_result, current_speed)
        if lat_pid_result.sharp_error >= 0.5 and current_speed > 80:
            throttle = 0
            brake = lat_pid_result.sharp_error / 2
            if lat_pid_result.sharp_error > 0.8:
                brake = lat_pid_result.sharp_error / 2
        elif lat_pid_result.wide_error > 0.09 and current_speed > 70:  # wide turn
            throttle = max(0, 1 - 6 * pow(lat_pid_result.wide_error + current_speed * 0.003, 7))
            brake = 0
        else:
            throttle = 1
            brake = 0

        return VehicleControl(throttle=throttle, steering=lat_pid_result.steering, brake=brake)


controls_sequence = [
    StraightControl(0),
    BrakeControl(1037, 27),
    StraightControl(1067),
    MountainControl(1367),
    BrakeControl(2047),
    MountainControl(2048),
    MountainControl2(3000),
    MountainControl3(4000),
    BrakeControl(4130),
    MountainControl3(4132),
    BrakeControl(4845),
    MountainControl3(4847),
    BrakeControl(5700),
    MountainControl4(5705),
    BrakeControl(5770),
    MountainControl4(5775),
    BrakeControl(5940),
    MountainControl2(5945),
    BrakeControl(7920),
    MountainControl5(7925),
    BrakeControl(8920),
    MountainControl5(8925),
    MountainControl(11000),
    BrakeControl(11025),
    MountainControl(11027),
    StraightControl(11265),
    MountainControl(11910),
    BrakeControl(12172),
    RingControl(12185),
    MountainControl(12290),
    # BrakeControl(12188),
    # MountainControl(12190),
    # BrakeControl(12240),
    # MountainControl(12241),
]
