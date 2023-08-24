import utils
from pathlib import Path

from ROAR.control_module.lat_pid_result import LatPIDResult
from ROAR.utilities_module.data_structures_models import Transform, Location, Rotation
from ROAR.utilities_module.vehicle_models import VehicleControl

if not utils.is_dev_mode():
    from ROAR.utilities_module.waypoints import waypoints
else:
    waypoints_path = Path(__file__).parent.parent / "datasets" / "segment_waypoint" / "eric-waypoints-jump.txt"
    waypoints = []
    with open(waypoints_path, "r") as f:
        lines = f.readlines()
        for line in lines:
            waypoints.append(utils.convert_transform_from_str_to_agent(line))


class Control:
    def __init__(self, start_line: int, start_within: float = 10):
        self.start_line = start_line
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
        print(f"Straight Control: {transform.record()}")
        print(f"Straight Control: {lat_pid_result} {current_speed}")
        if lat_pid_result.sharp_error < 0.9 or current_speed <= 90:
            throttle = 1
            brake = 0
        else:
            throttle = -1
            brake = 1

        return VehicleControl(throttle=throttle, steering=lat_pid_result.steering, brake=brake)


class MountainControl(Control):
    def apply_control(self, transform: Transform, lat_pid_result: LatPIDResult, current_speed: float) -> VehicleControl:
        print(f"Mountain Control: {transform.record()}")
        print(f"Mountain Control: {lat_pid_result} {current_speed}")
        if lat_pid_result.sharp_error >= 0.67 and current_speed > 120:
            throttle = 0
            brake = 0.5

            if lat_pid_result.sharp_error > 0.8:
                brake = lat_pid_result.sharp_error / 2

            if lat_pid_result.sharp_error > 1.2:
                brake = 1

        elif lat_pid_result.wide_error > 0.2 and current_speed > 95:  # wide turn
            throttle = max(0, 1 - 6 * pow(lat_pid_result.wide_error + current_speed * 0.003, 8))
            brake = 0
        else:
            throttle = 1
            brake = 0

        return VehicleControl(throttle=throttle, steering=lat_pid_result.steering, brake=brake)


class MountainControl1(Control):
    def apply_control(self, transform: Transform, lat_pid_result: LatPIDResult, current_speed: float) -> VehicleControl:
        print(f"Mountain Control: {transform.record()}")
        print(f"Mountain Control: {lat_pid_result} {current_speed}")
        if lat_pid_result.sharp_error >= 0.67 and current_speed > 80:
            throttle = 0
            brake = 0.3

            if lat_pid_result.sharp_error > 0.8:
                brake = lat_pid_result.sharp_error / 2

            if lat_pid_result.sharp_error > 1.2:
                brake = 1

        elif lat_pid_result.wide_error > 0.15 and current_speed > 92:  # wide turn
            throttle = max(0, 1 - 6 * pow(lat_pid_result.wide_error * 0.8 + current_speed * 0.003, 6))
            brake = 0
        else:
            throttle = 1
            brake = 0

        return VehicleControl(throttle=throttle, steering=lat_pid_result.steering, brake=brake)


class MountainControl2(Control):
    def apply_control(self, transform: Transform, lat_pid_result: LatPIDResult, current_speed: float) -> VehicleControl:
        print(f"Mountain Control2: {transform.record()}")
        print(f"Mountain Control2: {lat_pid_result} {current_speed}")
        if lat_pid_result.sharp_error >= 0.67 and current_speed > 90:
            throttle = 0
            brake = lat_pid_result.sharp_error / 1.8

            if lat_pid_result.sharp_error > 1:
                brake = 1
        elif lat_pid_result.wide_error > 0.3 and current_speed > 92:  # wide turn
            throttle = max(0, 1 - 4 * pow(lat_pid_result.wide_error * 0.9 + current_speed * 0.003, 6))
            brake = 0
        else:
            throttle = 1
            brake = 0

        return VehicleControl(throttle=throttle, steering=lat_pid_result.steering, brake=brake)


class MountainControl3(Control):
    def apply_control(self, transform: Transform, lat_pid_result: LatPIDResult, current_speed: float) -> VehicleControl:
        print(f"Mountain Control3: {transform.record()}")
        print(f"Mountain Control3: {lat_pid_result} {current_speed}")
        if lat_pid_result.sharp_error >= 0.67 and current_speed > 80:
            throttle = 0
            brake = 0.4
            if lat_pid_result.sharp_error > 0.8:
                brake = lat_pid_result.sharp_error / 2
        elif lat_pid_result.wide_error > 0.3 and current_speed > 100:  # wide turn
            throttle = max(0, 1 - 6 * pow(lat_pid_result.wide_error + current_speed * 0.003, 7))
            brake = 0  # min(0.8, 4 * pow(lat_pid_result.wide_error + current_speed * 0.003, 7))
        else:
            throttle = 1
            brake = 0

        return VehicleControl(throttle=throttle, steering=lat_pid_result.steering, brake=brake)


class MountainControl3_5(Control):
    def apply_control(self, transform: Transform, lat_pid_result: LatPIDResult, current_speed: float) -> VehicleControl:
        print(f"Mountain Control3_5: {transform.record()}")
        print(f"Mountain Control3_5: {lat_pid_result} {current_speed}")
        brake = 0
        if lat_pid_result.sharp_error >= 0.5 and current_speed > 90:
            # throttle = 0
            brake = 0.1 + lat_pid_result.sharp_error / 1.5
            # if lat_pid_result.sharp_error > 0.8:
            #     brake = lat_pid_result.sharp_error / 2
        if lat_pid_result.wide_error > 0.2 and current_speed > 100:  # wide turn
            throttle = max(0, 1 - 3 * pow(lat_pid_result.wide_error + current_speed * 0.002, 7))
            # brake = 0  # min(0.8, 4 * pow(lat_pid_result.wide_error + current_speed * 0.003, 7))
        else:
            throttle = 1
            brake = 0

        return VehicleControl(throttle=throttle, steering=lat_pid_result.steering, brake=brake)


class MountainControl4(Control):
    def apply_control(self, transform: Transform, lat_pid_result: LatPIDResult, current_speed: float) -> VehicleControl:
        print(f"Mountain Control4: {transform.record()}")
        print(f"Mountain Control4: {lat_pid_result} {current_speed}")
        if lat_pid_result.sharp_error >= 0.5 and current_speed > 80:
            throttle = 0
            brake = 0.4

            if lat_pid_result.sharp_error > 0.8:
                brake = lat_pid_result.sharp_error / 2
        elif lat_pid_result.wide_error > 0.3 and current_speed > 100:  # wide turn
            throttle = max(0, 1 - 6 * pow(lat_pid_result.wide_error + current_speed * 0.003, 7))
            brake = 0  # min(0.8, 4 * pow(lat_pid_result.wide_error + current_speed * 0.003, 7))
        else:
            throttle = 1
            brake = 0

        return VehicleControl(throttle=throttle, steering=lat_pid_result.steering, brake=brake)


class MountainControl4_5(Control):
    def apply_control(self, transform: Transform, lat_pid_result: LatPIDResult, current_speed: float) -> VehicleControl:
        print(f"Mountain Control4: {transform.record()}")
        print(f"Mountain Control4: {lat_pid_result} {current_speed}")
        if lat_pid_result.sharp_error >= 0.5 and current_speed > 80:
            throttle = 0
            brake = 0.5 + lat_pid_result.sharp_error / 2

            if lat_pid_result.sharp_error > 0.8:
                brake = lat_pid_result.sharp_error / 2
        elif lat_pid_result.wide_error > 0.3 and current_speed > 100:  # wide turn
            throttle = max(0, 1 - 6 * pow(lat_pid_result.wide_error + current_speed * 0.003, 7))
            brake = 0  # min(0.8, 4 * pow(lat_pid_result.wide_error + current_speed * 0.003, 7))
        else:
            throttle = 1
            brake = 0

        return VehicleControl(throttle=throttle, steering=lat_pid_result.steering, brake=brake)


class MountainControl5(Control):
    def apply_control(self, transform: Transform, lat_pid_result: LatPIDResult, current_speed: float) -> VehicleControl:
        print(f"Mountain Control2: {transform.record()}")
        print(f"Mountain Control2: {lat_pid_result} {current_speed}")
        if lat_pid_result.sharp_error >= 0.6 and current_speed > 80:
            throttle = 0
            brake = lat_pid_result.sharp_error / 1.8

            if lat_pid_result.sharp_error > 1:
                brake = 1
        elif lat_pid_result.wide_error > 0.3 and current_speed > 92:  # wide turn
            throttle = max(0, 1 - 4 * pow(lat_pid_result.wide_error * 0.9 + current_speed * 0.003, 6))
            brake = 0
        else:
            throttle = 1
            brake = 0

        if transform.rotation.pitch < -10 and current_speed > 90:
            throttle = throttle * 0.8
            brake = brake * 1.2

        return VehicleControl(throttle=throttle, steering=lat_pid_result.steering, brake=brake)


class MountainControl2_5(Control):
    def apply_control(self, transform: Transform, lat_pid_result: LatPIDResult, current_speed: float) -> VehicleControl:
        print(f"Mountain Control2: {transform.record()}")
        print(f"Mountain Control2: {lat_pid_result} {current_speed}")
        if lat_pid_result.sharp_error >= 0.67 and current_speed > 90:
            throttle = 0
            brake = 0.3 + lat_pid_result.sharp_error / 2

        elif lat_pid_result.wide_error > 0.25 and current_speed > 92:  # wide turn
            throttle = max(0, 1 - 3 * pow(lat_pid_result.wide_error * 0.9 + current_speed * 0.003, 6))
            brake = 0
        else:
            throttle = 1
            brake = 0

        return VehicleControl(throttle=throttle, steering=lat_pid_result.steering, brake=brake)


class MountainControl2_6(Control):
    def apply_control(self, transform: Transform, lat_pid_result: LatPIDResult, current_speed: float) -> VehicleControl:
        print(f"Mountain Control2: {transform.record()}")
        print(f"Mountain Control2: {lat_pid_result} {current_speed}")
        if lat_pid_result.sharp_error >= 0.67 and current_speed > 100:
            throttle = 0
            brake = 0.3 + lat_pid_result.sharp_error / 2

        elif lat_pid_result.wide_error > 0.25 and current_speed > 92:  # wide turn
            throttle = max(0, 1 - 3 * pow(lat_pid_result.wide_error * 0.9 + current_speed * 0.003, 7))
            brake = 0
        else:
            throttle = 1
            brake = 0

        return VehicleControl(throttle=throttle, steering=lat_pid_result.steering, brake=brake)


class MountainControl6(Control):
    def apply_control(self, transform: Transform, lat_pid_result: LatPIDResult, current_speed: float) -> VehicleControl:
        print(f"Mountain Control5: {transform.record()}")
        print(f"Mountain Control5: {lat_pid_result} {current_speed}")
        if lat_pid_result.sharp_error >= 0.67 and current_speed > 80:
            throttle = 0
            brake = 0.4

            if lat_pid_result.sharp_error > 0.8:
                brake = lat_pid_result.sharp_error / 2
        elif lat_pid_result.wide_error > 0.13 and current_speed > 90:  # wide turn
            throttle = max(0, 1 - 6 * pow(lat_pid_result.wide_error + current_speed * 0.003, 7))
            brake = 0
        else:
            throttle = 1
            brake = 0

        if transform.rotation.pitch < -10 and current_speed > 90:
            throttle = throttle * 0.9
            brake = brake * 1.1

        return VehicleControl(throttle=throttle, steering=lat_pid_result.steering, brake=brake)


class RingControl(Control):
    def apply_control(self, transform: Transform, lat_pid_result: LatPIDResult, current_speed: float) -> VehicleControl:
        print(f"Ring Control: {transform.record()}")
        print(f"Ring Control: {lat_pid_result} {current_speed}")
        if lat_pid_result.sharp_error >= 0.5 and current_speed > 80:
            throttle = 0
            brake = lat_pid_result.sharp_error / 2
            if lat_pid_result.sharp_error > 0.8:
                brake = lat_pid_result.sharp_error / 2
        elif lat_pid_result.wide_error > 0.09 and current_speed > 70:  # wide turn
            throttle = max(0, 1 - 3 * pow(lat_pid_result.wide_error + current_speed * 0.003, 7))
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
    BrakeControl(2041),
    MountainControl1(2045),
    BrakeControl(2230),
    MountainControl1(2232),
    MountainControl2(3000),
    BrakeControl(3485),
    MountainControl2(3487),
    MountainControl3(3800),
    BrakeControl(4130),
    MountainControl3(4131),
    BrakeControl(4845),
    MountainControl3(4847),
    BrakeControl(4920),
    MountainControl3_5(4922),
    BrakeControl(5012),
    MountainControl3_5(5014),
    BrakeControl(5700),
    MountainControl4(5705),
    BrakeControl(5770),
    MountainControl4(5775),
    BrakeControl(5940),
    MountainControl4_5(5943),
    BrakeControl(6207),
    MountainControl5(6210),
    MountainControl6(6800),
    MountainControl2_5(7000),
    MountainControl2_6(7500),
    BrakeControl(7645),
    MountainControl2_5(7647),
    BrakeControl(7898),
    MountainControl2_6(7904),
    BrakeControl(8220),
    MountainControl6(8222),
    BrakeControl(8922),
    MountainControl6(8925),
    BrakeControl(9970),
    MountainControl6(9973),
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
