from collections import deque
import numpy as np
import logging
from typing import Tuple, List
import json
from pathlib import Path

import utils
from ROAR.control_module.controller import Controller
from ROAR.utilities_module.vehicle_models import VehicleControl, Vehicle
from ROAR.utilities_module.data_structures_models import Transform
from ROAR.control_module.lat_pid_result import LatPIDResult


class TurboPIDController(Controller):
    def __init__(self, agent, steering_boundary: Tuple[float, float],
                 throttle_boundary: Tuple[float, float], **kwargs):
        super().__init__(agent, **kwargs)
        self.max_speed = self.agent.agent_settings.max_speed
        self.throttle_boundary = throttle_boundary
        self.steering_boundary = steering_boundary
        self.config = json.load(Path(agent.agent_settings.pid_config_file_path).open(mode='r'))
        self.lat_pid_controller = LatPIDController(
            agent=agent,
            config=self.config["latitudinal_controller"],
            steering_boundary=steering_boundary
        )
        self.logger = logging.getLogger(__name__)

    def init_controls(self):
        from ROAR.control_module.controls import controls_sequence, Control
        if utils.is_dev_mode():
            self.control_sequence: List[Control] = controls_sequence
        else:
            self.control_sequence: List[Control] = controls_sequence.copy()

    def run_in_series(self, next_waypoint: Transform, close_waypoint: Transform, far_waypoint: Transform,
                      **kwargs) -> VehicleControl:

        # run lat pid controller
        lat_result = self.lat_pid_controller.run_in_series(next_waypoint=next_waypoint,
                                                           close_waypoint=close_waypoint,
                                                           far_waypoint=far_waypoint)

        current_speed = Vehicle.get_speed(self.agent.vehicle)

        # calculate change in pitch
        pitch = float(next_waypoint.record().split(",")[4])

        if self.control_sequence.__len__() > 1 and self.control_sequence[1].should_start(self.agent.vehicle.transform):
            self.logger.debug(f"Changing control sequence to {self.control_sequence[0].__class__.__name__}")
            self.control_sequence.pop(0)

        vehicle_control = self.control_sequence[0].apply_control(self.agent.vehicle.transform, lat_result, current_speed)
        gear = max(1, int((current_speed - 2 * pitch) / 60))
        if vehicle_control.throttle == -1:
            gear = -1
        vehicle_control.gear = gear
        return vehicle_control

    @staticmethod
    def find_k_values(vehicle: Vehicle, config: dict) -> np.array:
        current_speed = Vehicle.get_speed(vehicle=vehicle)
        k_p, k_d, k_i = 1, 0, 0
        for speed_upper_bound, kvalues in config.items():
            speed_upper_bound = float(speed_upper_bound)
            if current_speed < speed_upper_bound:
                k_p, k_d, k_i = kvalues["Kp"], kvalues["Kd"], kvalues["Ki"]
                break
        return np.array([k_p, k_d, k_i])


class LatPIDController(Controller):
    def __init__(self, agent, config: dict, steering_boundary: Tuple[float, float],
                 dt: float = 0.03, **kwargs):
        super().__init__(agent, **kwargs)
        self.config = config
        self.steering_boundary = steering_boundary
        self._error_buffer = deque(maxlen=10)
        self._dt = dt

    def run_in_series(self, next_waypoint: Transform, close_waypoint: Transform, far_waypoint: Transform,
                      **kwargs) -> LatPIDResult:
        """
        Calculates a vector that represent where you are going.
        Args:
            next_waypoint ():
            **kwargs ():

        Returns:
            lat_control
        """
        # calculate a vector that represent where you are going
        v_begin = self.agent.vehicle.transform.location.to_array()
        direction_vector = np.array([-np.sin(np.deg2rad(self.agent.vehicle.transform.rotation.yaw)),
                                     0,
                                     -np.cos(np.deg2rad(self.agent.vehicle.transform.rotation.yaw))])
        v_end = v_begin + direction_vector

        v_vec = np.array([(v_end[0] - v_begin[0]), 0, (v_end[2] - v_begin[2])])

        # calculate error projection
        w_vec = np.array(
            [
                next_waypoint.location.x - v_begin[0],
                0,
                next_waypoint.location.z - v_begin[2],
            ]
        )

        v_vec_normed = v_vec / np.linalg.norm(v_vec)
        w_vec_normed = w_vec / np.linalg.norm(w_vec)
        # error = np.arccos(v_vec_normed @ w_vec_normed.T)
        error = np.arccos(
            min(max(v_vec_normed @ w_vec_normed.T, -1), 1))  # makes sure arccos input is between -1 and 1, inclusive
        _cross = np.cross(v_vec_normed, w_vec_normed)

        # calculate close error projection
        w_vec = np.array(
            [
                close_waypoint.location.x - v_begin[0],
                0,
                close_waypoint.location.z - v_begin[2],
            ]
        )
        w_vec_normed = w_vec / np.linalg.norm(w_vec)
        # wide_error = np.arccos(v_vec_normed @ w_vec_normed.T)
        wide_error = np.arccos(
            min(max(v_vec_normed @ w_vec_normed.T, -1), 1))  # makes sure arccos input is between -1 and 1, inclusive

        # calculate far error projection
        w_vec = np.array(
            [
                far_waypoint.location.x - v_begin[0],
                0,
                far_waypoint.location.z - v_begin[2],
            ]
        )
        w_vec_normed = w_vec / np.linalg.norm(w_vec)
        # sharp_error = np.arccos(v_vec_normed @ w_vec_normed.T)
        sharp_error = np.arccos(
            min(max(v_vec_normed @ w_vec_normed.T, -1), 1))  # makes sure arccos input is between -1 and 1, inclusive

        if _cross[1] > 0:
            error *= -1
        self._error_buffer.append(error)
        if len(self._error_buffer) >= 2:
            _de = (self._error_buffer[-1] - self._error_buffer[-2]) / self._dt
            _ie = sum(self._error_buffer) * self._dt
        else:
            _de = 0.0
            _ie = 0.0

        k_p, k_d, k_i = TurboPIDController.find_k_values(config=self.config, vehicle=self.agent.vehicle)

        lat_control = float(
            np.clip((k_p * error) + (k_d * _de) + (k_i * _ie), self.steering_boundary[0], self.steering_boundary[1])
        )
        return LatPIDResult(steering=lat_control, error=error, wide_error=wide_error, sharp_error=sharp_error)
