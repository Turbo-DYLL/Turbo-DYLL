from collections import deque
import numpy as np
import logging
from typing import Tuple
import json
from pathlib import Path

from ROAR.control_module.controller import Controller
from ROAR.utilities_module.vehicle_models import VehicleControl, Vehicle
from ROAR.utilities_module.data_structures_models import Transform, Location, Rotation
from ROAR.utilities_module.pid_functions import *


class TurboPIDController(Controller):
    def __init__(self, agent, steering_boundary: Tuple[float, float],
                 throttle_boundary: Tuple[float, float], **kwargs):
        super().__init__(agent, **kwargs)
        self.max_speed = self.agent.agent_settings.max_speed
        throttle_boundary = throttle_boundary
        self.steering_boundary = steering_boundary
        self.config = json.load(Path(agent.agent_settings.pid_config_file_path).open(mode='r'))

        # useful variables
        self.region = 1
        self.brake_counter = 0

        self.lat_pid_controller = LatPIDController(
            agent=agent,
            config=self.config["latitudinal_controller"],
            steering_boundary=steering_boundary
        )
        self.logger = logging.getLogger(__name__)
        self.location = Location.from_array([2107.3212890625,117.31633758544922,3417.138671875])



        ## Sectional PID
        self.current_waypoint = 0
        self.section_id = 0
        sectional_pid_path = Path("./ROAR/datasets/sectional_waypoint/text.txt")
        with open(sectional_pid_path) as f:
            self.sectional_pid_list = list(map(SectionalPID, f.split('///\n')))
        
        self.current_pid = self.sectional_pid_list[0]
        self.last_pid = self.sectional_pid_list[0]

    def run_in_series(self, next_waypoint: Transform, close_waypoint: Transform, far_waypoint: Transform,
                      **kwargs) -> VehicleControl:

        # run lat pid controller
        steering, error, wide_error, sharp_error = self.lat_pid_controller.run_in_series(next_waypoint=next_waypoint,
                                                                                         close_waypoint=close_waypoint,
                                                                                         far_waypoint=far_waypoint)

        print(next_waypoint.record())

        current_speed = Vehicle.get_speed(self.agent.vehicle)

        # get errors from lat pid
        error = abs(round(error, 3))
        wide_error = abs(round(wide_error, 3))
        sharp_error = abs(round(sharp_error, 3))
        # print(error, wide_error, sharp_error)

        # calculate change in pitch
        pitch = float(next_waypoint.record().split(",")[4])


        print(f"sharp: {sharp_error}, speed: {current_speed}")

        
        self.current_waypoint += 1
        # SectionalPid

        #change to next sectional PID
        next_pid_change = self.sectional_pid_list[self.section_id].array[0][0]
        if self.current_waypoint == next_pid_change:
            self.last_pid = self.current_pid
            self.current_pid = self.sectional_pid_list[self.section_id]
            self.sectional_id += 1



        for index in range(5):
            sub_pid = self.current_pid.array[index]
            if sub_pid[-1] == 1:
                if sharp_turn(sharp_error, current_speed, sub_pid[1], sub_pid[2],sub_pid[3],sub_pid[4]) != None:
                    brake,throttle = sharp_turn(sharp_error, current_speed, sub_pid[1], sub_pid[2],sub_pid[3],sub_pid[4])
            elif sub_pid[-1] == 2:
                if wide_turn(sharp_error, current_speed, sub_pid[1], sub_pid[2],sub_pid[3],sub_pid[4]) != None:
                    brake,throttle = wide_turn(sharp_error, current_speed, sub_pid[1], sub_pid[2],sub_pid[3],sub_pid[4])
            elif sub_pid[-1] == 3:
                brake,throttle = sub_pid[3],sub_pid[4]
                sub_pid[1] -= 1
                if sub_pid[1] == 0:
                    self.current_pid = self.last_pid
            else:
                brake,throttle = 0,1
                              
        
        gear = max(1, int((current_speed - 2 * pitch) / 60))
        if throttle < -1:
            gear = -1
        throttle = -throttle
        return VehicleControl(throttle=throttle, steering=steering, brake=brake, gear=gear)

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


# a = list of SectionalPid()
# i = 0
# if a[i].waypoint == current waypoint then current pid = a[i]  i += 1


class SectionalPID:
        
    def __init__(self, file_path_str):
        """
        file_path_str format:
        mode 1(sharp turn) : (waypoint number, max sharp error, current speed, wide, brake, throttle, mode)
        mode 2(wide turn)  : (waypoint number, max wide error, current speed, wide, brake, throttle, mode)
        mode 3(manual break)      : (waypoint nunmber, duration of brake, _ , wide, brake, throttle, mode)

        
        ... up to 5
        """
        lines = file_path_str.readlines()
        self.array = [[-1 for _ in range(6)] for _ in range(5)]
        #creates 5x5 numpy array
        row = 0
        for line in lines:
            print(line.strip("() \n").split(","))
            a,b,c,d,e = map(float, line.strip("() \n").split(","))
            self.array[row] = [a,b,c,d,e]
            row += 1
            

class LatPIDController(Controller):
    def __init__(self, agent, config: dict, steering_boundary: Tuple[float, float],
                 dt: float = 0.03, **kwargs):
        super().__init__(agent, **kwargs)
        self.config = config
        self.steering_boundary = steering_boundary
        self._error_buffer = deque(maxlen=10)
        self._dt = dt

    def run_in_series(self, next_waypoint: Transform, close_waypoint: Transform, far_waypoint: Transform,
                      **kwargs) -> float:
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
        return lat_control, error, wide_error, sharp_error

    