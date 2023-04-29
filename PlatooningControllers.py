import carla
from collections import deque
from agents.navigation import controller
import numpy as np
from agents.tools.misc import draw_waypoints


class FollowerController:
    """Controller for follower vehicles. Desired speed is given by a user-defined function at every time step.
    Spatial trajectory is inherited from the lead vehicle"""

    def __init__(self, vehicle, control_function, platoon, handbrake_on_stop=False, parameters=None, dependencies=None,
                 pid_args_lateral=None, pid_args_longitudinal=None, max_brake=0.3, max_throttle=1):
        # parameters: ORDERED list of 2(or more)-tuples of vehicle index and attribute(s), index 0 is ego
        # alternative: dependencies: a list of indices with 0 = ego
        self.control_function = control_function
        self.parameters = parameters
        self.dependencies = dependencies
        if self.dependencies is not None and self.parameters is not None:
            raise Exception("Only one of parameters / dependencies should be defined.")

        if pid_args_lateral is None:
            pid_args_lateral = {'K_P': 1.95, 'K_I': 0.05, 'K_D': 0.2, 'dt': 0.01}

        if pid_args_longitudinal is None:
            pid_args_longitudinal = {'K_P': 2 / 5, 'K_I': 0.26 / 2, 'K_D': 0.26 / 3, 'dt': 0.01}

        self.vehicle = vehicle
        self.pid = controller.VehiclePIDController(self.vehicle,
                                                   args_lateral=pid_args_lateral,
                                                   args_longitudinal=pid_args_longitudinal,
                                                   max_brake=max_brake, max_throttle=max_throttle)
        self.platoon = platoon
        self.target_speed = 0
        self.handbrake_on_stop = handbrake_on_stop

    # self.waypoints_to_track = deque()

    def compute_target_speed(self, index):
        if self.parameters is not None:
            _args = [getattr(self.platoon[index + p[0]], *p[1:]) for p in self.parameters]
        else:
            _args = [self.platoon[index + d] for d in self.dependencies]
        self.target_speed = self.control_function(*_args)

    def compute_control(self, lead_waypoints, index):  # index is ego vehicle index
        # find next waypoint
        _passed = 0
        for i, wp in enumerate(lead_waypoints):
            if self.vehicle.get_location().distance(wp.transform.location) < 5:
                _passed = max(_passed, i + 1)  # warning: this causes problems in a loop-like trajectory

        # last vehicle removes waypoints that it passes
        if index == len(self.platoon) - 1:
            for i in range(_passed):
                lead_waypoints.popleft()
            _passed = 0

        if self.handbrake_on_stop:
            v = self.vehicle.get_velocity()
            if v.x**2 + v.y**2 + v.z**2 < 0.1 and self.target_speed < 0.1:
                return carla.VehicleControl(throttle=0, hand_brake=True, brake=1)
            else:
                self.vehicle.apply_control(carla.VehicleControl(hand_brake=False))

        return self.pid.run_step(self.target_speed, lead_waypoints[_passed])  # todo: handle end of route


class LeadNavigator:
    """Navigator for the lead vehicle."""
    def __init__(self, vehicle, initial_speed=0, pid_args_lateral=None, pid_args_longitudinal=None,
                 max_brake=0.3, max_throttle=1):
        self._vehicle = vehicle
        self.target_speed = initial_speed
        self.pid_args_lateral = pid_args_lateral
        self.pid_args_longitudinal = pid_args_longitudinal
        self.max_brake = max_brake
        self.max_throttle = max_throttle

        if self.pid_args_lateral is None:
            self.pid_args_lateral = {'K_P': 1.95, 'K_I': 0.05, 'K_D': 0.2, 'dt': 0.01}

        if self.pid_args_longitudinal is None:
            self.pid_args_longitudinal = {'K_P': 2 / 5, 'K_I': 0.26 / 2, 'K_D': 0.26 / 3, 'dt': 0.01}

        self.pid = controller.VehiclePIDController(self._vehicle, args_lateral=self.pid_args_lateral,
                                                   args_longitudinal=self.pid_args_longitudinal,
                                                   max_brake=self.max_brake, max_throttle=self.max_throttle)
        self.world = self._vehicle.get_world()
        self.map = self.world.get_map()
        self.target_waypoint = None
        self.waypoints_ahead = deque()
        self.reset_waypoints()

    def reset_waypoints(self):
        self.target_waypoint = self.map.get_waypoint(self._vehicle.get_location())
        self.waypoints_ahead = deque()
        self.waypoints_ahead.append(self.target_waypoint)
        self.find_waypoints_ahead()

    @property
    def vehicle(self):
        return self._vehicle

    @vehicle.setter
    def vehicle(self, vehicle):
        self._vehicle = vehicle
        self.pid = controller.VehiclePIDController(self._vehicle, args_lateral=self.pid_args_lateral,
                                                   args_longitudinal=self.pid_args_longitudinal,
                                                   max_brake=self.max_brake, max_throttle=self.max_throttle)

    def set_target_speed(self, target_speed):
        self.target_speed = target_speed
        return self.target_speed

    def compute_topology(self):
        pass

    def find_waypoints_ahead(self):     # todo: make it possible to choose direction
        # finding waypoints in the current lane
        next_wpts = self.waypoints_ahead[-1].next(5.0)
        driving_direction = self.waypoints_ahead[-1].transform.rotation.yaw
        if len(next_wpts) == 0:
            wpt = next_wpts[0]
        elif len(next_wpts) > 0:
            wpt = min(next_wpts, key=lambda x: np.abs(x.transform.rotation.yaw-driving_direction) % 360)
        else:
            return "No waypoints found"

        if wpt.is_junction:
            junction_wpt_pairs = wpt.get_junction().get_waypoints(carla.LaneType.Driving)
            junction_wpts_1 = [w[0] for w in junction_wpt_pairs]
            junction_wpts_2 = [w[1] for w in junction_wpt_pairs]
            wpt_1 = min(junction_wpts_1, key=lambda x: np.abs(x.transform.rotation.yaw-driving_direction) % 360)
            wpt_2 = min(junction_wpts_2, key=lambda x: np.abs(x.transform.rotation.yaw-driving_direction) % 360)
            wpt = max(wpt_1, wpt_2, key=lambda x: x.transform.location.distance(self.waypoints_ahead[-1].transform.location))

        self.waypoints_ahead.append(wpt)
        draw_waypoints(self.world, self.waypoints_ahead)

    def find_next_waypoint(self):
        _passed = 0
        for i, wp in enumerate(self.waypoints_ahead):
            if self._vehicle.get_location().distance(wp.transform.location) < 5:
                _passed = max(_passed, i + 1)  # warning: this causes problems in a loop-like trajectory

        for i in range(_passed):
            self.waypoints_ahead.popleft()

        self.target_waypoint = self.waypoints_ahead[0]

    def run_step(self):
        if len(self.waypoints_ahead) < 10:
            self.find_waypoints_ahead()
        self.find_next_waypoint()
        return self.pid.run_step(self.target_speed, self.target_waypoint)
