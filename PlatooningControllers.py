import carla
from collections import deque
from agents.navigation import controller


class FollowerController:
    """Controller for follower vehicles. Desired speed is given by a user-defined function at every time step.
    Spatial trajectory is inherited from the lead vehicle"""

    def __init__(self, vehicle, control_function, parameters, platoon):
        # parameters: ORDERED list of 2(or more)-tuples of vehicle index and attribute(s), index 0 is ego
        self.control_function = control_function
        self.parameters = parameters
        self.vehicle = vehicle
        self.pid = controller.VehiclePIDController(self.vehicle,
                                                   args_lateral={'K_P': 1.95, 'K_I': 0.05, 'K_D': 0.2, 'dt': 0.01},
                                                   args_longitudinal={'K_P': 2 / 5, 'K_I': 0.26 / 2, 'K_D': 0.26 / 3,
                                                                      'dt': 0.01},
                                                   max_brake=0.3, max_throttle=0.75)
        self.platoon = platoon
        self.target_speed = 0

    # self.waypoints_to_track = deque()

    def compute_target_speed(self, index):
        _args = [getattr(self.platoon[index + p[0]], *p[1:]) for p in self.parameters]
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

        return self.pid.run_step(self.target_speed, lead_waypoints[_passed])  # todo: handle end of route
