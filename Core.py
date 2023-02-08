import carla
from agents.navigation import controller, local_planner


class Simulation(carla.Client):
	"""Top level simulation object."""
	def __init__(self, host='localhost', port=2000, world="Town06", render=True):
		# carla setup
		super().__init__(host, port)
		self.set_timeout(10000)
		self.world = self.load_world(world)
		self.map = self.world.get_map()
		self.spectator = self.world.get_spectator()

		if not render:
			_settings = self.world.get_settings()  # turn rendering off
			_settings.no_rendering_mode = True
			self.world.apply_settings(_settings)

		self.platoons = []

	def add_platoon(self, platoon):
		self.platoons.append(platoon)

	def run_step(self):
		for platoon in self.platoons:
			platoon.run_step()

	def run(self):
		pass


class Platoon:
	"""Represents a given platoon in the simulation. Contains a number of Vehicle objects."""
	def __init__(self, simulation): 	# follower vehicles must be listed in correct order
		self.lead_waypoints = []  # stores waypoints of the lead vehicle
		self.world = simulation.world
		self.map = simulation.map
		self.lead_vehicle = None
		self.follower_vehicles = []

	def __getitem__(self, item):
		if item == 0:
			return self.lead_vehicle
		elif isinstance(item, int):
			return self.follower_vehicles[item]
		else:
			raise IndexError("Index must be an integer.")

	def add_lead_vehicle(self, blueprint, spawn_point):
		if self.lead_vehicle is None:
			self.lead_vehicle = Vehicle(blueprint, spawn_point, self.world, 0)
			self.lead_vehicle.attach_controller(local_planner.LocalPlanner(self.lead_vehicle))
			return self.lead_vehicle
		else:
			raise Exception("This platoon already has a lead vehicle.")

	def add_follower_vehicle(self, blueprint, spawn_point, index = None):  # index: 0 is the lead vehicle
		if index is None:
			index = len(self.follower_vehicles)
		_new_vehicle = Vehicle(blueprint, spawn_point, self.world, index)
		self.follower_vehicles.insert(index-1, _new_vehicle)
		for i, vehicle in enumerate(self.follower_vehicles):	 # adjust indices
			vehicle.index = i + 1
		return _new_vehicle

	def run_step(self):
		self.lead_waypoints.append(self.map.get_waypoint(self.lead_vehicle.get_location()))
		self.lead_vehicle.controller.set_speed(30)		# todo: allow custom control of lead vehicle
		self.lead_vehicle.apply_control(self.lead_vehicle.controller.run_step(debug=True))
		for vehicle in self.follower_vehicles:
			vehicle.control(lead_waypoints=self.lead_waypoints)

	def split(self, first, last):  # new platoon is created from the vehicles between indices first and last
		pass
		# new_platoon = Platoon(follower_vehicles=self.follower_vehicles[first+1:last], lead_vehicle=self.follower_vehicles[first])
		# del self.follower_vehicles[first:last]
		# if first == 0:
		# 	self.lead_vehicle = self.follower_vehicles[0]
		# return new_platoon

	def merge(self, other):
		pass


class Vehicle:
	"""Represents a vehicle in the simulation. Spawns a carla vehicle at init and has additional features.
	The underlying carla Actor instance can be accessed directly."""
	def __init__(self, blueprint, spawn_point, world, index):
		self.blueprint = blueprint
		self.spawn_point = spawn_point
		self.world = world
		self._carla_vehicle = world.spawn_actor(blueprint, spawn_point)
		self.index = index

	def attach_controller(self, controller):
		self.controller = controller

	def __lt__(self, other):
		return self.index < other.index

	def __getattr__(self, attr):  # makes the underlying carla Actor instance available
		return getattr(self._carla_vehicle, attr)

	def control(self, lead_waypoints):
		self.apply_control(self.controller.compute_control(lead_waypoints, self.index))


class FollowerController:
	"""Controller for follower vehicles. Desired speed is given by a user-defined function at every time step.
	Spatial trajectory is inherited from the lead vehicle"""
	def __init__(self, vehicle, control_function, parameters, platoon):
		# parameters: ORDERED list of 2(or more)-tuples of vehicle index and attribute(s), 0 is ego
		self.control_function = control_function
		self.parameters = parameters
		self.vehicle = vehicle
		self.pid = controller.VehiclePIDController(self.vehicle,
												   args_lateral={'K_P': 1, 'K_I': 0.05, 'K_D': 0.2, 'dt': 0.05},
												   args_longitudinal={'K_P': 3/5, 'K_I': 0.29/2, 'K_D': 0.29/3, 'dt': 0.05})
		self.platoon = platoon

	def compute_control(self, lead_waypoints, index):  # index is ego vehicle index
		# compute target speed via user defined control strategy
		_args = [getattr(self.platoon[index + p[0]], *p[1:]) for p in self.parameters]
		target_speed = self.control_function(_args)
		# find next waypoint
		_to_remove = 0
		for i, wp in enumerate(lead_waypoints):
			if self.vehicle.get_location().distance(wp.transform.location) < 2:
				_to_remove = max(_to_remove, i + 1)  # warning: this causes problems in a loop-like trajectory
		del lead_waypoints[0:_to_remove]
		return self.pid.run_step(target_speed, lead_waypoints[0])

