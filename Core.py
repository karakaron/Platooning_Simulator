import carla
from agents.navigation import controller, local_planner
from collections import deque


class Simulation(carla.Client):
	"""Top level simulation object."""
	def __init__(self, host='localhost', port=2000, world="Town06", render=True, synchronous=False, dt=0.01):
		# carla setup
		super().__init__(host, port)
		self.set_timeout(10000)
		self.world = self.load_world(world)
		self.map = self.world.get_map()
		self.spectator = self.world.get_spectator()

		_settings = self.world.get_settings()
		_settings.no_rendering_mode = not render
		if synchronous:
			_settings.fixed_delta_seconds = dt
			_settings.substepping = True
			_settings.max_substep_delta_time = 0.01
			_settings.max_substeps = round(dt/0.01) + 1
			_settings.synchronous_mode = True
		if world == 'Town11':
			_settings.actor_active_distance = 2000
		self.world.apply_settings(_settings)

		self.platoons = []

	def add_platoon(self, platoon):
		self.platoons.append(platoon)

	def run_step(self, mode="control"):
		# self.world.tick()
		# mode: either "control" or "sample"; determines if new target speed is computed or only pid iteration is called
		for platoon in self.platoons:
			if mode == "sample":
				platoon.take_measurements()
			platoon.run_pid_step()

	def run(self):
		pass


class Platoon:
	"""Represents a given platoon in the simulation. Contains a number of Vehicle objects."""
	def __init__(self, simulation): 	# follower vehicles must be listed in correct order
		self.lead_waypoints = deque()  # stores waypoints of the lead vehicle
		self.world = simulation.world
		self.map = simulation.map
		self.lead_vehicle = None
		self.follower_vehicles = []

	def __getitem__(self, item):
		if item == 0:
			return self.lead_vehicle
		elif isinstance(item, int):
			return self.follower_vehicles[item-1]
		else:
			raise IndexError("Index must be an integer.")

	def __len__(self):
		return len(self.follower_vehicles) + 1

	def add_lead_vehicle(self, blueprint, spawn_point):
		if self.lead_vehicle is None:
			self.lead_vehicle = Vehicle(blueprint, spawn_point, self.world, 0)
			self.lead_vehicle.attach_controller(local_planner.LocalPlanner(self.lead_vehicle,
							{"longitudinal_control_dict": {'K_P': 2/5, 'K_I': 0.26/2, 'K_D': 0.26/3, 'dt': 0.01}}))
			return self.lead_vehicle
		else:
			raise Exception("This platoon already has a lead vehicle.")

	def add_follower_vehicle(self, blueprint, spawn_point, index=None):  # index: 0 is the lead vehicle
		if index is None:
			index = len(self.follower_vehicles)

		_new_vehicle = Vehicle(blueprint, spawn_point, self.world, index)
		self.follower_vehicles.insert(index, _new_vehicle)

		for i, vehicle in enumerate(self.follower_vehicles):	 # adjust indices
			vehicle.index = i + 1

		return _new_vehicle

	def take_measurements(self):
		self.lead_vehicle.controller.set_speed(90)  # todo: allow custom control of lead vehicle
		lead_waypoint = self.map.get_waypoint(self.lead_vehicle.get_location())
		self.lead_waypoints.append(lead_waypoint)
		for vehicle in self.follower_vehicles:
			vehicle.controller.compute_target_speed(vehicle.index)
			# vehicle.controller.waypoints_to_track.append(lead_waypoint)

	def run_pid_step(self):
		# run pid step on the lead vehicle
		try:
			self.lead_vehicle.apply_control(self.lead_vehicle.controller.run_step())
		except Exception as e:
			print(e)

		# run pid step on the follower vehicles
		for vehicle in self.follower_vehicles:
			vehicle.control(self.lead_waypoints)

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
		self.controller = None

	def attach_controller(self, controller):
		self.controller = controller

	def __lt__(self, other):
		return self.index < other.index

	def __getattr__(self, attr):  # makes the underlying carla Actor instance available
		return getattr(self._carla_vehicle, attr)

	def control(self, lead_waypoints):
		self.apply_control(self.controller.compute_control(lead_waypoints, self.index))

