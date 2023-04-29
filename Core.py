import carla
import numpy as np
from collections import deque
from copy import copy
import warnings


class Simulation(carla.Client):
	"""Top level simulation object."""
	def __init__(self, host='localhost', port=2000, world="Town06", render=True, synchronous=False, dt=0.01):
		# carla setup
		super().__init__(host, port)
		self.set_timeout(10000)
		self.world = self.load_world(world)

		_settings = self.world.get_settings()
		_settings.no_rendering_mode = not render
		if synchronous:
			_settings.fixed_delta_seconds = dt
			_settings.substepping = True
			_settings.max_substep_delta_time = 0.01
			_settings.max_substeps = round(dt/0.01) + 1
			_settings.synchronous_mode = True
			self.world.apply_settings(_settings)
			self.world.tick()
		if world == 'Town11':
			_settings.actor_active_distance = 2000
			self.world.apply_settings(_settings)

		self.map = self.world.get_map()
		self.spectator = self.world.get_spectator()
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
	def __init__(self, simulation):
		self.lead_waypoints = deque()  # stores waypoints of the lead vehicle
		self.world = simulation.world
		self.map = simulation.map
		self.lead_vehicle = None
		self.follower_vehicles = []
		self.simulation = simulation

	def __getitem__(self, item):
		all_vehicles = [self.lead_vehicle] + self.follower_vehicles
		try:
			return all_vehicles[item]
		except IndexError as e:
			print(all_vehicles, item)
			raise e

	def __len__(self):
		return len(self.follower_vehicles) + 1

	def add_lead_vehicle(self, blueprint, spawn_point):
		if self.lead_vehicle is None:
			self.lead_vehicle = Vehicle(blueprint, spawn_point, self.world, 0)
			return self.lead_vehicle
		else:
			raise Exception("This platoon already has a lead vehicle.")

	def add_follower_vehicle(self, blueprint, spawn_point, index=None, history_depth=0, history_data=None):  # index: 0 is the lead vehicle
		if index is None:
			index = len(self.follower_vehicles)

		_new_vehicle = Vehicle(blueprint, spawn_point, self.world, index, history_depth, history_data)
		self.follower_vehicles.insert(index, _new_vehicle)

		self.reindex()

		return _new_vehicle

	def take_measurements(self):
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
			warnings.warn(f"{e}, lead vehicle")

		# run pid step on the follower vehicles
		for vehicle in self.follower_vehicles:
			try:
				vehicle.control(self.lead_waypoints)
				vehicle.update_history()
			except Exception as e:
				warnings.warn(f"{e}, follower vehicle {vehicle.index}")

	def reindex(self):
		for i, vehicle in enumerate(self.follower_vehicles):	 # adjust indices
			vehicle.index = i + 1

	def split(self, first, last):  # new platoon is created from the vehicles between indices first and last
		new_platoon = Platoon(self.simulation)
		new_lead_controller = copy(self.lead_vehicle.controller)
		vehicles_to_split = self[first: last + 1 - (first == 0)]

		del self.follower_vehicles[first-1: last - (first == 0)]

		if first == 0:
			own_new_lead_vehicle = self.follower_vehicles.pop(last)
			self.lead_vehicle.controller.vehicle = own_new_lead_vehicle
			own_new_lead_vehicle.attach_controller(self.lead_vehicle.controller)

		for vehicle in vehicles_to_split[1:]:
			vehicle.controller.platoon = new_platoon

		new_lead_controller.vehicle = vehicles_to_split[0]
		vehicles_to_split[0].attach_controller(new_lead_controller)

		new_platoon.lead_vehicle = vehicles_to_split[0]
		new_platoon.follower_vehicles = vehicles_to_split[1:]
		new_platoon.reindex()

		new_lead_controller.reset_waypoints()

		self.reindex()
		self.simulation.add_platoon(new_platoon)
		# new_platoon.take_measurements()

		return new_platoon, new_lead_controller

	def merge(self, other):  # merges other platoon into self (at the end)
		other_follower_controller = copy(other[1].controller)  # copying first followers controller to assign to lead
		other_follower_controller.vehicle = other[0]
		other[0].attach_controller(other_follower_controller)
		self.follower_vehicles.append(other[0])
		self.follower_vehicles.extend(other.follower_vehicles)
		self.reindex()

		# other.lead_waypoints.reverse()  # extendleft reverses order
		# self.lead_waypoints.extendleft(other.lead_waypoints)  # todo: add lead_waypoints from other platoon

		other_follower_controller.platoon = self
		for vehicle in other.follower_vehicles:
			vehicle.controller.platoon = self

		self.simulation.platoons.remove(other)
		del other


class Vehicle:
	"""Represents a vehicle in the simulation. Spawns a carla vehicle at init and has additional features.
	The underlying carla Actor instance can be accessed directly."""
	def __init__(self, blueprint, spawn_point, world, index, history_depth=0, history_data=None):
		self.blueprint = blueprint
		self.spawn_point = spawn_point
		self.world = world
		self._carla_vehicle = world.spawn_actor(blueprint, spawn_point)
		self.index = index
		self.controller = None
		self.history = VehicleHistory(history_depth)
		if history_data is None:
			self.history_data = []
		else:
			self.history_data = history_data
			for attribute in self.history_data:
				self.history.add_attribute(attribute)

	def attach_controller(self, controller):
		self.controller = controller

	def __lt__(self, other):
		return self.index < other.index

	def __getattr__(self, attr):  # makes the underlying carla Actor instance available
		return getattr(self._carla_vehicle, attr)

	def __str__(self):
		return f"Follower vehicle {self.index}"

	@property
	def speed(self):  # norm of vehicle velocity in m/s
		v = self._carla_vehicle.get_velocity()
		return np.sqrt(v.x**2 + v.y**2 + v.z**2)

	@property
	def acceleration(self):  # signed norm of acceleration in m/s^2
		a = self._carla_vehicle.get_acceleration()
		v = self._carla_vehicle.get_velocity()
		sign = 1 if a.x * v.x + a.y * v.y >= 0 else -1
		return sign*np.sqrt(a.x**2 + a.y**2 + a.z**2)

	@property
	def heading(self):
		transform = self._carla_vehicle.get_transform()
		return transform.rotation.yaw

	def distance_to(self, other):  # distance to other vehicle
		location = self._carla_vehicle.get_location()
		other_location = other.get_location()
		return location.distance(other_location)

	def control(self, lead_waypoints):
		self.apply_control(self.controller.compute_control(lead_waypoints, self.index))

	def update_history(self):
		for attribute in self.history_data:
			self.history[attribute] = getattr(self, attribute)
		# todo: this is limited to variables, cannot access method return values


class VehicleHistory:
	def __init__(self, depth=0):
		self.depth = depth

	def add_attribute(self, attribute_name):
		setattr(self, attribute_name, deque(maxlen=self.depth))

	def __setitem__(self, key, value):
		getattr(self, key).append(value)

