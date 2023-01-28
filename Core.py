import carla


class Simulation:
	"""Top level simulation object."""


class Platoon:
	"""Represents a given platoon in the simulation. Contains a number of Vehicle objects."""
	def __init__(self, vehicles, lead_vehicle):
		self.vehicles = vehicles
		self.lead_vehicle = lead_vehicle

	def split(self, first, last):  # new platoon is created from the vehicles between indices first and last
		new_platoon = Platoon(vehicles=self.vehicles[first:last], lead_vehicle=self.vehicles[first])
		del self.vehicles[first:last]
		if first == 0:
			self.lead_vehicle = self.vehicles[0]
		return new_platoon

	def merge(self, other):
		pass


class Vehicle:
	"""Represents a vehicle in the simulation. Spawns a carla vehicle at init and has additional features."""
	def __init__(self, blueprint, spawn_point, world):
		self.blueprint = blueprint
		self.spawn_point = spawn_point
		self.world = world
		self._carla_vehicle = world.spawn_actor(blueprint, spawn_point)

	def __getattr__(self, attr):  # makes the underlying carla Actor instance available
		return getattr(self._carla_vehicle, attr)



