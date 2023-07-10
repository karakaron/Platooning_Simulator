"""An example using Carla's built-in autopilot and
a simple CACC strategy."""
from PlatooningSimulator import Core, PlatooningControllers
import numpy as np
from matplotlib import pyplot as plt


def v_ref_cruise(predecessor, ego):
	"""Cooperative-Adaptive Cruise Controller
	:param predecessor: Core.Vehicle instance of vehicle i-1
	:param ego: Core.Vehicle instance of vehicle i
	:return: reference velocity for vehicle i"""
	tau = 0.66  # vehicle engine constant
	h = 0.5  # time headway
	c = 2  # controller constant
	length = 5  # vehicle length

	delta_i = ego.distance_to(predecessor)  # distance between cars
	speed_i_1 = predecessor.speed  # speed in m/s
	speed_i = ego.speed
	return (tau / h * (speed_i_1 - speed_i + c * (delta_i - length - h * speed_i)) + speed_i) * 3.6


# setting up the simulation

simulation_length = 120
sampling_rate = 10
pid_rate = 10
N = 8

log = np.zeros([N, 3, simulation_length*sampling_rate])

simulation = Core.Simulation(world="Town06", dt=float(1 / sampling_rate / pid_rate), large_map=False, render=True)

bps = simulation.get_vehicle_blueprints()
bp = bps.filter("*vehicle.bmw.grandtourer*")[0]
spawn_points = simulation.map.get_spawn_points()

platoon = Core.Platoon(simulation)
lead_vehicle = platoon.add_lead_vehicle(bp, spawn_points[30])
simulation.tick()

tm = simulation.get_trafficmanager()
tm.set_synchronous_mode(True)
tm_port = tm.get_port()
lead_vehicle.set_autopilot(True, tm_port)		# enabling autopilot on the lead vehicle

followers = []
deps = [-1, 0]

# adding follower vehicles 10 meters apart
for i in range(0, N-1):
	fv = platoon.add_follower_vehicle(bp, (lead_vehicle.transform_ahead(-10, force_straight=True) if i == 0
										  else followers[i-1].transform_ahead(-10, force_straight=True)))
	cacc_controller = PlatooningControllers.FollowerController(fv, v_ref_cruise, platoon, dependencies=deps)
	fv.attach_controller(cacc_controller)
	followers.append(fv)
	simulation.tick()

# optionally recording the simulation so that it can be replayed
# simulation.start_recorder("C:/Dev/Carla_logs/autopilot_recording_2.log", True)

for i in range(simulation_length*pid_rate*sampling_rate):
	simulation.run_step(mode="sample" if i % pid_rate == 0 else "control")

	# moving the camera with the platoon
	if i % 100 == 0:
		s_transform = platoon[0].transform_ahead(30, force_straight=True)
		s_transform.location.z += 8
		s_transform.rotation.yaw += 180
		simulation.spectator.set_transform(s_transform)

	# saving simulation data at each sampling time step (optional)
	if i % pid_rate:
		for j, vehicle in enumerate(platoon):
			log[j, 1, int(i/pid_rate)] = vehicle.speed
			if i > 0:
				log[j, 2, int(i/pid_rate)] = (vehicle.speed - log[j, 1, int(i/pid_rate)-1]) / sampling_rate
			if j > 0:
				log[j, 0, int(i/pid_rate)] = vehicle.distance_to(platoon[j-1])
	simulation.tick()

# simulation.stop_recorder()
simulation.release_synchronous()

# plotting the saved simulation data
t = np.linspace(0, simulation_length*sampling_rate-1, simulation_length*sampling_rate)
colors = plt.cm.get_cmap('turbo', N)

for j, vehicle in enumerate(platoon):
	plt.plot(t, log[j, 1, :], label=f"vehicle {j}", c=colors(j))		# velocity
	if j > 0:
		plt.plot(t[1:], log[j, 0, 1:], linestyle="dashed", c=colors(j))		# physical spacing

plt.legend()
plt.show()
