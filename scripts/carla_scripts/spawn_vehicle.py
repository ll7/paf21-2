import carla
import numpy as np


client = carla.Client("localhost", 2000)

client.set_timeout(10.0)  # seconds
world = client.get_world()
blueprint_library = world.get_blueprint_library()

vehicle_bp = blueprint_library.find("vehicle.tesla.model3")
spawn_points = world.get_map().get_spawn_points()
model3_spawn_point = np.random.choice(spawn_points)


target_waypoints = [100, 207.5, 1.5]


# spawn position
start = carla.Transform(carla.Location(x=12, y=207.5, z=1.5), carla.Rotation(yaw=180))
vehicle = world.spawn_actor(vehicle_bp, start)

# custom_controller = VehiclePIDController(vehicle, args_lateral = {'K_P': 1, 'K_D': 0.0, 'K_I': 0},
# 	args_longitudinal = {'K_P': 1, 'K_D': 0.0, 'K_I': 0.0})

# ticks_to_track = 20
# current_waypoint = start
# for i in range(ticks_to_track):
# 	control_signal = custom_controller.run_step(1, target_waypoint)
# 	vehicle.apply_control(control_signal)
