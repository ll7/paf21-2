import carla
import numpy as np


client = carla.Client("localhost", 2000)

client.set_timeout(10.0)  # seconds
world = client.get_world()
blueprint_library = world.get_blueprint_library()

vehicle_bp = blueprint_library.find("vehicle.tesla.model3")
spawn_points = world.get_map().get_spawn_points()
model3_spawn_point = np.random.choice(spawn_points)

# spawn position
start = carla.Transform(carla.Location(x=-20, y=203.7, z=1.5), carla.Rotation(yaw=360))
vehicle = world.spawn_actor(vehicle_bp, start)

vehicle.apply_control(carla.VehicleControl(throttle=0.3, steer=0.0))
