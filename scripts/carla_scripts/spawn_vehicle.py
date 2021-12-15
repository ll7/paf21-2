import carla
import numpy as np


client = carla.Client("localhost", 2000)

client.set_timeout(10.0)  # seconds
world = client.get_world()
blueprint_library = world.get_blueprint_library()

vehicle_bp = np.random.choice(blueprint_library.filter("vehicle.bmw.*"))
spawn_points = world.get_map().get_spawn_points()
model3_spawn_point = np.random.choice(spawn_points)

# spawn position
transform = carla.Transform(carla.Location(x=12, y=207.5, z=1.5), carla.Rotation(yaw=180))
actor = world.spawn_actor(vehicle_bp, transform)
