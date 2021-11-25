## Credits:

PSAF 2: WS 20/21 (perception package)

## Launch Command

Reads obstacles from segmented lidar sensor ```lidar1``` and publishes a ```PafObstacleList```. Each ```PafObstacle```
has to outer bounding box points and the closest point as attributes

```
Subscribed Topics:
- /carla/ego_vehicle/semantic_lidar/lidar1/point_cloud (PointCloud2)
- /carla/ego_vehicle/odometry (Odometry)

Published Topics:
- /paf/paf_perception/obstacles (PafObstacleList)

Launchers:
- roslaunch paf_perception semantic_lidar.launch
```
