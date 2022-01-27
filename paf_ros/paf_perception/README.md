## Credits:

PSAF 2: WS 20/21 (perception package)

## Launch Command

Reads obstacles from segmented lidar sensor ```lidar1``` and publishes a ```PafObstacleList```. Each ```PafObstacle```
has the two outermost (north-aligned) bounding box points and the closest point as attributes.

```
Subscribed Topics:
- /carla/ego_vehicle/semantic_lidar/lidar1/point_cloud (PointCloud2)
- /carla/ego_vehicle/odometry (Odometry)

Published Topics:
- /paf/paf_perception/obstacles (PafObstacleList)

Launchers:
- roslaunch paf_perception semantic_lidar.launch
```

## TrafficLightDetector:

Credits: psaf1 WS 20/21

Benötigt PyTorch:
"Ebenso muss pytorch entsprechen der Anleitung auf (Pytorch.org)[https://pytorch.org/] installiert werden. Dabei ist auf die lokal verwendete Treiberversion zu achten. Für eine optimale Nutzung wird eine Nvidia-Grafikarte in Verbindung mit dem ensprechenden Cuda-Treiber benötigt."
(pip3 install torch==1.10.1+cu113 torchvision==0.11.2+cu113 torchaudio==0.10.1+cu113 -f https://download.pytorch.org/whl/cu113/torch_stable.html)
