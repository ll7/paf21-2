## Credits:

PSAF 2: WS 20/21 (perception package)

## Launch Command

Erfasst Hindernisse um das Auto herum, wie z.B. andere Verkehrsteilnehmer, als Datenpunkte mit Hilfe eines Semantic-Lidar-Sensors "Lidar1" und sendet die Information über das Topic /paf/paf_perception/obstacles. Diese enthält Informationen aus drei Positionskoordinaten, bestehend aus jeweils einer x- und y-Koordinate, sowie einem Distanzwert zwischen dem Hindernis und dem Auto. Genauer handelt es sich bei den Positionspunkte um die beiden weit entferntesten Punkte plus der Punkt mit der kürzesten Distanz. Darüber hinaus enthält eine gesendete Nachricht Informationen über die berechnete Geschwindigkeit über Positionsänderungen innerhalb zweier Zeitpunkte und über den Hindernistyp, der entweder den Wert "Vehicles" oder "Pedestrians" annehmen kann.

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

```
Subscribed Topics:
- /paf/paf_local_planner/activate_traffic_light_detection (Bool)

Published Topics:
- /paf/paf_perception/detected_traffic_lights (PafDetectedTrafficLights)

Launchers:
- roslaunch paf_perception traffic_light_detector.launch
```
