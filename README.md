

# Praktikum für Autonomes Fahren - Gruppe 2

## Inhaltsverzeichnis

- [Praktikum für Autonomes Fahren - Gruppe 2](#praktikum-für-autonomes-fahren---gruppe-2)
  - [Inhaltsverzeichnis](#inhaltsverzeichnis)
    - [Aufgabenstellung](#aufgabenstellung)
    - [Architekturbeschreibung](#architekturbeschreibung)
    - [Module](#module)
      - [Acting](#acting)
      - [Map Manipulation](#map-manipulation)
      - [Perception](#perception)
      - [Planning](#planning)
          - [Global Planner](#global-planner)
          - [Local Planner](#local-planner)
          - [Obstacle Planner](#obstacle-planner)
      - [Starter](#starter)
      - [Validation (optional)](#validation-optional)
  - [Installation des Projekts](#installation-des-projekts)
  - [Ausführen des Projekts](#ausführen-des-projekts)



###  Aufgabenstellung
Im Rahmen des Praktikums "Praktikum Autonomes Fahren" sollte ein autonomes Fahrzeug entwickelt werden, dass autonom von einem Startpunkt zu einem Zielpunkt fahren kann. Dafür wurde als Simulationsumgebung Carla verwendet. Das Fahrzeug an sich soll in ROS implementiert werden. 

Die Aufgabenstellung umfasst 2 Teile.\
Allgemeine Anforderungen:
1. Das Fahrzeug muss autonom einen Weg vom Startpunkt zum Zielpunkt planen 
2. Das Fahrzeug muss diesen Weg autonom abfahren 
3. Folgenden CARLA Karten müssen unterstützt werden: Town01, Town02, Town03, Town04, Town05, Town06, Town07, Town10HD
4. Das Fahrzeug befindet sich in einer Welt mit anderen Verkehrsteilnehmer (andere Autos, Motorradfahrer, Fußgänger) 

Teil 1: Modus ohne Regeln. Ziel dabei ist es ohne Beachtung aller Verkehrsregeln möglichst schnell ans Ziel zu kommen 

Teil 2: Modus mit Regeln. Ziel dabei ist es unter Beachtung aller Verkehrsregeln möglichst schnell ans Ziel zu kommen. Für Verstöße gegen die Verkehrsregeln werden Strafzeiten vergeben.

###  Architekturbeschreibung 



![new Architecture Graph](docs/imgs/PAF_Architektur_final.drawio.svg)


###  Module

####  Acting
Dieses Modul dient dazu dass das Ego Vehicle einem vorgegenen Pfad folgen kann. Dabei wird ein Regler zu Regelung der Geschwindigkeit verwendet und ein Stanley-Controller zur Regelung des Lenkwinkels. Zustätzlich dient diese Modul noch dazu, selbstständig aus kritischen Situationen freizukommen

####  Map Manipulation
Diese Modul dient der Verwaltung von Kartendaten. Diese werden als Commonroad-Kartendaten abgespeichert und enthalten Informationen über alle befahrbaren Wege einer Stadt. Zusätzlich sind in den Karten alle Schilder und Amplen als auch deren zugeordneten Haltelinien eingetragen. 


#### Perception 
Diese Modul dient dazu die Umgebung um das Ego Vehicle wahrzunehmen. Dazu wird sowohl ein Semantic-Lidar-Sensor wie auch die Segmentation-Camera verwendet. 
Dabei wird der Semantic-Lidar-Sensor dazu verwendet andere Verkehrsteilnehmer zu erkennen und zu identifizieren. Die Segmentation-Kamera dient hier zur Ampelerkennung
die unter Verwendung des Systems YOLO v3 funktioniert. 


#### Planning
Dieses Modul ist für die Planung der Route des Ego Vehicles zuständig. Dafür wurde es in folgende Teimodule gegliedert:
###### Global Planner
Der Global Planner ist für die Berechnung der kürzesten Route zwischen Start- und Zielpunkt zuständig. Dafür wird der CommonRoad Route Planner verwendet. 
###### Local Planner
Der Local Planner ist dafür zuständig ein Segment des globalen Pfads bestmöglich abzufahren und dabei alle Verkehrsregeln beachten und korrekt auf andere Verkehrsteilnehmer zu reagieren

###### Obstacle Planner
Beim Obstacle Planner handelt es sich um ein Teilmodul des Planners, dass dazu dient auf erkannte Hindernisse, wie andere Verkehrsteilnehmer reagieren zu können. 
Funktionen diese Moduls sind das Folgen anderer Verkehrsteilnhemer unter Beachtung des Mindestabstands und das Anhalten, wenn sich ein Hindenis direkt vor dem Ego Vehicle befindet. 

#### Starter 
Dieses Modul dient zum einfachen Starten des Projekts. Dabei werden alle notwendigen oben genannten Module gestartet, die dazu benötigt werden autonom von einem Startpunkt zu einem Zielpunkt zu gelangen. 

###### Competition Manager
Das Starter-Modul bietet außerdem die Möglichkeit, den Competition Manager zu verwenden. Hierzu müssen folgende Schritte ausgeführt werden:

 1. Starte Carla und unsere Module:
```
cd paf21-2/scripts
./runscript.sh -mc <town>
```
 2. Platziere NPC's (optional auch später möglich):
```
cd carla_0.9.10.1/PythonAPI/examples/
python spawn_npc.py -n 50 -w 50 -s 256
```
3. Lade die Konfiguration des Competition Managers. Das Starter-Modul liest daraus `traffic_rules` und die Zielposition zur Planung aus und wartet anschließend, bis der Competition Manager gestartet wird.
```
cd paf_competition_manager
rosparam load config/<town_directory>/<yaml_file_name>.yaml
```
4. Teleportiere das Fahrzeug an die Startposition (`set_position_server.py` muss ggf. zweimal ausgeführt werden, damit es funktioniert):
```
rosrun paf_competition_manager set_position_server.py
rosrun paf_competition_manager set_position_client.py
```
5. Starte den Competition Manager:
```
rosrun paf_competition_manager simple_competition_manager.py
```

#### Validation (optional) 
Das Validation Modul dient dazu, beim Abfahren einer Route automatisch Fehler beim Verhalten des Fahrzeugs zu notieren und das Endergebnis nach dem Abfahren einer Route zu berwerten. 

## Installation des Projekts 
Zum Installieren des Projekts inklusive gesamter Simulationsumgebung:

```
bash scripts/setup.sh
```

Dadurch werden Carla, ROS und alle notwendigen Dependencies installiert.\
\
Um alle Komponenten einzeln zu installieren, folgende Scrips ausführen:

```
bash scripts/subscripts/_setup_carla.sh      # carla install
bash scripts/subscripts/_setup_ros.sh        # ros install
bash scripts/subscripts/_setup_paf.sh        # dependency install
bash scripts/subscripts/_setup_commonroad.sh # commonroad extended install
```
## Ausführen des Projekts 
Ausführen des von scrips/runscripts.sh 
``` 
bash /paf21-2/scripts/runscripts.sh
 ```

Argumente:
```
--skip-carla-restart/-scr   die Carla Simulation wird nicht neugestartet 
--build/-b                  erneutes builden des Projekts, wenn man Änderungen im Code vorgenommen hat 
--npcs/-n                   Starten mit npcs 
--low-quality/-lq           Starten in niedrigerer Qualität für leistungsschwächere Geräte 
--manual-control/-mc        Starten mit Öffnen eines manual-control Fensters
--no-rules/-nr              Unterscheidung ob man mit oder ohne Regeln Fahren möchte
TownXX                      Town auf der gefahren werden soll, mögliche Towns Town01-07, Town10HD
```



