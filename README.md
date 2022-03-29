# Praktikum für Autonomes Fahren - Gruppe 2

## Inhaltsverzeichnis

- [Praktikum für Autonomes Fahren - Gruppe 2](#praktikum-für-autonomes-fahren---gruppe-2)
    - [Inhaltsverzeichnis](#inhaltsverzeichnis)
        - [Aufgabenstellung](#aufgabenstellung)
        - [Architekturbeschreibung](#architekturbeschreibung)
        - [Module](#module)
            - [<a href = "https://github.com/ll7/paf21-2/tree/main/paf_ros/paf_actor#readme">  Acting </a>](#--acting-)
            - [<a href = "https://github.com/ll7/paf21-2/tree/main/paf_ros/paf_map_manipulation#readme">  Map Manipulation </a>](#--mapmanipulation-)
            - [<a href = "https://github.com/ll7/paf21-2/tree/main/paf_ros/paf_perception#readme">  Perception </a>](#--perception-)
            - [<a href = "https://github.com/ll7/paf21-2/tree/main/paf_ros/paf_planning#readme">  Planning </a>](#--planning-)
                - [Global Planner](#global-planner)
                - [Local Planner](#local-planner)
                - [Obstacle Planner](#obstacle-planner)
            - [Starter](#starter)
                - [Competition Manager](#competition-manager)
            - [<a href = "https://github.com/ll7/paf21-2/tree/main/paf_ros/paf_validation#readme">  Validation </a>](#--validation-)
    - [Installation des Projekts](#installation-des-projekts)
    - [Ausführen des Projekts](#ausführen-des-projekts)

## Aufgabenstellung

Im Rahmen des Praktikums "Praktikum Autonomes Fahren" sollte ein autonomes Fahrzeug entwickelt werden, welches autonom
von einem Startpunkt zu einem Zielpunkt fahren kann. Dafür wurde als Simulationsumgebung Carla verwendet. Das Fahrzeug
an sich soll in ROS implementiert werden.

Die Aufgabenstellung umfasst 2 Teile.

Allgemeine Anforderungen:

1. Das Fahrzeug muss einen Weg vom Startpunkt zum Zielpunkt planen.
2. Das Fahrzeug muss diesen Weg autonom abfahren.
3. Folgenden CARLA Karten müssen unterstützt werden: Town01, Town02, Town03, Town04, Town05, Town06, Town07, Town10HD
4. Das Fahrzeug befindet sich in einer Welt mit anderen Verkehrsteilnehmern (andere Autos, Motorradfahrer, Fußgänger).

Teil 1: Modus ohne Regeln. Ziel dabei ist es, ohne Beachtung jeglicher Verkehrsregeln möglichst schnell ans Ziel zu
kommen. \
Teil 2: Modus mit Regeln. Ziel dabei ist es, unter Beachtung aller Verkehrsregeln möglichst schnell ans Ziel zu kommen.
Für Verstöße gegen die Verkehrsregeln werden Strafzeiten vergeben.

## Architekturbeschreibung

![new Architecture Graph](docs/imgs/PAF_Architektur_final.drawio.svg)

## Module

### <a href = "https://github.com/ll7/paf21-2/tree/main/paf_ros/paf_actor#readme">  Acting </a>

Dieses Modul dient dazu, dass das Ego Vehicle einem vorgegenen Pfad folgen kann. Dabei wird ein Regler zu Regelung der
Geschwindigkeit verwendet und ein Stanley-Controller zur Regelung des Lenkwinkels. Zustätzlich dient dieses Modul noch
dazu, selbstständig aus kritischen Situationen freizukommen

### <a href = "https://github.com/ll7/paf21-2/tree/main/paf_ros/paf_map_manipulation#readme">  Map Manipulation </a>

Dieses Modul dient der automatisierten Bearbeitung von Kartendaten. Commonroad-Szenarien können mithilfe der
MapManipulator-Klasse geladen, modifiziert und abgespeichert werden.

### <a href = "https://github.com/ll7/paf21-2/tree/main/paf_ros/paf_perception#readme">  Perception </a>

Dieses Modul dient dazu, die Umgebung um das Ego Vehicle und Ampelzustände wahrzunehmen. Dazu werden als Sensoren zum
einen ein Semantic-Lidar-Sensor, zum anderen eine Depth-, Segmentation- und RGBCamera verwendet. Der
Semantic-Lidar-Sensor ermöglicht die Erkennung und Identifikation anderer Verkehrsteilnehmer. Die unterschiedlichen
Kameras werden synchronisiert und liefern die Informationen, anhand derer die TrafficLightDetection Ampeln mittels
Deep-Learning klassifiziert.

### <a href = "https://github.com/ll7/paf21-2/tree/main/paf_ros/paf_planning#readme">  Planning </a>

Dieses Modul ist für die Planung der Route des Ego Vehicles zuständig. Dafür wurde es in folgende Teimodule gegliedert:

#### Global Planner

Der Global Planner ist für die Berechnung der kürzesten Route zwischen Start- und Zielpunkt zuständig. Dafür wird der
Commonroad Route Planner verwendet, dessen Ergebnis in eine Routenmatrix mit allen Spuroptionen umgewandelt wird.

#### Local Planner

Der Local Planner ist dafür zuständig ein Segment des globalen Pfads bestmöglich abzufahren und dabei alle
Verkehrsregeln beachten und korrekt auf andere Verkehrsteilnehmer zu reagieren. Dabei wird versucht, anderen
Verkehrsteilnehmern mit Spurwechseln auszuweichen.

#### Obstacle Planner

Beim Obstacle Planner handelt es sich um ein Teilmodul des Planners, dass die Hinderniserkennung des Semantic Lidar in
eine Information umwandelt, mit welcher Distanz und Geschwindigkeit einem Fahrzeug vor dem Ego-Vehicle oder auf dem
lokalen Pfad gefolgt werden muss. Das Ergebnis wird als Handlungsanweisung für das Acting und zur Entscheidung bei der
Spurwahl verwendet.

### Starter

Dieses Modul enthält einen Launcher zum einfachen Starten des Projekts. Dabei werden alle notwendigen oben genannten
Module gestartet, die dazu benötigt werden, um autonom von einem Startpunkt zu einem Zielpunkt zu gelangen. Dabei gibt
es zwei Modi, wie das Projekte verwendet werden kann. Für weitere Argumente des `runscript.sh`, siehe unten oder
verwende folgenden Befehl: `bash runscript.sh --help`.

1. Competition Mode
2. Validation Mode

#### Competition Manager (Competition Mode)

Das Starter-Modul bietet außerdem die Möglichkeit, den Competition Manager zu verwenden. Hierzu müssen folgende Schritte
ausgeführt werden:

1. Starte Carla und unsere Module:

```
cd paf21-2/scripts
bash runscript.sh <town>
```

2. Platziere NPC's (optional auch später möglich):

```
cd carla_0.9.10.1/PythonAPI/examples/
python spawn_npc.py -n 50 -w 50 -s 256
```

3. Lade die Konfiguration des Competition Managers. Das Starter-Modul liest daraus `traffic_rules` und die Zielposition
   zur Planung aus und wartet anschließend, bis der Competition Manager gestartet wird.

```
cd paf_competition_manager
rosparam load config/<town_directory>/<yaml_file_name>.yaml
```

4. Teleportiere das Fahrzeug an die Startposition (`set_position_server.py` muss ggf. zweimal ausgeführt werden, damit
   es funktioniert):

```
rosrun paf_competition_manager set_position_server.py
rosrun paf_competition_manager set_position_client.py
```

5. Starte den Competition Manager:

```
rosrun paf_competition_manager simple_competition_manager.py
```

#### Random Route / Standard Route (Validation Mode)

Starte Carla und unsere Module:

```
cd paf21-2/scripts
bash runscript.sh --validation <town>
```

Optional: Setze Flag `USE_GLOBAL_STANDARD_LOOP=True` in `paf_local_planner.py` und verwende den Standard Loop der
jeweiligen Map aus `MapManager.py`.

### <a href = "https://github.com/ll7/paf21-2/tree/main/paf_ros/paf_validation#readme">  Validation </a>

Das Validation Modul dient dazu, beim Abfahren einer Route automatisch Fehler beim Verhalten des Fahrzeugs zu notieren
und das Endergebnis nach dem Abfahren einer Route zu bewerten. Zudem wird eine Top-Down-Ansicht mit Anzeige von
beliebigen Punkten und Pfaden für das Debugging generiert, die mit RVIZ angezeigt werden kann.

Zu dem Modul zählen:

- ScoreCalculation Node
- Tensorboard Node
- TopDownView Node

## Installation des Projekts

Zum Installieren des Projekts inklusive gesamter Simulationsumgebung:

```
bash scripts/setup.sh
```

Dadurch werden Carla, ROS und alle notwendigen Dependencies installiert. Um alle Komponenten einzeln zu installieren,
folgende Scrips ausführen:

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
--help/-h                   zeige alle Argumente
--skip-carla-restart/-scr   die Carla Simulation wird nicht neugestartet
--build/-b                  erneutes builden des Projekts vor dem Start (schließt CARLA)
--npcs/-n                   Starten mit npcs
--low-quality/-lq           Starten in niedrigerer Qualität für leistungsschwächere Geräte
--manual-control/-mc        Starten mit Öffnen eines manual-control Fensters
--no-rules/-nr              Unterscheidung ob man mit oder ohne Regeln Fahren möchte
TownXX                      Town auf der gefahren werden soll, mögliche Towns Town01-07, Town10HD
```
