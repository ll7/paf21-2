

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

#### Validation (optional) 
Das Validation Modul dient dazu, beim Abfahren einer Route automatisch Fehler beim Verhalten des Fahrzeugs zu notieren und das Endergebnis nach dem Abfahren einer Route zu berwerten. 

## Installation des Projekts 
[TODO]
## Ausführen des Projekts 
Ausführen des von scrips/runscripts.sh \
``` bash /paf21-2/scripts/runscripts.sh ```

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



