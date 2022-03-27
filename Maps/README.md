# Kartendaten

Die in diesem Verzeichnis als Commonroad Scenario abgespeicherten Kartendaten bilden die Grundlage für das autonome Fahren. Die Kartendaten wurden durch den [MapProvider](https://github.com/ll7/psaf1/blob/master/psaf_ros/psaf_global_planner/src/psaf_global_planner/map_provider/map_provider.py) der Gruppe 1 des PAF 2020/21 aus dem in Carla verfügbaren OpenDrive-Format in ein Commonroad Scenario konvertiert und abgespeichert. Sie werden durch den [MapManager](../paf_ros/paf_planning/src/classes/MapManager.py) geladen, wobei zwischen Karten für die Modi mit und ohne Verkehrsregeln unterschieden wird.

Die Kartendaten im Verzeichnis `/Rules/` bilden die vorhandenen Fahrbahnen der Carla-Towns exakt ab und beinhalten zusätzliche Informationen, wie Geschwindigkeitsbegrenzungen und Haltepunkte für Ampeln und Stoppschilder, die benötigt werden, damit sich das autonome Fahrzeug an die Verkehrsregeln halten kann. Nachfolgend sind beispielhaft die "Rules"-Kartendaten von Town03 abgebildet.

![](../docs/imgs/town03rules.JPG)

Die Kartendaten im Verzeichnis `/No Rules/` wurden modifiziert, um zusätzlich zu den vorhandenen Fahrbahnen auch Abkürzungen, Seitenstreifen, Gehwege etc. zur Verfügung zu stellen. Diese können im Modus ohne Verkehrsregeln genutzt werden, um Zeit durch Abkürzen und Umfahren von anderen Verkehrsteilnehmern zu sparen. Die potentiellen Abkürzungen etc. wurden durch Abfahren der Towns identifiziert und nach einer Prioritätenliste eingebaut. Weitere Modifikationen können durch den Commonroad Scenario Designer einfach vorgenommen werden (siehe [Common Road Scenario Designer](https://commonroad.in.tum.de/scenario-designer) und [CRDesigner_Modified_Files](../CRDesigner_Modified_Files)). Ampeln, Verkehrsschilder und weitere für das Einhalten von Verkehrsregeln relevanten Informationen sind unter Umständen nicht oder fehlerhaft vorhanden. Die nachfolgende Abbildung zeigt die modifizierten Kartendaten für Town03.

![](../docs/imgs/town03norules.JPG)

## Aufbau der Kartendaten:

Die Kartendaten liegen als *Commonroad Scenario* vor. Ein solches Szenario besteht unter anderem aus einem *lanelet network*, in welchem die einzelnen Fahrbahnabschnitte als *lanelets* abgespeichert sind. Ein *lanelet* beinhaltet Referenzen zu seinen Vorgängern, Nachfolgern, rechten und linken Nachbarn sowie zu Ampeln und Verkehrsschildern die auf dem Lanelet liegen. Für eine detaillierte Beschreibung des Commonroad-Szenarios und der Lanelets sei an dieser Stelle auf die [Commonroad-Website](https://commonroad.in.tum.de/tutorials/commonroad-interface) und die [Commonroad Doku](https://commonroad-io.readthedocs.io/en/latest/) verwiesen.

## Vorhandene Kartendaten:

- Town01
- Town02
- Town03
- Town04
- Town05
- Town06
- Town07
- Town10HD

## Bekannte Probleme:

- Die Haltepunkte von Ampeln und Stoppschildern liegen teils nicht genau an den Stellen, an denen das Fahrzeug tatsächlich halten sollte (z.B. an einer Haltelinie). Die Haltepunkte wurden bereits in der kompletten Town03 und vereinzelt in weiteren Maps angepasst.
- Abbiegespuren, welche auf eingebaute Abkürzungen führen, sind teilweise sehr eng, sodass das Fahrzeug der Spur nicht genau oder nur bei sehr geringen Geschwindigkeiten folgen kann, und können teilweise in Gegenspuren etc. hineinragen.
