# Kartendaten

Die in diesem Verzeichnis als Commonroad Scenario abgespeicherten Kartendaten bilden die Grundlage für das autonome Fahren. Sie werden durch den [MapManager](./paf_ros/paf_planning/src/classes/MapManager.py) geladen, wobei zwischen Karten für die Modi mit und ohne Verkehrsregeln unterschieden wird.

Die Kartendaten im Verzeichnis `/Rules/` bilden die vorhandenen Fahrbahnen der Carla-Towns exakt ab und beinhalten zusätzliche Informationen, wie Geschwindigkeitsbegrenzungen und Haltepunkte für Ampeln und Stoppschilder, die benötigt werden, damit sich das autonome Fahrzeug an die Verkehrsregeln halten kann.

Die Kartendaten im Verzeichnis `/No Rules/` wurden modifiziert, um zusätzlich zu den vorhandenen Fahrbahnen auch Abkürzungen, Seitenstreifen, Gehwege etc. zur Verfügung zu stellen. Diese können im Modus ohne Verkehrsregeln genutzt werden, um Zeit durch Abkürzen und Umfahren von anderen Verkehrsteilnehmern zu sparen. Die potentiellen Abkürzungen etc. wurden durch Abfahren der Towns identifiziert und nach einer Prioritätenliste eingebaut. Weitere Modifikationen können durch den Commonroad Scenario Designer einfach vorgenommen werden (siehe [Common Road Scenario Designer](https://commonroad.in.tum.de/scenario-designer) und [CRDesigner_Modified_Files](./CRDesigner_Modified_Files)).
