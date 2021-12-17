# Map Manager (WIP)

## Funktion

Die MapManager-Klasse aus der map_manager.py dient zum Laden und halten der Map-Daten im CommonRoad-Scenario-XML-Format. Die XML-Files im Verzeichnis Maps/Rules/ bilden die vorhandenen Fahrbahnen, erlaubten Abbiegespuren etc. exakt ab. Ampeln, Geschwindigkeitsbegrenzungen und Stoppschilder sind enthalten und stimmen mit der "Realität" überein. Die XML-Files im Verzeichnis Maps/No Rules/ wurden modifiziert, um die vorhandenen Begebenheiten optimal auszunutzen und mögliche Abkürzungen miteinzuschließen. Diese Kartendaten sind lediglich für den Gebrauch im Modus ohne Verkehrsregeln gedacht.

## Erstellen und modifizieren von Kartendaten

Für die Umwandlung der Kartendaten aus dem in Carla bereitgestellten OpenDrive-Format in ein CommonRoad-Szenario wurde das (veraltete) Opendrive2lanelet-Tool aus der CommonRoad-Toolbox verwendet. Dabei wurden große Bestandteile des Codes aus dem psaf-1-Projekt herangezogen. Der neuere CommonRoad-Scenario-Designer konnte keine zufriedenstellenden Ergebnisse liefern.
Für die manuelle Modifikation der Kartendaten (Hinzufügen von Schildern, Ampeln usw.) wurde die GUI des CommonRoad-Scenario-Designer verwendet. Die Installation kann über das Skript commonroad_designer.sh aus dem scripts-Verzeichnis des paf21-2-Repository erfolgen.

## Hinweise zur Verwendung der CRDesigner GUI

  - Lanelets können beim Erstellen eines Elements nicht durch Klick referenziert werden --> Auswahl der ID über Drop-Down-Menü
  - Wenn zu einem Schild kein Lanelet referenziert wurde, kann die XML nicht mehr geöffnet werden (vorher keine Fehlermeldung)
  - Referenzierte Lanelets eines ausgewählten Elements werden in der GUI nicht in die Drop-Down-Menüs übernommen
  - Referenzierte Lanelets können nicht gelöscht/verändert werden, alle Veränderungen werden lediglich hinzugefügt
  - Das "Additional Value" Textfeld des ersten Traffic-Sign-Elements ist buggy, um einen Wert (z.B. für ein Speedlimit) eintragen zu können empfiehlt es sich, ein zweites Traffic-Sign-Element zu erstellen und das erste nachträglich zu löschen
  - Wenn mehrere Traffic-Sign-Elements auf einmal eingetragen werden, kann in der GUI nur das letzte in der Liste gelöscht werden
