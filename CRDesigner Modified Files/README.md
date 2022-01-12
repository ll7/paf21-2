# Modifikationen des CommonRoad-Scenario-Designers

Die Dateien in diesem Ordner sind veränderte Code-Files des CRDesigners, welche Fehler beheben und dessen Funktionalität erweitern.
Dafür müssen die entsprechenden Dateien aus dem "commonroad-scenario-designer"-Verzeichnis durch die modifizierten Dateien ersetzt werden, solange das commonroad_designer.sh-Skript das nicht automatisch macht.

## Verwendung der "Lanelet Editor"-Toolbox

Teilen von Lanelets:

Lanelets können geteilt werden, indem die Lanelet ID ausgewählt wird und die Koordinaten des Punkts, an dem das Lanelet geteilt werden soll, im "Split Lanelets"-Unterpunkt eingetragen werden. Die Werte können durch Klicken auf das Lanelet automatisch übernommen werden, solange die entsprechende Checkbox ausgewählt ist. Standardmäßig werden auch alle Nachbarn eines Lanelets geteilt. Ist dies nicht gewünscht, kann die Funktion über eine Checkbox deaktiviert werden.

Probleme/Bugs:

- Werden entgegengesetzte Nachbarn automatisch geteilt, sind die Nachbarreferenzen danach vertauscht.
- Geteilte Lanelets können teilweise nicht in der GUI durch Anklicken ausgewählt werden.
- Wird ein Lanelet, welches Nachbarn besitzt, alleine geteilt, sollten die Nachbarreferenzen danach manuell angepasst werden.
