# Modifikationen des CommonRoad-Scenario-Designers

Die Dateien in diesem Ordner sind veränderte Code-Files des CRDesigners, welche Fehler beheben und dessen Funktionalität erweitern.
Dafür müssen die entsprechenden Dateien aus dem *commonroad-scenario-designer*-Verzeichnis durch die modifizierten Dateien ersetzt werden, solange dies nicht bereits automatisch durch das **commonroad_designer.sh**-Skript erfolgt ist.

## Verwendung der "Lanelet Editor"-Toolbox

### Teilen von Lanelets:

In der **Split Lanelets**-Sektion können Lanelets geteilt werden, indem die Lanelet ID ausgewählt wird und die Koordinaten des Punkts, an dem das Lanelet geteilt werden soll, eingetragen werden. Die Werte können durch Klicken auf das Lanelet automatisch übernommen werden, solange die entsprechende Checkbox ausgewählt ist. Standardmäßig werden auch alle Nachbarn eines Lanelets geteilt. Ist dies nicht gewünscht, kann die Funktion über eine weitere Checkbox deaktiviert werden.

### Erstellen von neuen Lanelets:

Neue Lanelets können über die **Create Lanelet**-Sektion einfach erstellt werden. Durch Klicken auf die Karte können Wegpunkte festgelegt werden, welche die Center-Line des neuen Lanelets darstellen. Durch Abwählen der entsprechenden Checkbox kann diese Funktion deaktiviert werden. Die Wegpunkte werden in eine Tabelle aufgenommen, wo sie einzeln entfernt oder bearbeitet werden können. Die Breite des Lanelet beträgt standardmäßig 4.0 und kann über ein Textfeld angepasst werden. Für alle weiteren Eigenschaften des Lanelets werden Standardwerte verwendet. Durch einen Klick auf den Button *Create Lanelet* wird das Lanelet der Karte hinzugefügt. Dabei kann vorher über die darüberliegende Checkbox ein Optimierungsverfahren aktiviert werden, wobei der Verlauf der gesetzten Wegpunkte mithilfe von Beziérkurven geglättet wird. Hier muss beachtet werden, dass das resultierende Lanelet nicht mehr exakt dem vorgegebenen Verlauf folgt. Ist ein präzises Lanelet gefordert, sollte die Optimierung deaktiviert werden.

### Hinweise zur Verwendung:

- Werden neben einem Lanelet auch dessen Nachbarn automatisch geteilt, sind die Nachbarreferenzen danach unter Umständen vertauscht.
- Wird ein Lanelet, welches Nachbarn besitzt, alleine geteilt, sollten die Nachbarreferenzen danach manuell angepasst werden.
- Beim Festlegen der Center-Line eines neuen Lanelets durch Klicken auf die Map sollte beachtet werden, dass diese beim Klicken manchmal springt.
