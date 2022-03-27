# Map Manipulation

Das Python-Skript `map_manipulation.py` aus diesem Package dient der (automatisierten) Bearbeitung der Kartendaten. Die Verwendung ist für Änderungen sinnvoll, welche leicht in Code implementiert werden können und welche mit Hilfe des Commonroad Scenario Designers nur mit erhöhtem Aufwand vorgenommen werden können. Das Skript ermöglicht beispielsweise das Löschen aller Verkehrsschilder eines gewissen Typs oder das Entfernen aller Ampeln mit geringem Arbeitsaufwand. Hierfür werden Methoden zum Laden und Abspeichern der Commonroad-Szenario-XML-Dateien bereitgestellt. Bearbeitete Karten werden im `/Temporary Maps/`-Verzeichnis abgespeichert.

**Hinweis**: Das Skript ist zur Bearbeitung von Kartendaten offline gedacht und wird zur Laufzeit des Projekts nicht verwendet.

## Verwendung:

1. Implementieren der erwünschten Änderung(en) als Methode(n) der `MapManipulator`-Klasse.
2. Anpassen der `_load_scenario`-Methode, sodass die korrekte Map geladen wird.
3. Anpassen der `_generate_cr_file`-Methode, sodass die Karte mit gewünschtem Dateinamen abgespeichert wird.
4. In Methode `main()`: MapManipulator-Objekt generieren, Szenario laden, Methoden zur Anpassung aufrufen, Szenario abspeichern.
5. Skript ausführen.

## Bereits implementierte Funktionalität:

- Erstellung eines inversen Szenarios, in welchem jedes Lanelet invertiert ist
- Erstellung eines bidirektionalen Szenarios, in welchem jedes Lanelet in beide Richtungen befahrbar ist
- Löschen von allen Ampeln und Verkehrsschildern
- Ändern der ID eines Lanelets, sowie Anpassung aller Referenzen
- Entfernen aller Vorgänger/Nachfolger-Referenzen, bei denen die Lanelets nicht aneinander anschließen
