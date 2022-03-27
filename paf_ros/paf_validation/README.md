# Validation Package

Das Validation Package ist verantwortlich für Test und Validierung und wird zur Wettbewerbszeit nicht benötigt. Die
Nodes dieses Packages werden mit dem Flag `--validation` bzw `-val` aktiviert.

## TopDownView

Diese Node ist dafür verantwortlich, eine Live-Karte zu generieren, und diese für RVIZ bereitzustellen. Diese stellt alle CARLA-Aktoren, den globalen und lokalen Pfad und beliebige Debug-Punkte dar. Karte und
Aktor-Positionen sind der CARLA-Simulation selbst entnommen und können so leicht mit den Sensordaten verglichen werden.
Die Implementierung basiert auf dem pip-paket `carla_birdeye_view`. Die Hauptklasse dieses Pakets `BirdViewProducer`
wird in `classes/TopDownView.py` geerbt und erweitert.

Beliebige Punkte können jederzeit mit dem Message-Typ `PafTopDownViewPointSet` als verbundene Linien oder Einzelpunkte
als änderbarer dict-Eintrag an die Node übermittelt werden.

**Achtung**: Beim ersten Start mit einer neuen Auflösung wird ein Cache-Objekt im `~/.ros/` Ordner angelegt, daher kann es bis zu 30s dauern, bis das entsprechende Bild in RVIZ gezeigt wird.

Parameter werden in `paf_validation/parameters/top_down_view.yaml` definiert, davon sind folgende zum Testen relevant:

- `update_hz` Angestrebte Framerate, die an RVIZ gesendet wird.
- `img_size` Größe des zu generierenden Bildes
- `pixels_per_meter` Auflösung des zu generierenden Bildes
- `north_is_up` Norden oder Fahrzeugorientierung oben
- `dark_mode` Dunkler Modus
- `show_whole_map` Ganze Karte zeigen mit gegebener Auflösung (sehr langsam)

Subscriber:

- `/paf/paf_perception/obstacles` (`PafObstacleList`)
- `/paf/paf_local_planner/path` (`PafLocalPath`)
- `/paf/paf_validation/draw_map_points` (`PafTopDownViewPointSet`)
- `/paf/paf_validation/draw_map_lines` (`PafTopDownViewPointSet`)
- `/paf/paf_validation/speed_text` (`PafSpeedMsg`) verantwortlich für den Info-Text (Geschwindigkeit und Position) auf der linken Seite des Bildes

Publisher:

- `/paf/paf_validation/top_down_view` (`Image`) resultierendes Bild für RVIZ

## Score Calculation

Die Score Calculation Node ist verantwortlich für Zeitmessung von Start- zu Zielpunkt und Erfassung von Fail-States.
Dabei sind allerdings nicht alle Fail-States aus der Aufgabenstellung erfasst. Das Ergebnis wird nach dem stop-Kommando auf der Konsole ausgegeben und auf dem Tensorboard veröffentlicht.

Parameter:

- `EVENT_THRESHOLD_SECS` Mindestanzahl an Sekunden zwischen zwei Events. Ansonsten werden diese nur als ein Event gewertet.
- `MIN_RUN_TIME` Minimale Zeit (in Sekunden) zwischen Start- und Zielzeit, sonst keine Wertung
- `MAX_RUN_TIME` Maximale Zeit (in Sekunden) zwischen Start- und Zielzeit, sonst keine Wertung

Subscriber:

- `/carla/ego_vehicle/collision` (`CarlaCollisionEvent`) Kollisionsevent mit Actor oder Umgebung
- `/carla/ego_vehicle/lane_invasion` (`CarlaLaneInvasionEvent`) Linienüberschreitungsevent
- `/paf/paf_validation/score/start` (`Empty`) Timer starten
- `/paf/paf_validation/score/stop` (`Empty`) Timer beenden
- `carla/ego_vehicle/odometry` (`Odometry`)

Publisher:

- `/paf/paf_validation/tensorboard/scalar` (`PafLogScalar`) Ergebnis für Tensorboard

## Tensorboard

Diese Node kann Bilder, Skalare und Text in einem intuitiven Webinterface darstellen. Tensorboard muss zur Anzeige zunächst mit dem Skript `tensorboard.sh` (`scripts`-Ordner) gestartet werden. Das Logging der Werte geht allerdings auch ohne das Starten des Webinterfaces. Die Log-Dateien werden unter `~/.ros/tensorboard/` gespeichert und können je nach Logging-Verhalten auch recht groß werden. Zum Aufräumen kann der Befehl `find ~/.ros/tensorboard/ -mtime +3 -exec rm -rf {} \;` verwendet werden (löscht alle Dateien älter als 3 Tage).

Die x-Achse kann nur ganze Schritte darstellen, daher kann in den Messages das gewünschte Verhalten gewählt werden:
- `step_as_distance=True` bedeutet, dass die x-Achse die gefahrene Strecke in Metern darstellt.
- `step_as_distance=False` bedeutet, dass die x-Achse die vergangene Zeit in `0.1s`-Schritten darstellt.

Subscriber:

- `/paf/paf_validation/tensorboard/scalar` (`PafLogScalar`)
- `/paf/paf_validation/tensorboard/text` (`PafLogText`)
- `/paf/paf_validation/tensorboard/image` (`PafLogImage`)
- `carla/ego_vehicle/odometry` (`Odometry`)
