# Rückwärtsfahren mit Stanley

## 1. Erste Tests

Das Fahrzeug ist beim Rückwärtsfahren instabil, und die Lenkung ist zu spät.

Zum Testen wurde deshalb folgende Testumbung verwendet, um die Komplexität aus der Problemstellung zu nehmen und den PID und Stanley Controller einzeln zu Testen. [Github Repo](https://github.com/AtsushiSakai/PythonRobotics/blob/master/PathTracking/stanley_controller/stanley_controller.py)

## 2. Erste Beobachtung

Vorwärtsfahren funktioniert wie erwartet, keine Abweichungen aus unserer Implementierung in Carla.

Unterschiede:
- Strecke ist in vielen einzelnen Punkten aufgetragen
- Geschwindigkeit ist hier in m/s statt km/h (PSAF WS20/21 - Gruppe 2)
- Keine Physikalischen Berechnungen mit bem Fahrzeug

Normale Situation mit 30km/h vorwärts.
![Normale Situation Vorwärts, 30km/h](normal_30.png)

## 3. Änderungen zum Rückwärtsfahren

Folgende Formel:
```
# Calc front axle position
fx = state.x + L * np.cos(state.yaw)
fy = state.y + L * np.sin(state.yaw)
```

muss beim Rückwärtsfahren zu folgender Formel verändert werden:
```
# Calc front axle position
fx = state.x - L * np.cos(state.yaw)
fy = state.y - L * np.sin(state.yaw)
```

Ebenfalls muss beim Rückwärtsfahren die Geschwindigkeit negativ sein.

### 4. Beobachtungen beim Rückwärtsfahren


Normale Situation mit 30km/h rückwärts.
![backwards, 30km/h](backwards_30.png)


Bei erhöhter Geschwindigkeit rückwärts:

Rückwärts mit 60km/h:
![backwards, 60km/h](backwards_60.png)

Zu sehen ist, das der Controller am Anfang schwierigkeiten hat sich auszurichten. Es stabiliesiert sich, jedoch ist ebenfalls zu sehen, dass die Kurve nicht sauber gefahren wird.


Rückwärts mit 100km/h:
![backwards, 100km/h](backwards_100.png)

Hier ist das Phänomen noch extremer.

Rückwärts mit 150km/h:
![backwards, 150km/h](backwards_150.png)

## 4. Fazit

Es ist nicht zu empfehlen schneller als 30km/h rückwärts zu fahren, eine Erhöhte Punktedichte beim Rückwärtsfahren ist vorteilhaft.

Rückwärts fahren 30 km/h mit geringer Punktedichte:

![backwards, 30km/h, few](backwards_30_few.png)
