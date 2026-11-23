# unitree_go2_navigation – erstes Nav2/SLAM-Paket

ROS-2-Paket (ament_cmake) aus der Anfangsphase des Projekts, angelegt am 25.10.2026 von Student 3 (Navigation/SLAM). Es enthält Konfigurationen für slam_toolbox und Nav2 sowie das Beispielskript `simple_commander.py`.

> **Stand 23.11.2026:** Für den echten Hund wird dieses Paket **nicht mehr verwendet**. Karte, Nav2-Parameter und Start laufen über [`../go2_bridge`](../go2_bridge/README.md) (`make_nav2_params.py`, `go2_nav2.launch.py`, `go2_mission.py`). Das Paket bleibt als Referenz und als Vorlage für eine State Machine im Repo.

---

## Inhalt

| Datei | Inhalt |
|---|---|
| `config/nav2_params.yaml` | Nav2-Parameter: MPPI-Controller, NavFn-Planner, Simple Smoother, AMCL mit `base_frame_id: base_footprint`, `odom_topic: /odom`, `robot_radius: 0.22`, VoxelLayer in der lokalen Costmap |
| `config/mapper_params_online_async.yaml` | slam_toolbox-Standardwerte mit `base_frame: base_footprint`, `scan_topic: /scan`, `mode: mapping` |
| `scripts/simple_commander.py` | Schickt den Roboter über `nav2_simple_commander` zu einem festen Ziel (x = 2,0 m, y = 1,0 m im Frame `map`) und meldet Erfolg, Abbruch oder Fehler |
| `Navigation_Ablauf.txt` | Projektstand Navigation vom 25.10.2026: erledigte Schritte, Abhängigkeiten zu den anderen Teammitgliedern, nächste Schritte |
| `CMakeLists.txt`, `package.xml` | Paketdefinition, installiert `simple_commander.py` nach `lib/unitree_go2_navigation` |

Im Projektstand erwähnt, aber **nicht im Repo enthalten**: `launch/navigation.launch.py` und der Ordner `maps/`. Die Karten liegen inzwischen in `../go2_bridge/maps/`.

---

## Warum es am echten Hund nicht passt

| Einstellung hier | Am echten Go2 nötig | Umgesetzt in |
|---|---|---|
| `base_footprint` | `base_link` (der Go2 liefert keinen `base_footprint`) | `make_nav2_params.py`, `slam_go2.yaml` |
| `odom_topic: /odom` | `/utlidar/robot_odom` | `make_nav2_params.py` |
| MPPI-Controller | DWB (leichter für den Pi 5, ohne Rotation-Shim) | `make_nav2_params.py` |
| `robot_radius: 0.22` | rechteckiger Footprint 0,64 × 0,30 m | `make_nav2_params.py` |
| `/scan` und `/tf` vorausgesetzt | werden von der Brücke erzeugt | `go2_odom_tf.py`, `go2_scan_accum.py` |
| Nav2 publiziert `/cmd_vel` | der Go2 hört nur auf die Sport-API | `go2_cmd_vel.py` + `go2_sport_sender.py` |

---

## Bauen und starten

```bash
cd ~/unitree_ws
colcon build --packages-select unitree_go2_navigation
source install/setup.bash
ros2 run unitree_go2_navigation simple_commander.py
```

**Bekannter Fehler im Skript:** Die letzte Zeile lautet `if _name_ == '_main_':` (je ein statt zwei Unterstriche). Das Skript bricht deshalb beim Start mit `NameError: name '_name_' is not defined` ab. Richtig ist:

```python
if __name__ == '__main__':
    main()
```

Nach der Korrektur wartet das Skript mit `waitUntilNav2Active()` auf einen aktiven Nav2-Stack, d. h. Bridge und `go2_nav2.launch.py` müssen laufen. Die Startpose muss in Foxglove gesetzt sein.

---

## Nutzen für die State Machine

`simple_commander.py` zeigt das Muster, das `go2_mission.py` inzwischen umsetzt: Ziel senden → auf `TaskResult.SUCCEEDED` warten → nächste Aktion auslösen. Im Missionsablauf folgt auf das Ziel das Ausrichten und Hinsetzen. Als nächste Aktion ist die Fensterprüfung mit `../yolo_zeugs/go2_integration.py` vorgesehen.
