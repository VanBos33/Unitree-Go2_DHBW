# go2_bridge – Go2 EDU ↔ ROS 2 Jazzy: SLAM, Nav2 und Missions-Steuerung

Stand: **23.11.2026**, getestet am echten Go2 EDU (Raspberry Pi 5, Ubuntu 24.04, ROS 2 Jazzy, LAN-Kabel Pi ↔ Hund).

Dieser Ordner enthält alles, was der echte Hund für autonome Navigation braucht:

- **Brücke Hund → ROS:** TF `odom → base_link` und ein dichter 2D-Scan `/scan` aus dem L1-LiDAR
- **Nav2-Konfiguration** für den Vierbeiner (Parameter-Generator, Launch-Datei, Karten)
- **Brücke ROS → Hund:** `/cmd_vel` → Unitree Sport-API (Laufen, Stoppen, Sitzen, Aufstehen)
- **Missions-Steuerung:** Ziel auf der Karte → hinlaufen → ausrichten → hinsetzen → warten

Der Ordner ist **kein colcon-Paket**. Alle Skripte werden direkt mit `python3` bzw. `ros2 launch` und vollem Dateipfad gestartet.

---

## 1. Dateien

| Datei | Typ | Zweck |
|---|---|---|
| `go2.sh` | Shell | **Ein Befehl für alles:** startet/überwacht/beendet alle Programme in einer tmux-Sitzung mit Tabs (Abschnitt 9) |
| `go2_env.sh` | Shell | Umgebung für ROS ↔ Go2. **In jedem ROS-Terminal zuerst `source`n** (nicht für den Sport-Sender). |
| `go2_bridge.launch.py` | Launch | Startet statischen TF `base_link → utlidar_lidar`, `go2_odom_tf.py` und `go2_scan_accum.py` |
| `go2_odom_tf.py` | Node | `/utlidar/robot_odom` → TF `odom → base_link` |
| `go2_scan_accum.py` | Node | `/utlidar/cloud_deskewed` → dichter `/scan` (LaserScan) |
| `go2_cloud_fix.py` | Node | Früherer Lösungsversuch (Frame von `/utlidar/cloud` umschreiben). **Wird nicht mehr benutzt**, von keiner Launch-Datei gestartet. |
| `go2_cmd_vel.py` | Node | ROS-Seite der Fahr-Brücke: `/cmd_vel` → UDP `127.0.0.1:9870` |
| `go2_sport_sender.py` | Skript | SDK-Seite der Fahr-Brücke: UDP → Unitree `SportClient` (Move, StopMove, Sit, RiseSit, BalanceStand) |
| `go2_mission.py` | Node | Missions-Steuerung: `/mission_goal` → Nav2 → ausrichten → sitzen |
| `go2_sit_test.py` | Skript | Hinsetzen/Aufstehen von Hand über den Sport-Sender |
| `make_nav2_params.py` | Skript | Erzeugt `nav2_go2_params.yaml` aus den Nav2-Standardwerten der installierten Version + Go2-Anpassungen |
| `nav2_go2_params.yaml` | YAML | **Erzeugt** von `make_nav2_params.py`, nicht von Hand ändern |
| `go2_nav2.launch.py` | Launch | Startet `nav2_bringup` mit Karte (Standard `maps/raum3.yaml`) und `nav2_go2_params.yaml` |
| `reset_nav2_after_pose.sh` | Shell | Nach dem Setzen der Startpose: Costmaps leeren, bt_navigator prüfen/aktivieren |
| `slam_go2.yaml` | YAML | slam_toolbox-Einstellungen (`base_frame: base_link`, `scan_topic: /scan`) |
| `go2.rviz` | RViz | Gespeicherte RViz-Ansicht (nur am Pi-Monitor nutzbar) |
| `maps/` | Karten | Siehe [`maps/README.md`](maps/README.md) |

---

## 2. Datenfluss

```
Go2 ──DDS/eth0──> /utlidar/robot_odom ──> go2_odom_tf.py ──> TF odom → base_link
               └> /utlidar/cloud_deskewed ──> go2_scan_accum.py ──> /scan
                                                   (nutzt TF + Odometrie)

TF-Baum:  map ──(AMCL bzw. slam_toolbox)──> odom ──(go2_odom_tf)──> base_link ──(statisch)──> utlidar_lidar

Foxglove ─/mission_goal─> go2_mission.py ─NavigateToPose─> Nav2 ─/cmd_vel─> go2_cmd_vel.py
                               │                                                │
                               └──── UDP {"action": "sit"/"stand"} ────┐        │ UDP {"x","y","z"}
                                                                        ▼        ▼
                                                             go2_sport_sender.py (Port 9870)
                                                                        │ unitree_sdk2py
                                                                        ▼
                                                                     Go2 Sport-API
```

### Welche Punktwolke?

| Topic | Frame im Header | Bewertung |
|---|---|---|
| `/utlidar/cloud` | `utlidar_lidar` | ❌ Header falsch, Daten sind weltausgerichtet (Wände drehen sich in RViz mit dem Hund) |
| `/utlidar/cloud_base` | `base_link` | ✅ nur noch für `accum:=false` |
| `/utlidar/cloud_deskewed` | `odom` | ✅ Quelle für den dichten `/scan` |

---

## 3. Die Nodes im Detail

### 3.1 `go2_env.sh`

```bash
source /opt/ros/jazzy/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
# eigenes CycloneDDS aus ~/unitree_ws ausblenden (sonst Absturz "buffer overflow" mit eth0)
export LD_LIBRARY_PATH=$(echo "$LD_LIBRARY_PATH" | tr ':' '\n' | grep -v unitree_ws | paste -sd:)
export CYCLONEDDS_URI='<CycloneDDS><Domain><General><Interfaces><NetworkInterface name="eth0" priority="default" multicast="default"/></Interfaces></General></Domain></CycloneDDS>'
```

**Nicht** für `go2_sport_sender.py` verwenden. Das Unitree-SDK bringt sein eigenes CycloneDDS mit und verträgt die ROS-Einstellungen nicht.

### 3.2 `go2_odom_tf.py`

| Parameter | Standard | Bedeutung |
|---|---|---|
| `odom_topic` | `/utlidar/robot_odom` | Quelle |
| `odom_frame` / `base_frame` | `odom` / `base_link` | Frames des TF |
| `use_pi_time` | `false` (Launch: `true`) | Zeitstempel in Pi-Zeit verschieben |
| `offset_alpha` | `0.02` | Glättungsfaktor (EMA) für den Uhren-Offset |

Bei `use_pi_time:=true` wird der **Hardware-Zeitstempel behalten** und nur um einen geglätteten Offset `Pi-Uhr − Go2-Uhr` verschoben. Springt der Offset um mehr als 0,5 s (Hund neu gestartet), wird er neu gesetzt. Früher wurde jeder Stempel durch die Ankunftszeit ersetzt. Dadurch passte beim Drehen die Odometrie nicht mehr zu den Punktwolken, und die Karte verrutschte.

### 3.3 `go2_scan_accum.py`

Eine einzelne L1-Wolke hat nur wenige Punkte, ein `/scan` daraus wäre fast leer, und slam_toolbox würde eine „Farbeimer“-Karte erzeugen. Der Node sammelt deshalb Wolken über ein Zeitfenster (im Frame `odom`, also weltfest), rechnet sie mit dem aktuellen TF in `base_link` um, schneidet ein Höhenband aus und veröffentlicht einen 360°-LaserScan.

| Parameter | Standard | Bedeutung |
|---|---|---|
| `cloud_topic` | `/utlidar/cloud_deskewed` | Quelle (Frame `odom`) |
| `odom_topic` | `/utlidar/robot_odom` | für Uhren-Offset und Drehrate |
| `window` | `0.30` (Launch: `accum_window`) | Sammelzeit beim Geradeauslaufen in s |
| `window_rot` | `0.18` (Launch: `accum_window_rot`) | Sammelzeit beim Drehen in s |
| `rot_rate` | `0.15` rad/s | ab dieser Drehrate gilt „dreht“ |
| `rot_yaw` | `0.08` rad | oder ab dieser Gieränderung über den Puffer |
| `min_height` / `max_height` | `-0.10` / `0.40` m | Höhenband relativ zu `base_link` |
| `range_min` / `range_max` | `0.30` / `15.0` m | Entfernungsgrenzen |
| `angle_increment_deg` | `1.0` | Auflösung, ergibt 360 Strahlen |
| `rate` | `8.0` Hz | Scan-Rate |
| `use_pi_time` | `false` (Launch: `true`) | wie bei `go2_odom_tf.py` |

- **Dreherkennung:** Beim Drehen wird nur das kurze Fenster genutzt, damit der Scan nicht verschmiert. Im Log erscheint dann `Drehung erkannt -> Fenster 0.18 s` bzw. `Geradeaus -> Fenster 0.25 s`.
- **TF-Lookup:** zuerst zum Zeitpunkt der neuesten Wolke, sonst der neueste verfügbare TF. Der Scan trägt den Zeitstempel des tatsächlich verwendeten TF.
- **Typische Werte:** 85–120 von 360 Strahlen mit Treffer bei 4 Wolken (`accum_window:=0.25`).

### 3.4 `go2_bridge.launch.py`

```bash
ros2 launch ~/unitree_ws/src/go2_bridge/go2_bridge.launch.py accum_window:=0.25   # Nav2
ros2 launch ~/unitree_ws/src/go2_bridge/go2_bridge.launch.py accum_window:=0.6    # SLAM-Kartierung
```

| Argument | Standard | Bedeutung |
|---|---|---|
| `accum` | `true` | `true` = `go2_scan_accum.py`, `false` = `pointcloud_to_laserscan` aus `cloud_topic` |
| `accum_window` | `0.35` | Sammelzeit geradeaus (Nav2: `0.25`, SLAM: `0.6`) |
| `accum_window_rot` | `0.18` | Sammelzeit beim Drehen |
| `use_pi_time` | `true` | Uhren-Offset-Korrektur |
| `min_height` / `max_height` | `-0.10` / `0.40` | Höhenband |
| `range_min` / `range_max` | `0.30` / `20.0` | Entfernungsgrenzen |
| `cloud_topic` | `/utlidar/cloud_base` | nur bei `accum:=false` |
| `lidar_x/y/z`, `lidar_roll/pitch/yaw` | `0.28945 / 0 / -0.046825`, `0 / 2.8782 / 0` | Einbaulage L1 laut Unitree-URDF (`radar_joint`) |

### 3.5 `go2_cmd_vel.py` (ROS-Seite der Fahr-Brücke)

Liest `/cmd_vel`, formt die Befehle für den Go2 und schickt sie per UDP an den Sport-Sender.

| Parameter | Standard | Bedeutung |
|---|---|---|
| `max_vx` | `0.35` m/s | Höchstgeschwindigkeit vorwärts |
| `allow_reverse` | `true` | Rückwärts erlaubt (für Nav2-Recovery „BackUp“) |
| `max_reverse` | `0.15` m/s | maximal rückwärts |
| `min_reverse` / `reverse_boost_below` | `0.10` / `0.08` m/s | kleine Rückwärtsbefehle werden auf 0,10 angehoben, sonst lehnt sich der Hund nur nach hinten |
| `allow_strafe` | `false` | seitwärts gesperrt |
| `max_vy` / `vy_deadband` | `0.15` / `0.08` m/s | nur mit `allow_strafe:=true` |
| `max_vyaw` | `0.40` rad/s | maximale Drehrate |
| `yaw_deadband` | `0.06` rad/s | kleinere Drehbefehle werden ignoriert |
| `min_yaw` | `0.30` rad/s | Mindestdrehrate **auf der Stelle**. Darunter dreht der Go2 nicht (getestet: 0,22 reicht nicht). Beim Laufen gilt kein Mindestwert, sonst verfälscht er die Kurven von DWB. |
| `deadband` | `0.02` m/s | Totband für `vx` |

Beispiel: `python3 go2_cmd_vel.py --ros-args -p allow_strafe:=true -p max_vx:=0.25`

### 3.6 `go2_sport_sender.py` (SDK-Seite der Fahr-Brücke)

Empfängt UDP auf `127.0.0.1:9870` und ruft das Unitree-SDK auf. Beim Start sendet er `BalanceStand()`, damit `Move()` den Hund laufen lässt und nicht nur neigt.

| UDP-Nachricht | Aktion | Antwort an den Absender |
|---|---|---|
| `{"x": .., "y": .., "z": ..}` | `Move(x, y, z)` | – |
| `{"stop": true}` | `StopMove()` | – |
| `{"action": "sit"}` | `StopMove()`, `Sit()` | `{"ack": "sit", "ok": …, "info": …}` |
| `{"action": "stand"}` | `RiseSit()`, `BalanceStand()` | `{"ack": "stand", "ok": …, "info": …}` |

- **Watchdog:** 0,5 s ohne Befehl führt zu `StopMove()`, auch wenn die ROS-Seite abstürzt.
- **Im Sitzen** werden alle Fahrbefehle ignoriert.
- **Aufstehen gilt als erfolgreich, wenn `BalanceStand()` klappt.** `RiseSit()` meldet `-1`, wenn der Hund schon steht. Das ist kein Fehler.
- `--sit-mode standdown` nutzt `StandDown()`/`StandUp()` (Hinlegen) statt `Sit()`/`RiseSit()`, falls eine Firmware `Sit()` nicht unterstützt. Am Projekt-Go2 funktioniert `Sit()` (Rückgabe 0).
- **Nicht neu starten, während der Hund sitzt:** Das `BalanceStand()` beim Start lässt ihn aufstehen.

### 3.7 `go2_mission.py` (Missions-Steuerung)

```
            neues Ziel                Nav2: SUCCEEDED            |Fehler| < 0,10 rad
  IDLE ───────────────> NAVIGATING ───────────────> ALIGNING ───────────────────> SITTING_DOWN
   ▲                        │ Nav2: Fehler/Abbruch                                     │ ack sit
   └────────────────────────┘                                                          ▼
                                                                                    SITTING
  SETTLING (1 s) <── ack stand ── STANDING_UP <──────────── neues Ziel ─────────────┘
      └────────> NAVIGATING
```

- **Eingang:** `geometry_msgs/PoseStamped` auf `/mission_goal` (Foxglove „Publish pose“, Frame `map`). Position **und** Blickrichtung.
- **Navigation:** Nav2-Action `navigate_to_pose`. Ein neues Ziel ersetzt das laufende sofort.
- **Ausrichten:** Nav2 nimmt die Blickrichtung nur auf ±0,35 rad genau. Danach dreht die Mission den Hund über `/cmd_vel` auf ±0,10 rad (~6°). Nach dreimaligem Überdrehen oder 15 s wird das Ausrichten beendet.
- **Sitzen/Aufstehen:** per UDP an den Sport-Sender, mit Quittung (Timeout 12 s).
- **Fehlschlag:** Erreicht Nav2 das Ziel nicht (`Status 5` = abgebrochen, `Status 6` = gescheitert), bleibt der Hund stehen und die Mission geht auf `IDLE`.

| Parameter | Standard | Bedeutung |
|---|---|---|
| `goal_topic` | `/mission_goal` | Eingang |
| `map_frame` / `base_frame` | `map` / `base_link` | Frames |
| `sit_on_arrival` | `true` | am Ziel hinsetzen |
| `align_tolerance` | `0.10` rad | Genauigkeit der Blickrichtung |
| `align_timeout` | `15.0` s | maximale Ausrichtzeit |
| `align_max_rate` | `0.40` rad/s | maximale Drehrate beim Ausrichten |
| `stand_settle` | `1.0` s | Wartezeit nach dem Aufstehen |
| `ack_timeout` | `12.0` s | Wartezeit auf die Quittung des Sport-Senders |

### 3.8 `go2_sit_test.py`

```bash
python3 go2_sit_test.py          # hinsetzen, 5 s warten, aufstehen
python3 go2_sit_test.py sit      # nur hinsetzen
python3 go2_sit_test.py stand    # nur aufstehen
```

Erwartet: `<- OK: Sit() Rückgabe 0` bzw. `<- OK: RiseSit() Rückgabe 0, BalanceStand() Rückgabe 0`. Wer den Hund von Hand hinsetzt, muss ihn vor dem nächsten Missionsziel auch von Hand aufstehen lassen.

### 3.9 `reset_nav2_after_pose.sh`

Nach jeder neuen Startpose in Foxglove ausführen. Das Skript leert `/local_costmap` und `/global_costmap` (Geisterwände von der falschen Pose verschwinden) und prüft den Zustand des `bt_navigator`: Ist er `inactive`, wird er fortgesetzt, ist er `unconfigured`, wird Nav2 gestartet.

---

## 4. Nav2-Konfiguration

`nav2_go2_params.yaml` wird **auf dem Pi** erzeugt, damit es zur installierten Nav2-Version passt:

```bash
python3 ~/unitree_ws/src/go2_bridge/make_nav2_params.py
```

Grundlage ist `/opt/ros/jazzy/share/nav2_bringup/params/nav2_params.yaml`. Überall wird `base_footprint` durch `base_link` ersetzt und `/odom` durch `/utlidar/robot_odom`. Dazu kommen diese Anpassungen:

| Bereich | Einstellung | Grund |
|---|---|---|
| **AMCL** | `OmniMotionModel`, `alpha1..5 = 0.4/0.4/0.25/0.4/0.25` | Der Vierbeiner rutscht beim Drehen auch seitlich, die Dreh-Odometrie ist ungenau |
| | `update_min_d = update_min_a = 0.08` | häufigere Updates beim Drehen |
| | `max_beams 120`, Partikel `500–2500`, `transform_tolerance 1.0` | |
| | `set_initial_pose: true` (0, 0, 0) | Nav2 startet ohne Warten auf `map`. Die echte Pose wird in Foxglove gesetzt. |
| **Controller** | reiner `dwb_core::DWBLocalPlanner`, **kein** RotationShimController | Drehen auf der Stelle erzeugt den meisten Schlupf und verschmiert den Scan |
| | `max_vel_x 0.35`, `min_vel_x 0.0`, `max_vel_theta 0.40`, `min_speed_theta 0.18`, kein Seitwärts | |
| | `acc_lim_x 0.6`, `acc_lim_theta 0.8`, `sim_time 1.7`, 15 × 20 Samples, `controller_frequency 10` | |
| | Critics: `PathAlign 24`, `GoalAlign 16`, `PathDist 32`, `GoalDist 24`, `RotateToGoal 16`, `BaseObstacle 0.02` | |
| **Progress Checker** | `required_movement_radius 0.15`, `movement_time_allowance 20 s` | langsames Drehen gilt nicht sofort als „kein Fortschritt“ |
| **Goal Checker** | `xy_goal_tolerance 0.25`, `yaw_goal_tolerance 0.35` | die Feinausrichtung übernimmt `go2_mission.py` |
| **Costmaps** | Footprint `[[0.32, 0.15], [0.32, -0.15], [-0.32, -0.15], [-0.32, 0.15]]` | Go2-Rumpf, etwas knapp für enge Gänge |
| | `inflation_radius 0.28`, `cost_scaling_factor 4.0` | der Hund „klebt“ nicht an Wänden |
| | Scan: `raytrace 0.20–12.0 m`, `obstacle 0.25–8.0 m`, `clearing`/`marking` an | |
| | global: `track_unknown_space`, Planner `allow_unknown` | |
| **Behaviors** | Spin `0.20–0.35 rad/s`, `rotational_acc_lim 0.8` | langsame Recovery-Drehungen |
| **Velocity Smoother** | `max [0.35, 0, 0.40]`, `min [-0.15, 0, -0.40]`, `OPEN_LOOP` | lässt das BackUp-Recovery rückwärts durch |
| **BT-Navigator** | `default_server_timeout 500 ms`, `wait_for_service_timeout 2000 ms` | der Pi braucht länger als die Standard-20 ms |
| **Collision Monitor** | `FootprintApproach` deaktiviert | bremste den Hund durch Scan-Punkte am Rumpf ständig auf 0 |

Die Warnung `inflation radius … smaller than circumscribed radius` ist bekannt und unkritisch. Zusammenstöße verhindert DWB über die Footprint-Prüfung.

```bash
ros2 launch ~/unitree_ws/src/go2_bridge/go2_nav2.launch.py                       # Karte raum3
ros2 launch ~/unitree_ws/src/go2_bridge/go2_nav2.launch.py map:=$HOME/unitree_ws/src/go2_bridge/maps/raum1.yaml
```

---

## 5. Start-Reihenfolge

Jedes Programm in einem eigenen SSH-Fenster (`ssh fynn@100.121.161.1`):

```bash
# 1 – Bridge
source ~/unitree_ws/src/go2_bridge/go2_env.sh
ros2 launch ~/unitree_ws/src/go2_bridge/go2_bridge.launch.py accum_window:=0.25

# 2 – Sport-Sender (OHNE go2_env.sh)
python3 ~/unitree_ws/src/go2_bridge/go2_sport_sender.py

# 3 – cmd_vel
source ~/unitree_ws/src/go2_bridge/go2_env.sh
python3 ~/unitree_ws/src/go2_bridge/go2_cmd_vel.py

# 4 – Nav2
source ~/unitree_ws/src/go2_bridge/go2_env.sh
ros2 launch ~/unitree_ws/src/go2_bridge/go2_nav2.launch.py

# 5 – Foxglove-Bridge (Mac: Foxglove-App → ws://100.121.161.1:8765)
source ~/unitree_ws/src/go2_bridge/go2_env.sh
ros2 launch foxglove_bridge foxglove_bridge_launch.xml

# 6 – Mission
source ~/unitree_ws/src/go2_bridge/go2_env.sh
python3 ~/unitree_ws/src/go2_bridge/go2_mission.py
```

Danach in Foxglove die Startpose setzen (*Publish pose estimate*), `reset_nav2_after_pose.sh` ausführen und mit *Publish pose* auf `/mission_goal` das Ziel setzen.

**Kurztests:**

```bash
ros2 topic hz /scan                          # ~8 Hz
ros2 run tf2_ros tf2_echo odom base_link     # Werte ändern sich beim Bewegen
ros2 topic echo /cmd_vel                     # Fahrbefehle von Nav2
```

**Mit Tastatur fahren** (statt Nav2, Fenster 2 und 3 müssen laufen):

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

`i` vor, `,` zurück, `j`/`l` drehen, `k` stopp, `z` langsamer.

---

## 6. Fehlerbilder aus dem Betrieb

| Log-Meldung / Verhalten | Bedeutung und Abhilfe |
|---|---|
| `Failed to make progress` + Hund dreht sich lange | Lokalisierung falsch. Abbrechen, Pose neu setzen, Reset-Skript ausführen. |
| `Running spin` / `Running backup` | Nav2-Recovery: drehen bzw. bis 0,15 m/s rückwärts. Normal. |
| `Control loop missed its desired rate of 10.0000 Hz` | Der Pi schafft knapp 10 Hz. Unkritisch, Abhilfe: `controller_frequency` auf 8. |
| `Failed to get parameters: 'dock_database'` / `'FootprintApproach.max_points'` | Foxglove fragt ungenutzte Parameter ab. Harmlos. |
| Mission: `Nav2 hat das Ziel nicht erreicht (Status 5)` | Ziel wurde abgebrochen (z. B. per `cancel_goal`). |
| Mission: `Nav2 hat das Ziel nicht erreicht (Status 6)` | Nav2 ist gescheitert (kein Fortschritt, keine Route). |
| Mission: `Aufstehen fehlgeschlagen … bleibe sitzen` | Sport-Sender mit alter Logik oder Hund nicht bereit. Mission neu starten. |
| Sport-Sender: `Aktion 'stand': FEHLER (RiseSit() Rückgabe -1 …)` | Alte Version: Der Hund stand schon. Seit 23.11. wird das als Erfolg gewertet. |
| Alle Nodes: `ddsi_udp_conn_write … failed with retcode -1` | LAN-Verbindung weg (Hund aus). Alles beenden und neu starten. |
| Programm reagiert nicht auf control + C | Aus einem freien Fenster zwangsweise beenden, z. B. `pkill -9 -f go2_cmd_vel.py` oder `pkill -9 -f go2_mission.py`. |
| `Failed to publish: publisher's context is invalid` beim Beenden der Mission | Kosmetisch, in der aktuellen `go2_mission.py` abgefangen. |
| `*** buffer overflow detected ***` | Workspace-CycloneDDS mit `eth0`. Immer `go2_env.sh` sourcen. |

---

## 7. Einmalige Installation

```bash
sudo apt install -y ros-jazzy-rmw-cyclonedds-cpp ros-jazzy-pointcloud-to-laserscan \
  ros-jazzy-slam-toolbox ros-jazzy-navigation2 ros-jazzy-nav2-bringup \
  ros-jazzy-foxglove-bridge ros-jazzy-rviz2 ros-jazzy-teleop-twist-keyboard
chmod +x ~/unitree_ws/src/go2_bridge/reset_nav2_after_pose.sh
python3 ~/unitree_ws/src/go2_bridge/make_nav2_params.py
```

`go2_sport_sender.py` braucht das Python-SDK `unitree_sdk2py` mit dem Paket `cyclonedds==0.10.2` (`CYCLONEDDS_HOME=~/unitree_ws/cyclonedds/install`).

---

## 8. RViz (nur am Pi-Monitor)

```bash
source ~/unitree_ws/src/go2_bridge/go2_env.sh
rviz2 -d ~/unitree_ws/src/go2_bridge/go2.rviz
```

Bei LaserScan und PointCloud2 `Reliability Policy = Best Effort`, bei `/map` Durability `Transient Local`. Über SSH ist die Foxglove-App die bessere Wahl.

---

## 9. `go2`: ein Fenster statt vieler

`go2.sh` startet alle Programme in einer tmux-Sitzung namens `go2`. Jedes Programm bekommt einen Tab, und am Mac braucht man nur noch ein Fenster.

| Tab | Inhalt | mit `go2_env.sh` | „bereit“, wenn im Log steht |
|---|---|---|---|
| `0:Status` | `go2 status` alle 3 s | – | – |
| `1:Bridge` | `ros2 launch go2_bridge.launch.py accum_window:=0.25` | ja | `Strahlen mit Treffer` |
| `2:Sport` | `python3 go2_sport_sender.py` | **nein** (Unitree-SDK) | `SportClient bereit` |
| `3:cmd_vel` | `python3 go2_cmd_vel.py` | ja | `Bereit: /cmd_vel` |
| `4:Nav2` | `ros2 launch go2_nav2.launch.py` | ja | `Managed nodes are active` |
| `5:Foxglove` | `ros2 launch foxglove_bridge foxglove_bridge_launch.xml` | ja | `listening` bzw. `8765` |
| `6:Mission` | `python3 go2_mission.py` | ja | `Bereit. Ziele` |
| `7:Kamera` | `python3 ../yolo_zeugs/go2_tuer_stream.py auto --ohne-ki` | **nein** (Unitree-SDK) | `8080` |
| `8:Befehle` | freie Shell mit geladener `go2_env.sh` | ja | – |

Die tmux-Sitzung startet mit einer **leeren Umgebung** (`env -i`). So erben Sport-Sender und Kamera keine ROS-Variablen, selbst wenn `go2 start` aus einer Shell mit geladener `go2_env.sh` aufgerufen wird.

**Befehle:** `go2 start`, `go2 stop`, `go2 restart`, `go2 status`, `go2 oeffnen`, `go2 neu mission` (oder `bridge`, `sport`, `cmd_vel`, `nav2`, `foxglove`, `kamera`), `go2 reset`, `go2 abbruch`, `go2 sitz`, `go2 steh`, `go2 hilfe`.

**Beenden:** Erst Strg+C an das Programm, nach 15 s ohne Ende wird der ganze Prozessbaum mit `kill -9` beendet.

### Einrichtung auf dem Pi (einmalig)

```bash
chmod +x ~/unitree_ws/src/go2_bridge/go2.sh
mkdir -p ~/.local/bin
ln -sf ~/unitree_ws/src/go2_bridge/go2.sh ~/.local/bin/go2
sudo apt install -y tmux
```

### Einrichtung am Mac (einmalig, im Mac-Terminal)

```bash
# 1. SSH-Schlüssel erzeugen (falls noch keiner da ist) und auf den Pi kopieren – danach kein Passwort mehr
[ -f ~/.ssh/id_ed25519 ] || ssh-keygen -t ed25519 -N "" -f ~/.ssh/id_ed25519
ssh-copy-id -i ~/.ssh/id_ed25519.pub fynn@100.121.161.1

# 2. Kurzname "hund" anlegen
cat >> ~/.ssh/config <<'CONF'

Host hund
  HostName 100.121.161.1
  User fynn
  ServerAliveInterval 15
  ServerAliveCountMax 4
CONF

# 3. Test: muss ohne Passwort "Rasp-Pi-5-Hund" ausgeben
ssh hund hostname
```

Die Doppelklick-Dateien `Go2.command` und `Go2 Stopp.command` liegen auf dem Schreibtisch des MacBooks. Sie rufen `ssh -t hund 'bash ~/unitree_ws/src/go2_bridge/go2.sh oeffnen'` bzw. `… stop` auf. Gibt es den Kurznamen `hund` nicht, nehmen sie direkt `fynn@100.121.161.1`.
