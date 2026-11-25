# Unitree Go2 EDU – DHBW-Projekt Robotertechnik

Autonomer Roboterhund für Innenräume: Der **Unitree Go2 EDU** kartiert einen Raum, läuft selbstständig zu Zielen, die auf der Karte gesetzt werden, setzt sich dort in die gewünschte Blickrichtung hin und erkennt mit der Frontkamera, ob Fenster offen oder geschlossen sind.

DHBW Stuttgart, Campus Horb · Teamprojekt Robotertechnik · Stand: **23.11.2026**

| | |
|---|---|
| Roboter | Unitree Go2 EDU (L1-4D-LiDAR, Frontkamera, Sport-API) |
| Rechner am Hund | Raspberry Pi 5, 8 GB RAM, Ubuntu 24.04 (aarch64), Hostname `Rasp-Pi-5-Hund`, Benutzer `fynn` |
| Middleware | ROS 2 Jazzy, CycloneDDS (`rmw_cyclonedds_cpp`) über `eth0` |
| Navigation | slam_toolbox (Karte), Nav2 mit AMCL + DWB (Fahren) |
| Bilderkennung | YOLO (Ultralytics), Modell `best.onnx` mit 6 Klassen |
| Bedienung | MacBook über Tailscale/SSH, Foxglove-App, Unitree-App auf dem iPad als Not-Aus |

---

## Inhalt

1. [Was der Hund kann](#1-was-der-hund-kann)
2. [Aufbau: Hardware und Netzwerk](#2-aufbau-hardware-und-netzwerk)
3. [Software-Architektur](#3-software-architektur)
4. [Ordnerstruktur des Repos](#4-ordnerstruktur-des-repos)
5. [Einmalige Einrichtung](#5-einmalige-einrichtung)
6. [Betrieb: alles starten](#6-betrieb-alles-starten)
7. [Ziel setzen: hinlaufen und hinsetzen](#7-ziel-setzen-hinlaufen-und-hinsetzen)
8. [Karte aufnehmen (SLAM)](#8-karte-aufnehmen-slam)
9. [Kamera und Fenstererkennung](#9-kamera-und-fenstererkennung)
10. [Dashboard (HMI)](#10-dashboard-hmi)
11. [Git-Workflow](#11-git-workflow)
12. [Bekannte Probleme](#12-bekannte-probleme)
13. [Projektstand und nächste Schritte](#13-projektstand-und-nächste-schritte)

---

## 1. Was der Hund kann

| Funktion | Status |
|---|---|
| Pi ↔ Go2 per LAN-Kabel (statische IP `192.168.123.43`), Ping ~0,2 ms | ✅ |
| Fernzugriff auf den Pi über Tailscale (`100.121.161.1`) | ✅ |
| Go2-Topics direkt in ROS 2 (DDS, ohne Treiber) | ✅ |
| TF `odom → base_link` und dichter `/scan` aus dem L1-LiDAR | ✅ |
| SLAM-Karten mit slam_toolbox (`raum1`, `raum2`, `raum3`) | ✅ |
| Nav2 steuert den echten Hund (`/cmd_vel` → Unitree Sport-API) | ✅ |
| Rückwärts-Recovery („BackUp“) bei Wandkontakt | ✅ |
| Ziel auf der Karte setzen → hinlaufen → ausrichten → **hinsetzen** → beim nächsten Ziel aufstehen | ✅ getestet am 23.11.2026 |
| Kamera-Livestream mit Tür-/Fenstererkennung im Browser | ✅ |
| Feste Fenster-Punkte speichern und anfahren (`go2 punkt speichern`, `go2 fahre`) | ✅ gebaut 23.11.2026 |
| Kamera-Fenster am Mac: Fenster-Scan startet automatisch, sobald der Hund sitzt | ✅ gebaut 23.11.2026 |
| Dashboard-Demo (simuliert) für Mac, iPad, iPhone | ✅ |
| Dashboard mit Live-Daten vom Hund | 🔜 geplant |
| Hindernisse oberhalb von 40 cm (Tischkanten) | ⚠️ werden vom Scan nicht erfasst |

---

## 2. Aufbau: Hardware und Netzwerk

```
MacBook ──(Tailscale-VPN)──> Raspberry Pi 5 ──(LAN-Kabel, eth0)──> Go2 EDU
                              100.121.161.1                         192.168.123.161
                              eth0: 192.168.123.43/24
iPad (Unitree-App) ──(WLAN/Bluetooth)──> Go2   ← nur zum Aufstehen/Not-Aus
```

- **SSH vom Mac:** `ssh fynn@100.121.161.1` (Tailscale muss auf Mac und Pi laufen).
- **Pi sieht den Hund?** `ping -c 3 192.168.123.161`
- **Dateien Mac → Pi** (im Mac-Terminal): `scp ~/Desktop/go2_bridge/go2_mission.py fynn@100.121.161.1:~/unitree_ws/src/go2_bridge/`
- **Dateien Pi → Mac** (im Mac-Terminal): `scp "fynn@100.121.161.1:~/unitree_ws/src/go2_bridge/maps/raum3.*" ~/Desktop/`
- Steht vorne in der Eingabezeile `maximilian@MacBook…`, ist man **am Mac**. Steht dort `fynn@Rasp-Pi-5-Hund`, ist man **auf dem Pi**. `ros2`-Befehle gehen nur auf dem Pi.

### Statische IP am Pi (einmalig, NetworkManager)

Die von netplan erzeugte Verbindung `netplan-eth0` blieb trotz `ipv4.method manual` auf DHCP und lief ins Timeout. Deshalb gibt es eine eigene Verbindung `go2-lan`:

```bash
sudo nmcli connection add type ethernet ifname eth0 con-name go2-lan \
  ipv4.method manual ipv4.addresses 192.168.123.43/24 ipv6.method disabled
sudo nmcli connection modify "netplan-eth0" connection.autoconnect no
sudo nmcli connection up go2-lan
ping -c 4 192.168.123.161
```

Die Verbindung startet automatisch und übersteht Neustarts. Die SSH-Verbindung läuft über Tailscale bzw. WLAN und bleibt beim Umstellen von `eth0` bestehen.

---

## 3. Software-Architektur

```
                      Go2 EDU (DDS über eth0)
   /utlidar/robot_odom ───────┐        /utlidar/cloud_deskewed
                              ▼                    ▼
                     go2_odom_tf.py        go2_scan_accum.py
                   TF odom → base_link     /scan (LaserScan, 360°, 8 Hz)
                              │                    │
                              ▼                    ▼
          ┌────────────── Nav2 (AMCL, Planner, DWB, Behaviors) ──────────────┐
          │      Karte maps/raum3.yaml · Parameter nav2_go2_params.yaml       │
          └──────────────────────────────┬──────────────────────────────────┘
   Foxglove ─/mission_goal─> go2_mission.py ──NavigateToPose──┘   │ /cmd_vel
   (Mac)                          │  sit / stand                   ▼
                                  │                         go2_cmd_vel.py
                                  │                               │ UDP localhost:9870
                                  └─────────── UDP ──────────────▶ go2_sport_sender.py
                                                                  │ Unitree SDK2 (SportClient)
                                                                  ▼
                                                     Move · StopMove · Sit · RiseSit · BalanceStand
```

Wichtige Erkenntnisse, auf denen der Aufbau beruht:

- **Der Go2 EDU braucht keinen ROS-Treiber.** Er sendet seine Topics (`/utlidar/*`, `/sportmodestate`, `/lowstate` …) direkt per DDS über das LAN-Kabel. Es gibt aber **kein `/tf`** und **kein `/scan`**, beides liefert `go2_bridge`.
- **Der Go2 hört nicht auf `/cmd_vel`.** Fahrbefehle gehen nur über die Unitree Sport-API.
- **ROS 2 und das Unitree-SDK lassen sich nicht im selben Prozess starten** (zwei CycloneDDS-Instanzen, „create domain error“). Deshalb gibt es die Zwei-Prozess-Brücke `go2_cmd_vel.py` (ROS) ↔ UDP ↔ `go2_sport_sender.py` (SDK).
- **Die Uhr des Hundes weicht von der Pi-Uhr ab** (teils über eine Minute). Die Zeitstempel werden mit einem geglätteten Uhren-Offset in die Pi-Zeit verschoben (`use_pi_time:=true`), ohne den Abstand zwischen Odometrie und Punktwolken zu verändern.
- Das Repo `unitree_go2_ros2_jazzy` (CHAMP/Gazebo) ist **nur Simulation** und wird am echten Hund nicht verwendet.

Ausführliche Beschreibung aller Nodes und Parameter: [`src/go2_bridge/README.md`](src/go2_bridge/README.md)

---

## 4. Ordnerstruktur des Repos

Das Repo liegt auf dem Pi als Workspace in `~/unitree_ws` (Repo-Wurzel = Workspace-Wurzel). `build/`, `install/` und `log/` entstehen durch `colcon build` und sind nicht im Git.

| Pfad | Inhalt | Wird am echten Hund benutzt? |
|---|---|---|
| [`src/go2_bridge/`](src/go2_bridge/README.md) | Hardware-Brücke (TF, `/scan`), Nav2-Konfiguration, `/cmd_vel`-Brücke, Missions-Steuerung, Karten | ✅ Kern des Projekts |
| [`src/go2_bridge/maps/`](src/go2_bridge/maps/README.md) | Gespeicherte SLAM-Karten | ✅ |
| [`src/yolo_zeugs/`](src/yolo_zeugs/README.md) | Bilderkennung Türen/Fenster, Kamera-Stream, Testbilder | ✅ |
| [`src/unitree_go2_navigation/`](src/unitree_go2_navigation/README.md) | Erstes Nav2/SLAM-Paket (Konfiguration aus der Simulationsphase) und `simple_commander.py` | ❌ abgelöst durch `go2_bridge` |
| [`src/unitree_go2_ros2_jazzy/`](src/unitree_go2_ros2_jazzy/README.md) | Fremdprojekt RobInLabUJI: CHAMP-Regler, URDF, Gazebo-Simulation | ❌ nur Simulation |
| `src/Ablauf Projekt` | Ursprüngliche Projektplanung (Schritte, Werkzeuge) | Doku |
| `src/projekt_statusbericht_fynn.md` | Statusbericht Einrichtung Pi, Workspace, YOLO, Netzwerk | Doku |
| `Train_YOLO_Models.ipynb` | Google-Colab-Notebook zum Trainieren des YOLO-Modells | Training |

`go2_bridge` und `yolo_zeugs` sind **keine colcon-Pakete** (kein `package.xml`). Ihre Skripte werden direkt mit `python3` bzw. `ros2 launch` und vollem Dateipfad gestartet und müssen nicht gebaut werden.

---

## 5. Einmalige Einrichtung

### 5.1 Pi: ROS-Pakete

```bash
sudo apt update
sudo apt install -y ros-jazzy-rmw-cyclonedds-cpp ros-jazzy-pointcloud-to-laserscan \
  ros-jazzy-slam-toolbox ros-jazzy-navigation2 ros-jazzy-nav2-bringup \
  ros-jazzy-foxglove-bridge ros-jazzy-rviz2 ros-jazzy-teleop-twist-keyboard
```

### 5.2 Pi: Python und Unitree-SDK

```bash
sudo apt install -y python3-pip python3-opencv ffmpeg libgstreamer1.0-0 v4l-utils
pip3 install --user --break-system-packages ultralytics onnxruntime "setuptools<80"
export CYCLONEDDS_HOME=~/unitree_ws/cyclonedds/install
pip3 install --user --break-system-packages cyclonedds==0.10.2
echo 'export PATH="$HOME/.local/bin:$PATH"' >> ~/.bashrc
```

`setuptools` muss unter Version 80 bleiben, sonst bricht `colcon build` ab. Das Python-SDK `unitree_sdk2py` (Unitree SDK2 für Python) muss importierbar sein. Es wird von `go2_sport_sender.py` und vom Kamera-Stream genutzt.

### 5.3 Pi: Workspace bauen

```bash
cd ~/unitree_ws && git pull
colcon build
source install/setup.bash
```

### 5.4 Mac

- **Foxglove-App** von foxglove.dev/download (die Web-Version funktioniert nicht, weil Browser `ws://` von einer `https://`-Seite blockieren).
- **Tailscale** mit demselben Konto wie auf dem Pi.

---

## 6. Betrieb: alles starten

### 6.1 Einfachster Weg: `go2` mit einem Fenster

Alle Programme laufen auf dem Pi in **einer tmux-Sitzung** mit Tabs. Am Mac gibt es dafür nur **ein Fenster**.

- **Mac:** Doppelklick auf `Go2.command` auf dem Schreibtisch. Das verbindet, startet alles (falls nötig) und zeigt die Sitzung. Unten in der Tab-Leiste klickt man zwischen `0:Status`, `1:Bridge`, `2:Sport`, `3:cmd_vel`, `4:Nav2`, `5:Foxglove`, `6:Mission`, `7:Kamera` und `8:Befehle` hin und her.
- **Mac:** `Go2` öffnet automatisch auch das Kamera-Fenster `Go2 Kamera` (Abschnitt 9).
- **Mac:** Doppelklick auf `Go2 Stopp.command` beendet alles und schließt das Kamera-Fenster.
- **Pi (z. B. im Tab `8:Befehle`):**

| Befehl | Wirkung |
|---|---|
| `go2 start` | prüft den Ping zum Hund, startet alle 7 Programme in der richtigen Reihenfolge und wartet jeweils auf die Bereit-Meldung |
| `go2 status` | Übersicht mit ✅/❌ (dauerhaft sichtbar im Tab `0:Status`) |
| `go2 stop` | beendet alles in umgekehrter Reihenfolge, hängende Programme nach 15 s erzwungen |
| `go2 restart` | stop + start, z. B. nachdem der Hund aus war |
| `go2 neu mission` | nur ein Programm neu starten (`bridge`, `sport`, `cmd_vel`, `nav2`, `foxglove`, `mission`, `kamera`) |
| `go2 reset` | nach dem Setzen der Startpose: Costmaps leeren |
| `go2 abbruch` | laufende Fahrt abbrechen |
| `go2 sitz` / `go2 steh` | Hund von Hand hinsetzen / aufstehen lassen |
| `go2 punkt speichern fenster_1` | aktuelle Position + Blickrichtung als Punkt speichern |
| `go2 punkte` | gespeicherte Punkte anzeigen |
| `go2 fahre fenster_1` | Punkt anfahren, ausrichten, hinsetzen, Fenster prüfen |
| `go2 oeffnen` | in die Sitzung zurückkehren |

Schließt man das Mac-Fenster oder bricht Tailscale ab, **läuft auf dem Pi alles weiter**. Mit **Strg+b, dann d** verlässt man die Sitzung absichtlich, ohne etwas zu beenden. Weil die Maus in tmux aktiv ist, markiert man Text im Mac-Terminal mit gedrückter **fn-Taste**.

Einrichtung (einmalig): `go2.sh` liegt in `src/go2_bridge/`. Auf dem Pi `ln -sf ~/unitree_ws/src/go2_bridge/go2.sh ~/.local/bin/go2` und `chmod +x ~/unitree_ws/src/go2_bridge/go2.sh`. Am Mac einen SSH-Schlüssel und den Kurznamen `hund` in `~/.ssh/config` anlegen, siehe [`src/go2_bridge/README.md`](src/go2_bridge/README.md#9-go2-ein-fenster-statt-vieler).

### 6.2 Von Hand (ohne `go2`)

Jedes Programm läuft in einem **eigenen Terminal-Fenster am Mac**, das per `ssh fynn@100.121.161.1` mit dem Pi verbunden ist. Reihenfolge einhalten.

**Vorher:** Hund einschalten, warten bis er steht, dann `ping -c 3 192.168.123.161`.

| Nr. | Fenster | Befehl | Fertig, wenn … |
|---|---|---|---|
| 1 | Bridge (TF + `/scan`) | `source ~/unitree_ws/src/go2_bridge/go2_env.sh`<br>`ros2 launch ~/unitree_ws/src/go2_bridge/go2_bridge.launch.py accum_window:=0.25` | `Erste Odometrie empfangen` und z. B. `/scan: 104/360 Strahlen mit Treffer` (gut: ab ~90) |
| 2 | Sport-Sender | `python3 ~/unitree_ws/src/go2_bridge/go2_sport_sender.py` (**ohne** `go2_env.sh`!) | `SportClient bereit` und `Sitz-Modus 'sit'` |
| 3 | cmd_vel-Brücke | `source ~/unitree_ws/src/go2_bridge/go2_env.sh`<br>`python3 ~/unitree_ws/src/go2_bridge/go2_cmd_vel.py` | `Drehung 0.3..0.4 rad/s` in der Startmeldung |
| 4 | Nav2 | `source ~/unitree_ws/src/go2_bridge/go2_env.sh`<br>`ros2 launch ~/unitree_ws/src/go2_bridge/go2_nav2.launch.py` | `Managed nodes are active` |
| 5 | Foxglove-Bridge | `source ~/unitree_ws/src/go2_bridge/go2_env.sh`<br>`ros2 launch foxglove_bridge foxglove_bridge_launch.xml` | `Server listening on port 8765` |
| 6 | Mission | `source ~/unitree_ws/src/go2_bridge/go2_env.sh`<br>`python3 ~/unitree_ws/src/go2_bridge/go2_mission.py` | `Bereit. Ziele auf /mission_goal` |
| 7 | Kamera (nur Video, KI auf dem Mac) | `cd ~/unitree_ws/src/yolo_zeugs && python3 go2_tuer_stream.py auto --ohne-ki` (**ohne** `go2_env.sh`!) | Startmeldung mit Port `8080` |
| 8 | frei | für Reset-Skript, Abbruch, Sitz-Test, `git` | – |

**Beenden:** in umgekehrter Reihenfolge mit **control + C** (7 → 1). Hängt ein Programm, aus einem freien Pi-Fenster zwangsweise beenden, z. B. `pkill -9 -f go2_cmd_vel.py` oder `pkill -9 -f go2_mission.py`.

**Nach dem Ausschalten des Hundes** melden alle ROS-Programme `ddsi_udp_conn_write … failed with retcode -1`. Dann alles beenden, den Hund wieder einschalten, den Ping prüfen und alles neu starten. In jedem Fenster holt die **Pfeil-nach-oben-Taste** den letzten Befehl zurück.

---

## 7. Ziel setzen: hinlaufen und hinsetzen

### 7.1 Foxglove einrichten (einmalig)

1. Foxglove-App → **Open connection → Foxglove WebSocket → `ws://100.121.161.1:8765`**.
2. Im **3D-Panel** → Zahnrad:
   - **Frame → Display frame:** `map`
   - **Publish → 2D pose estimate → Topic:** `/initialpose`
   - **Publish → 2D pose → Topic:** `/mission_goal` (**nicht** `/goal_pose`, sonst umgeht man die Mission)

### 7.2 Ablauf

1. **Startpose setzen:** Werkzeug *Publish pose estimate*. Auf die echte Position des Hundes klicken und in seine Blickrichtung ziehen. Danach müssen die lila Scan-Punkte auf den schwarzen Wänden der Karte liegen.
2. Im freien Fenster: `~/unitree_ws/src/go2_bridge/reset_nav2_after_pose.sh` (leert die Costmaps, prüft den bt_navigator).
3. **Ziel malen:** Werkzeug *Publish pose*. Auf den Zielpunkt klicken und in die gewünschte Blickrichtung ziehen.
4. Der Hund läuft hin, dreht sich auf ±6° in die gezogene Richtung und **setzt sich**. Beim nächsten Ziel steht er auf und läuft weiter.

Im Mission-Fenster sieht ein erfolgreicher Durchlauf so aus:

```
Neues Ziel: x=0.00 y=-0.96 Blick 28° (Frame map)
[IDLE] -> [NAVIGATING]
Ziel erreicht.
[NAVIGATING] -> [ALIGNING]
Ausgerichtet (Restfehler -6°). Setze mich hin.
[SITTING_DOWN] -> [SITTING]
```

**Fahrt abbrechen** (freies Pi-Fenster):

```bash
source ~/unitree_ws/src/go2_bridge/go2_env.sh
ros2 service call /navigate_to_pose/_action/cancel_goal action_msgs/srv/CancelGoal "{}"
```

**Sitzen/Aufstehen von Hand** (Sport-Sender muss laufen):

```bash
python3 ~/unitree_ws/src/go2_bridge/go2_sit_test.py sit     # hinsetzen
python3 ~/unitree_ws/src/go2_bridge/go2_sit_test.py stand   # aufstehen
```

---

## 8. Karte aufnehmen (SLAM)

Fenster 1 (Bridge) und 5 (Foxglove-Bridge) wie oben, dazu:

```bash
source ~/unitree_ws/src/go2_bridge/go2_env.sh
ros2 launch ~/unitree_ws/src/go2_bridge/go2_bridge.launch.py accum_window:=0.6   # statt 0.25: dichterer Scan
ros2 launch slam_toolbox online_async_launch.py slam_params_file:=$HOME/unitree_ws/src/go2_bridge/slam_go2.yaml
```

In Foxglove `/map` und `/scan` einblenden. Den Hund per Fernbedienung **langsam** fahren und **besonders langsam drehen**: erst an den Wänden entlang, zurück zum Start (Schleife schließen), dann durch die Gänge. Speichern, während SLAM noch läuft:

```bash
ros2 run nav2_map_server map_saver_cli -f ~/unitree_ws/src/go2_bridge/maps/raum4
ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph \
  "{filename: '$HOME/unitree_ws/src/go2_bridge/maps/raum4_slam'}"
```

Eine neue Karte für Nav2 nutzen: `ros2 launch ~/unitree_ws/src/go2_bridge/go2_nav2.launch.py map:=$HOME/unitree_ws/src/go2_bridge/maps/raum4.yaml`

---

## 9. Kamera und Fenstererkennung

**Ablauf im Betrieb:**

```
go2 fahre fenster_1 ──> Mission: aufstehen → hinlaufen → ausrichten → SITZEN
                                                                  │
Mac-Fenster „Go2 Kamera“ (immer offen): Video ── sieht SITTING ──> YOLO an (nur Fenster)
          Kopfzeile: „FENSTER: ZU“ (grün) bzw. „FENSTER: OFFEN“ (rot), wenn 2 s stabil
          Ergebnis zurück an den Pi → Topic /window_status (für das Dashboard)
nächstes Ziel ──> Mission: aufstehen ──> Mac: KI aus, nur Video
```

- **Pi:** Tab `7:Kamera` sendet nur Video (`go2_tuer_stream.py auto --ohne-ki`). Immer `auto` verwenden, nicht `eth0`.
- **Mac:** `Go2 Kamera` öffnet sich automatisch mit `Go2` (`~/Downloads/go2_mac_fenster.py`, KI auf dem M3).
- **Fenster-Punkte anlegen:** Hund per Fernbedienung ca. 1–1,5 m vor das Fenster stellen und darauf ausrichten, dann `go2 punkt speichern fenster_1`. Anfahren mit `go2 fahre fenster_1`, Liste mit `go2 punkte`.
- Ohne Mac geht auch die Variante mit KI auf dem Pi (`python3 go2_tuer_stream.py auto`, Browser `http://100.121.161.1:8080`). Sie ist allerdings deutlich langsamer.

Einzelheiten, Klassen und Modelle: [`src/yolo_zeugs/README.md`](src/yolo_zeugs/README.md)

---

## 10. Dashboard (HMI)

Eine Webseite für alles: Karte mit Hund, Ziele, Steuern, Kamera und Zustand. Läuft im Browser am Mac, Foxglove bleibt parallel nutzbar.

- **Öffnen:** `http://100.121.161.1:8082` (startet mit `go2 start` im Tab `9:Dashboard`).
- **Dateien:** `src/go2_bridge/go2_dashboard_server.py` (Server auf dem Pi) und `src/go2_bridge/dashboard/go2_dashboard.html` (Seite). Die Seite ist die weiterentwickelte Demo `go2_dashboard_demo.html`.
- **Zwei Modi** (oben „Verbindung“ anklicken):
  - **Simulation:** die Demo im Browser (Labor, Klassenzimmer, simulierter Hund). Braucht weder Pi noch Hund.
  - **Live:** echte Karte `raum3`, Position und Blickrichtung des Hundes (TF `map → base_link`), Nav2-Pfad, Lidar-Punkte, gespeicherte Fenster-Punkte mit Ergebnis (grün = zu, rot = offen), Kamerabild, Pi-CPU/RAM/Temperatur/Drosselung, Programmstatus.
- **Bedienung live:** Rechtsklick auf die Karte → „Ziel hier“ (dann Blickrichtung zeigen und klicken), „Hund steht hier (Startpose)“, „Mein Standort hier“ (für „Call“); Punkte in der Liste anklicken = anfahren; „Punkt +“ speichert die aktuelle Position; WASD/Q/E steuert (max. 0,35 m/s, Totmann 0,5 s, bricht eine laufende Fahrt ab); Leertaste = NOT-AUS, Esc = Fahrt abbrechen; Aufstehen/Sitz.
- **Ohne Hund testen:** `python3 src/go2_bridge/go2_dashboard_server.py --simulation` → `http://localhost:8082`. Der Server spielt einen Hund auf der echten Karte `raum3` vor, die Seite läuft dabei im Live-Modus.

Details und Schnittstelle: [`src/go2_bridge/README.md`](src/go2_bridge/README.md), Abschnitt 10.

---

## 11. Git-Workflow

Auf dem Pi ist das Repo der Workspace `~/unitree_ws`:

```bash
cd ~/unitree_ws
git pull                       # vor dem Arbeiten
git status                     # was hat sich geändert?
git add README.md .gitignore src/go2_bridge src/yolo_zeugs src/unitree_go2_navigation
git commit -m "Mission: Ziel anlaufen, ausrichten, hinsetzen"
git push
```

- **Nicht** `git add -A` verwenden: Im Workspace liegen ungetrackte Ordner wie `~/unitree_ws/cyclonedds` und die Build-Ordner, die nicht ins Repo gehören.
- Modelldateien (`*.onnx`, `*.pt`) sind per `.gitignore` ausgeschlossen. Sie werden mit `scp` kopiert.
- Divergierende Branches lösen, wenn die lokalen Änderungen verworfen werden dürfen:
  ```bash
  git merge --abort
  git fetch && git reset --hard origin/main
  ```
- `colcon build` nur in `~/unitree_ws` ausführen, nicht in `src/`.

---

## 12. Bekannte Probleme

| Problem | Ursache und Lösung |
|---|---|
| Hund dreht sich minutenlang auf der Stelle, `Failed to make progress` | Startpose stimmt nicht, Scan und Karte passen nicht übereinander. Fahrt abbrechen, Pose estimate neu setzen (lila Punkte auf schwarzen Wänden), Reset-Skript ausführen, erst nahe Ziele (1–2 m) testen. |
| Hund steht am Ziel, setzt sich aber nicht | Nav2 meldet das Ziel nicht als erreicht, weil die Lokalisierung versetzt ist. Hinsetzen passiert nur nach `Ziel erreicht`. |
| Hund dreht nicht, obwohl `z: 0.22` gesendet wird | Der Go2 reagiert erst ab etwa 0,30 rad/s. `go2_cmd_vel.py` hebt Drehungen auf der Stelle deshalb auf `min_yaw = 0.30` an. |
| Hund läuft gegen Tischkanten | Der Scan erfasst nur 10–40 cm über dem Boden (`min_height`/`max_height`). Hindernisse darüber sind unsichtbar. |
| Mission lehnt Ziele ab (`Aufstehen fehlgeschlagen … bleibe sitzen`) | Der Hund wurde von Hand aufgestanden, während die Mission „sitzt“ dachte. Seit 23.11. wertet der Sport-Sender `RiseSit() = -1` bei stehendem Hund als Erfolg. Sonst die Mission neu starten. |
| Hund steht von selbst auf | Der Sport-Sender wurde neu gestartet, während der Hund saß (beim Start sendet er `BalanceStand`). |
| `ddsi_udp_conn_write … failed with retcode -1` | Keine LAN-Verbindung zum Hund (aus oder Kabel ab). Alles neu starten, siehe Abschnitt 6. |
| `*** buffer overflow detected ***` | CycloneDDS aus `~/unitree_ws/install` mit `eth0`. `go2_env.sh` blendet es aus. Beim Kamera-Stream `auto` statt `eth0`. |
| Foxglove: `Connection failed` | Foxglove-Bridge (Fenster 5) läuft nicht. |
| Foxglove: Ziel geht direkt an Nav2, Hund setzt sich nie | 2D-pose-Topic steht noch auf `/goal_pose`. Auf `/mission_goal` umstellen. |
| `zsh: command not found: ros2` | Befehl im Mac-Fenster statt auf dem Pi eingegeben. |
| `best.pt is not a loadable checkpoint … invalid load key '\x0d'` | Modelldatei beschädigt (von Git als Text behandelt) oder nur ein Platzhalter. Modell per `scp` direkt auf den Pi kopieren. |
| In RViz wird nichts angezeigt | Bei LaserScan/PointCloud2 `Reliability Policy = Best Effort`, bei `/map` Durability `Transient Local`. |

---

## 13. Projektstand und nächste Schritte

**Erreicht (Stand 23.11.2026):** Netzwerk und Fernzugriff, Hardware-Brücke, Karten `raum1`–`raum3`, Nav2 am echten Hund, Rückwärts-Recovery, Missions-Ablauf „Ziel → hinlaufen → ausrichten → sitzen“, Kamera-Stream mit Erkennung, Dashboard-Demo.

**Nächste Schritte:**

1. Lokalisierung robuster machen: Startpose sorgfältig setzen, bei Bedarf `xy_goal_tolerance` auf 0,40 m erhöhen.
2. Kollisionsschutz: Höhenband des Scans auf ~0,60 m erweitern, Höchstgeschwindigkeit auf 0,25 m/s senken, Collision Monitor mit kleiner Zone wieder aktivieren.
3. `controller_frequency` von 10 auf 8 Hz senken (der Pi schafft knapp 9,6 Hz).
4. Fenster-Punkte für alle Fenster im Raum anlegen und den Blickwinkel im Sitzen prüfen.
5. Dashboard-Live-Version am Hund testen (Verzögerung beim Steuern, Kamera, Startpose über das Dashboard).
6. Kamera: Zeitüberschreitungen des Videodienstes im Hund (Code 3104) beheben, dann Fenster-Scan am echten Fenster prüfen.
7. Für die Vorführung: `go2 start` per systemd automatisch beim Hochfahren des Pi ausführen.
