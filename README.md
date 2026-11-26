# Unitree Go2 EDU – DHBW-Projekt Robotertechnik

Autonomer Roboterhund für Innenräume: Der **Unitree Go2 EDU** kartiert einen Raum, läuft selbstständig zu Zielen auf der Karte, setzt sich dort in der gewünschten Blickrichtung hin und prüft mit der Frontkamera, ob ein Fenster offen oder geschlossen ist. Bedient wird er über ein Dashboard im Browser, über Foxglove oder über Befehle im Terminal.

DHBW Stuttgart, Campus Horb · Teamprojekt Robotertechnik · Stand: **26.11.2026**

Diese Datei ist die **einzige Dokumentation des Repos**. Alle früheren READMEs der Unterordner sind hier zusammengeführt.

| | |
|---|---|
| Roboter | Unitree Go2 EDU (L1-4D-LiDAR, Frontkamera, Sport-API) |
| Rechner am Hund | Raspberry Pi 5, 8 GB RAM, Ubuntu 24.04 (aarch64), Hostname `Rasp-Pi-5-Hund`, Benutzer `fynn` |
| Middleware | ROS 2 Jazzy, CycloneDDS (`rmw_cyclonedds_cpp`) über `eth0` |
| Navigation | slam_toolbox (Karte), Nav2 mit AMCL + DWB (Fahren) |
| Bilderkennung | YOLO (Ultralytics), Modell `best.onnx` mit 6 Klassen; zweite Meinung Qwen3-VL 4B lokal über Ollama |
| Bedienung | MacBook über Tailscale/SSH: Dashboard (Browser, Port 8082), Foxglove-App, Terminal (`go2 …`); Unitree-App auf dem iPad als Not-Aus |

**Team**

| | Bereich |
|---|---|
| Student 1 – Fynn Bos | Fundament: Pi, Netzwerk, ROS 2, Hardware-Brücke, Sport-API, `go2`-Skript, Git |
| Student 2 – Jan Bertsch | Bilderkennung: Datensatz, YOLO-Training, Kamera-Stream |
| Student 3 – Martin Gass | Kartierung (SLAM), Nav2, Rucksack für den Pi |
| Student 4 – Maximilian Riesenfeld | Dashboard (HMI), Automatisierung (Mission, Fenster-Scan), Anbindung Qwen |

---

## Inhalt

1. [Was der Hund kann](#1-was-der-hund-kann)
2. [Hardware und Netzwerk](#2-hardware-und-netzwerk)
3. [Software-Architektur](#3-software-architektur)
4. [Aufbau des Repos](#4-aufbau-des-repos)
5. [Einmalige Einrichtung](#5-einmalige-einrichtung)
6. [Betrieb: alles starten](#6-betrieb-alles-starten)
7. [Mission: Ziel anfahren und hinsetzen](#7-mission-ziel-anfahren-und-hinsetzen)
8. [Dashboard (HMI)](#8-dashboard-hmi)
9. [Kamera und Fenstererkennung](#9-kamera-und-fenstererkennung)
10. [Karten (SLAM)](#10-karten-slam)
11. [Nodes und Parameter](#11-nodes-und-parameter)
12. [Nav2-Konfiguration](#12-nav2-konfiguration)
13. [Weitere Ordner: Simulation und erstes Nav2-Paket](#13-weitere-ordner-simulation-und-erstes-nav2-paket)
14. [Git-Workflow](#14-git-workflow)
15. [Fehlerbilder](#15-fehlerbilder)
16. [Projektstand und nächste Schritte](#16-projektstand-und-nächste-schritte)

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
| Hinlegen/Aufstehen über die Sport-API (`--sit-mode standdown`) | ✅ |
| Ziel auf der Karte setzen → hinlaufen → ausrichten → **hinsetzen** → beim nächsten Ziel aufstehen | ✅ getestet am 23.11.2026 |
| Kamera-Livestream mit Tür-/Fenstererkennung im Browser | ✅ |
| Feste Fenster-Punkte speichern und anfahren (`go2 punkt speichern`, `go2 fahre`) | ✅ gebaut 23.11.2026 |
| Kamera-Fenster am Mac: Fenster-Scan startet automatisch, sobald der Hund sitzt | ✅ getestet 25.11.2026 |
| Qwen3-VL als zweite Meinung, wenn YOLO keinen Griff findet | ✅ (Fix leere Antworten 26.11.2026) |
| Dashboard (HMI) mit Live-Daten: Karte, Pfad, Ziele, Startpose, Tastatursteuerung, Not-Aus, Kamera | ✅ getestet 25.11.2026 |
| Dashboard-Simulation ohne Pi und Hund (`--simulation`) | ✅ |
| Hindernisse oberhalb von 40 cm (Tischkanten) | ⚠️ werden vom Scan nicht erfasst |

---

## 2. Hardware und Netzwerk

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
   Dashboard ─┘ (Mac)             │  sit / stand                   ▼
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

Daneben laufen auf dem Pi die Kamera (`go2_tuer_stream.py`, Port 8080, nur Video) und der Dashboard-Server (`go2_dashboard_server.py`, Port 8082). Die Bildauswertung (YOLO, Qwen) läuft auf dem MacBook:

```
Go2-Kamera ─SDK─> go2_tuer_stream.py :8080/roh ─Tailscale─> go2_mac_fenster.py (Mac: YOLO + Qwen3-VL)
                                                                   │ POST :8081/fenster
go2_mission.py :8081 (/state, /punkte, /fahre, /sitz, /steh, /stopp) ◄┘ → Topic /window_status
go2_dashboard_server.py :8082 ─HTTP + SSE 10 Hz─> Browser (Mac)
```

Alle Nodes und Parameter: [Abschnitt 11](#11-nodes-und-parameter).

### 3.1 TF-Baum und Punktwolken

```
TF-Baum:  map ──(AMCL bzw. slam_toolbox)──> odom ──(go2_odom_tf)──> base_link ──(statisch)──> utlidar_lidar
```

**Welche Punktwolke?**

| Topic | Frame im Header | Bewertung |
|---|---|---|
| `/utlidar/cloud` | `utlidar_lidar` | ❌ Header falsch, Daten sind weltausgerichtet (Wände drehen sich in RViz mit dem Hund) |
| `/utlidar/cloud_base` | `base_link` | ✅ nur noch für `accum:=false` |
| `/utlidar/cloud_deskewed` | `odom` | ✅ Quelle für den dichten `/scan` |

---

## 4. Aufbau des Repos

Das Repo liegt auf dem Pi als Workspace `~/unitree_ws` (Repo-Wurzel = Workspace-Wurzel). `build/`, `install/` und `log/` entstehen durch `colcon build` und sind nicht im Git. `go2_bridge` und `yolo_zeugs` sind **keine colcon-Pakete** (kein `package.xml`): Ihre Skripte werden direkt mit `python3` bzw. `ros2 launch` und vollem Pfad gestartet und müssen nicht gebaut werden.

| Pfad | Inhalt | Am echten Hund benutzt? |
|---|---|---|
| `src/go2_bridge/` | Hardware-Brücke, Nav2-Konfiguration, Fahr-Brücke, Mission, Dashboard, `go2`-Skript | ✅ Kern des Projekts |
| `src/go2_bridge/maps/` | SLAM-Karten `raum1`–`raum3` (Abschnitt 10) | ✅ |
| `src/go2_bridge/dashboard/` | Dashboard-Seite (Abschnitt 8) | ✅ |
| `src/yolo_zeugs/` | Bilderkennung, Kamera-Stream, Testbilder, Trainings-Notebook (Abschnitt 9) | ✅ |
| `src/unitree_go2_navigation/` | Erstes Nav2/SLAM-Paket aus der Anfangsphase (Abschnitt 13) | ❌ abgelöst durch `go2_bridge` |
| `src/unitree_go2_ros2_jazzy/` | Fremdprojekt RobInLabUJI: CHAMP-Regler, URDF, Gazebo-Simulation (Abschnitt 13) | ❌ nur Simulation |
| `cyclonedds/` | Git-Submodul: CycloneDDS für das Unitree-SDK | ✅ (Sport-Sender, Kamera) |
| `unitree_sdk2_python/` | Git-Submodul: Unitree SDK2 für Python (`unitree_sdk2py`) | ✅ (Sport-Sender, Kamera) |

### 4.1 `src/go2_bridge/`

| Datei | Typ | Zweck |
|---|---|---|
| `go2.sh` | Shell | **Ein Befehl für alles:** startet, überwacht und beendet alle Programme in einer tmux-Sitzung (Abschnitt 6) |
| `go2_env.sh` | Shell | Umgebung für ROS ↔ Go2. In jedem ROS-Terminal zuerst `source`n (nicht für Sport-Sender und Kamera) |
| `go2_bridge.launch.py` | Launch | Statischer TF `base_link → utlidar_lidar`, `go2_odom_tf.py`, `go2_scan_accum.py` |
| `go2_odom_tf.py` | Node | `/utlidar/robot_odom` → TF `odom → base_link` |
| `go2_scan_accum.py` | Node | `/utlidar/cloud_deskewed` → dichter `/scan` |
| `go2_cmd_vel.py` | Node | ROS-Seite der Fahr-Brücke: `/cmd_vel` → UDP `127.0.0.1:9870` |
| `go2_sport_sender.py` | Skript | SDK-Seite der Fahr-Brücke: UDP → Unitree `SportClient` |
| `go2_mission.py` | Node + HTTP | Mission: Ziel → Nav2 → ausrichten → sitzen; HTTP-Port 8081 |
| `go2_punkte.py` | Skript | Feste Punkte speichern, auflisten, löschen, anfahren |
| `go2_sit_test.py` | Skript | Hinsetzen/Aufstehen von Hand |
| `go2_dashboard_server.py` | Node + Webserver | Dashboard-Server, Port 8082 |
| `dashboard/go2_dashboard.html` | Webseite | Dashboard (Simulation + Live) |
| `make_nav2_params.py` | Skript | Erzeugt `nav2_go2_params.yaml` aus den Nav2-Standardwerten + Go2-Anpassungen |
| `nav2_go2_params.yaml` | YAML | **Erzeugt**, nicht von Hand ändern |
| `go2_nav2.launch.py` | Launch | `nav2_bringup` mit Karte (Standard `maps/raum3.yaml`) |
| `reset_nav2_after_pose.sh` | Shell | Nach dem Setzen der Startpose: Costmaps leeren, bt_navigator prüfen |
| `slam_go2.yaml` | YAML | slam_toolbox-Einstellungen |
| `go2.rviz` | RViz | Gespeicherte Ansicht (nur am Pi-Monitor) |
| `go2_cloud_fix.py` | Node | Früherer Lösungsversuch, **wird nicht mehr benutzt** |
| `Go2_Projektstand_2026-11-02.md` | Doku | Historischer Projektstand vom 02.11.2026 |
| `maps/raum3_punkte.yaml` | YAML | Gespeicherte Punkte, entsteht auf dem Pi beim ersten `go2 punkt speichern` |

### 4.2 `src/yolo_zeugs/`

| Datei | Zweck |
|---|---|
| `go2_tuer_stream.py` | Live-Stream: Go2-Frontkamera → (YOLO) → Browser/Mac (MJPEG über HTTP, Port 8080) |
| `go2_integration.py` | Klasse `GO2InspectionDetector`: wertet ein Bild aus und liefert einen Inspektionsbericht (Fenster- und Türstatus) |
| `detect.py` | Minimalbeispiel: YOLO auf einer USB-/CSI-Kamera (`cv2.VideoCapture(0)`), Anzeige im Fenster, Ende mit `q` |
| `test_headless.py` | Test ohne Bildschirm: wertet `IMG_1944.JPEG` aus, speichert `output_result.jpg` |
| `config.json` | Einstellungen für die Missions-Logik: Modell `best.onnx`, `conf_threshold 0.35`, `required_frames 3`, RTSP-Adresse, Logging |
| `mission_log.txt` | Protokoll der Testläufe vom 24.–25.10.2026 (Erkennung „opened window“, 86,4 %) |
| `IMG_1647.JPEG`, `IMG_1673.JPEG`, `IMG_1685.JPEG`, `IMG_1790.JPEG`, `IMG_1944.JPEG` | Testbilder aus dem Gebäude |
| `output_result.jpg` | Ergebnisbild von `test_headless.py` |
| `Train_YOLO_Models.ipynb` | Colab-Notebook zum Trainieren des Modells (Abschnitt 9.6) |
| `runs/detect/predict/bus.jpg` | Beispielausgabe von Ultralytics (Installationstest) |

**Nicht im Git:** die Modelldateien `best.onnx` und `best.pt` (per `.gitignore` ausgeschlossen). Sie liegen lokal auf dem Pi in diesem Ordner. Auf einen anderen Rechner kopiert man sie mit `scp`, im Mac-Terminal:

```bash
scp fynn@100.121.161.1:~/unitree_ws/src/yolo_zeugs/best.onnx ~/Desktop/
scp ~/Desktop/best.onnx fynn@100.121.161.1:~/unitree_ws/src/yolo_zeugs/
```

### 4.3 Dateien auf dem MacBook (nicht im Repo)

| Datei | Ort | Zweck |
|---|---|---|
| `Go2.command` | Schreibtisch | Verbindet mit dem Pi, startet bei Bedarf alles, zeigt die tmux-Sitzung, öffnet das Kamera-Fenster |
| `Go2 Stopp.command` | Schreibtisch | Beendet alles auf dem Pi und schließt das Kamera-Fenster |
| `Go2 Kamera.command` | Schreibtisch | Startet `go2_mac_fenster.py` (startet es nach einem Absturz neu) |
| `Go2 Dashboard.command` | Schreibtisch bzw. `~/Desktop/go2_bridge` | Öffnet `http://100.121.161.1:8082` |
| `go2_mac_fenster.py` | `~/Downloads` | Kamera-Fenster mit YOLO und Qwen (Abschnitt 9.3) |
| `best.onnx` | `~/Downloads` | YOLO-Modell für den Mac |
| `go2env/` | `~/Downloads` | Python-Umgebung (`onnxruntime`, `opencv-python`, `numpy`) |
| `qwen_beispiele/` | `~/Downloads` | Beispielbilder für Qwen (`offen_*.jpg`, `zu_*.jpg`) |

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

### 5.4 Pi: `go2` und tmux

```bash
chmod +x ~/unitree_ws/src/go2_bridge/go2.sh
mkdir -p ~/.local/bin
ln -sf ~/unitree_ws/src/go2_bridge/go2.sh ~/.local/bin/go2
sudo apt install -y tmux
```

Dazu einmalig `chmod +x ~/unitree_ws/src/go2_bridge/reset_nav2_after_pose.sh` und `python3 ~/unitree_ws/src/go2_bridge/make_nav2_params.py` (Abschnitt 12).

### 5.5 Mac

- **Foxglove-App** von foxglove.dev/download (die Web-Version funktioniert nicht, weil Browser `ws://` von einer `https://`-Seite blockieren).
- **Tailscale** mit demselben Konto wie auf dem Pi.
- **Python-Umgebung für das Kamera-Fenster:**
  ```bash
  python3 -m venv ~/Downloads/go2env
  source ~/Downloads/go2env/bin/activate
  pip install onnxruntime opencv-python numpy
  ```
- **Ollama** (ollama.com) installieren und starten, dann `ollama pull qwen3-vl:4b`.

### 5.6 Mac: SSH-Schlüssel und Kurzname `hund`

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

---

## 6. Betrieb: alles starten

### 6.1 Einfachster Weg: `go2` mit einem Fenster

Alle Programme laufen auf dem Pi in **einer tmux-Sitzung** mit Tabs. Am Mac gibt es dafür nur **ein Fenster**.

- **Mac:** Doppelklick auf `Go2.command` auf dem Schreibtisch. Das verbindet, startet alles (falls nötig) und zeigt die Sitzung. Unten in der Tab-Leiste klickt man zwischen `0:Status`, `1:Bridge`, `2:Sport`, `3:cmd_vel`, `4:Nav2`, `5:Foxglove`, `6:Mission`, `7:Kamera`, `8:Befehle` und `9:Dashboard` hin und her.
- **Mac:** `Go2` öffnet automatisch auch das Kamera-Fenster `Go2 Kamera` (Abschnitt 9).
- **Mac:** Doppelklick auf `Go2 Dashboard.command` öffnet `http://100.121.161.1:8082` im Browser.
- **Mac:** Doppelklick auf `Go2 Stopp.command` beendet alles und schließt das Kamera-Fenster.
- **Pi (z. B. im Tab `8:Befehle`):**

| Befehl | Wirkung |
|---|---|
| `go2 start` | prüft den Ping zum Hund, startet alle 8 Programme in der richtigen Reihenfolge und wartet jeweils auf die Bereit-Meldung |
| `go2 status` | Übersicht mit ✅/❌ (dauerhaft sichtbar im Tab `0:Status`) |
| `go2 stop` | beendet alles in umgekehrter Reihenfolge, hängende Programme nach 15 s erzwungen |
| `go2 restart` | stop + start, z. B. nachdem der Hund aus war |
| `go2 neu mission` | nur ein Programm neu starten (`bridge`, `sport`, `cmd_vel`, `nav2`, `foxglove`, `mission`, `kamera`, `dashboard`) |
| `go2 reset` | nach dem Setzen der Startpose: Costmaps leeren |
| `go2 abbruch` | laufende Fahrt abbrechen |
| `go2 sitz` / `go2 steh` | Hund von Hand hinsetzen / aufstehen lassen |
| `go2 punkt speichern fenster_1` | aktuelle Position + Blickrichtung als Punkt speichern |
| `go2 punkte` | gespeicherte Punkte anzeigen |
| `go2 fahre fenster_1` | Punkt anfahren, ausrichten, hinsetzen, Fenster prüfen |
| `go2 oeffnen` | in die Sitzung zurückkehren |

Schließt man das Mac-Fenster oder bricht Tailscale ab, **läuft auf dem Pi alles weiter**. Mit **Strg+b, dann d** verlässt man die Sitzung absichtlich, ohne etwas zu beenden. Weil die Maus in tmux aktiv ist, markiert man Text im Mac-Terminal mit gedrückter **fn-Taste**.

Einrichtung (einmalig): `go2.sh` liegt in `src/go2_bridge/`. Auf dem Pi `ln -sf ~/unitree_ws/src/go2_bridge/go2.sh ~/.local/bin/go2` und `chmod +x ~/unitree_ws/src/go2_bridge/go2.sh`. Am Mac einen SSH-Schlüssel und den Kurznamen `hund` in `~/.ssh/config` anlegen, siehe Abschnitt 5.4 und 5.6.

**Tabs der Sitzung:**

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
| `9:Dashboard` | `python3 go2_dashboard_server.py` | ja | `Dashboard bereit` |
| `8:Befehle` | freie Shell mit geladener `go2_env.sh` | ja | – |

Die tmux-Sitzung startet mit einer **leeren Umgebung** (`env -i`). So erben Sport-Sender und Kamera keine ROS-Variablen. Beim Beenden bekommt jedes Programm erst Strg+C, nach 15 s ohne Ende wird der Prozessbaum mit `kill -9` beendet.

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
| 9 | Dashboard | `source ~/unitree_ws/src/go2_bridge/go2_env.sh`<br>`python3 ~/unitree_ws/src/go2_bridge/go2_dashboard_server.py` | `Dashboard bereit` (Browser: `http://100.121.161.1:8082`) |

**Beenden:** in umgekehrter Reihenfolge mit **control + C** (9 → 1). Hängt ein Programm, aus einem freien Pi-Fenster zwangsweise beenden, z. B. `pkill -9 -f go2_cmd_vel.py` oder `pkill -9 -f go2_mission.py`.

**Nach dem Ausschalten des Hundes** melden alle ROS-Programme `ddsi_udp_conn_write … failed with retcode -1`. Dann alles beenden, den Hund wieder einschalten, den Ping prüfen und alles neu starten. In jedem Fenster holt die **Pfeil-nach-oben-Taste** den letzten Befehl zurück.

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

## 7. Mission: Ziel anfahren und hinsetzen

Ziele lassen sich auf drei Wegen setzen. Alle laufen über die Mission, der Hund setzt sich also am Ziel hin:

| Weg | Bedienung |
|---|---|
| Dashboard | Rechtsklick auf die Karte → „Ziel hier“, Blickrichtung zeigen, klicken; oder Punkt in der Liste anklicken |
| Terminal | `go2 fahre fenster_1` (gespeicherter Punkt) |
| Foxglove | *Publish pose* auf `/mission_goal` (unten) |

Ein Ziel direkt auf `/goal_pose` umgeht die Mission: Nav2 fährt hin, der Hund setzt sich aber nicht.

### 7.1 Foxglove einrichten (einmalig)

1. Foxglove-App → **Open connection → Foxglove WebSocket → `ws://100.121.161.1:8765`**.
2. Im **3D-Panel** → Zahnrad:
   - **Frame → Display frame:** `map`
   - **Publish → 2D pose estimate → Topic:** `/initialpose`
   - **Publish → 2D pose → Topic:** `/mission_goal` (**nicht** `/goal_pose`, sonst umgeht man die Mission)

### 7.2 Ablauf in Foxglove

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

### 7.3 Zustandsautomat und Schnittstellen (`go2_mission.py`)

```
            neues Ziel                Nav2: SUCCEEDED            |Fehler| < 0,10 rad
  IDLE ───────────────> NAVIGATING ───────────────> ALIGNING ───────────────────> SITTING_DOWN
   ▲                        │ Nav2: Fehler/Abbruch                                     │ ack sit
   └────────────────────────┘                                                          ▼
                                                                                    SITTING
  SETTLING (1 s) <── ack stand ── STANDING_UP <──────────── neues Ziel ─────────────┘
      └────────> NAVIGATING
```

Im Zustand `SITTING` prüft das Kamera-Fenster am Mac das Fenster (YOLO, bei Bedarf Qwen3-VL) und meldet das Ergebnis per `POST /fenster` zurück. `/stopp` beendet `NAVIGATING` und `ALIGNING` sofort, der Hund bleibt stehen (Zustand `IDLE`).

- **Eingang:** `geometry_msgs/PoseStamped` auf `/mission_goal` (Foxglove „Publish pose“, Frame `map`). Position **und** Blickrichtung.
- **Navigation:** Nav2-Action `navigate_to_pose`. Ein neues Ziel ersetzt das laufende sofort.
- **Ausrichten:** Nav2 nimmt die Blickrichtung nur auf ±0,35 rad genau. Danach dreht die Mission den Hund über `/cmd_vel` auf ±0,10 rad (~6°). Nach dreimaligem Überdrehen oder 15 s wird das Ausrichten beendet.
- **Sitzen/Aufstehen:** per UDP an den Sport-Sender, mit Quittung (Timeout 12 s).
- **Fehlschlag:** Erreicht Nav2 das Ziel nicht (`Status 5` = abgebrochen, `Status 6` = gescheitert), bleibt der Hund stehen und die Mission geht auf `IDLE`.
- **Zielname:** Kommt das Ziel über `/fahre` (gespeicherter Punkt), trägt es dessen Namen. Liegt ein Ziel aus Foxglove näher als 0,5 m an einem gespeicherten Punkt, bekommt es ebenfalls dessen Namen, sonst heißt es `Ziel`.

#### Schnittstellen für Kamera-Fenster und Dashboard

| Weg | Inhalt |
|---|---|
| `GET http://100.121.161.1:8081/state` | `{"state": "SITTING", "ziel": "fenster_1", "seit": "10:42:05", "fenster": {...}}` |
| `GET http://100.121.161.1:8081/punkte` | alle gespeicherten Punkte aus `maps/raum3_punkte.yaml` |
| `GET http://100.121.161.1:8081/fahre?name=fenster_1` | gespeicherten Punkt anfahren (wie ein Ziel aus Foxglove) |
| `GET http://100.121.161.1:8081/sitz`, `/steh` | hinsetzen bzw. aufstehen über den Sport-Sender |
| `GET http://100.121.161.1:8081/stopp` | laufende Fahrt und Ausrichtung abbrechen, Hund bleibt stehen (für das Dashboard) |
| `POST http://100.121.161.1:8081/fenster` | Ergebnis der Fensterprüfung vom Mac, z. B. `{"zustand": "ZU", "sicherheit": 0.88}` |
| Topic `/mission_state` (`std_msgs/String`, JSON) | wie `/state`, bei jeder Änderung und einmal pro Sekunde |
| Topic `/window_status` (`std_msgs/String`, JSON) | letztes Ergebnis der Fensterprüfung, z. B. `{"zustand": "OFFEN", "sicherheit": 0.91, "ziel": "fenster_1", "zeit": "10:43:12"}` |

Die HTTP-Antworten erlauben Zugriffe von anderen Seiten (`Access-Control-Allow-Origin: *`), damit das Dashboard sie direkt im Browser abfragen kann.

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
| `http_port` | `8081` | Port für `/state`, `/punkte`, `/fahre`, `/sitz`, `/steh`, `/stopp`, `/fenster` |
| `punkte_datei` | `maps/raum3_punkte.yaml` | Datei mit den gespeicherten Punkten |
| `name_radius` | `0.5` m | Foxglove-Ziel so nah an einem Punkt bekommt dessen Namen |

### 7.4 Feste Punkte (`go2_punkte.py`)

```bash
go2 punkt speichern fenster_1     # Hund steht vor dem Fenster und schaut darauf
go2 punkte                        # Liste
go2 fahre fenster_1               # hinlaufen, ausrichten, hinsetzen, Fenster prüfen
go2 punkt loeschen fenster_1
```

- **Speichern:** Den Hund per Fernbedienung ca. 1–1,5 m vor das Fenster stellen und darauf ausrichten. Bridge und Nav2 müssen laufen und die Startpose muss gesetzt sein. Das Skript liest den TF `map → base_link` und schreibt `x`, `y`, `yaw_deg` und die Uhrzeit nach `maps/raum3_punkte.yaml`.
- **Anfahren:** `go2 fahre` ruft `http://127.0.0.1:8081/fahre?name=…` der Mission auf. Dieselbe Schnittstelle nutzt das Dashboard.
- Mit 20–25 cm Ziel-Toleranz von Nav2 und der Feinausrichtung der Mission (±6°) steht der Hund danach auf etwa 10–20 cm genau am Punkt.

### 7.5 Sitzen, Aufstehen, Hinlegen von Hand

```bash
python3 go2_sit_test.py          # hinsetzen, 5 s warten, aufstehen
python3 go2_sit_test.py sit      # nur hinsetzen
python3 go2_sit_test.py stand    # nur aufstehen
```

Erwartet: `<- OK: Sit() Rückgabe 0` bzw. `<- OK: RiseSit() Rückgabe 0, BalanceStand() Rückgabe 0`. Wer den Hund von Hand hinsetzt, muss ihn vor dem nächsten Missionsziel auch von Hand aufstehen lassen.

**Hinlegen:** Einen eigenen Befehl gibt es nicht. Entweder in der Unitree-App, oder den Sport-Sender im Hinlege-Modus starten (nur wenn der Hund steht, siehe 11.6):

```bash
python3 ~/unitree_ws/src/go2_bridge/go2_sport_sender.py --sit-mode standdown
go2 sitz     # legt sich hin
go2 steh     # steht wieder auf
```

Solange dieser Modus läuft, legt sich der Hund auch am Ende einer Mission hin. Zurück: `go2 neu sport`.

### 7.6 `reset_nav2_after_pose.sh`

Nach jeder neuen Startpose in Foxglove ausführen. Das Skript leert `/local_costmap` und `/global_costmap` (Geisterwände von der falschen Pose verschwinden) und prüft den Zustand des `bt_navigator`: Ist er `inactive`, wird er fortgesetzt, ist er `unconfigured`, wird Nav2 gestartet.

---

## 8. Dashboard (HMI)

Eine Webseite für alles: Karte mit Hund, Ziele, Steuern, Kamera und Zustand. Läuft im Browser am Mac, Foxglove bleibt parallel nutzbar.

- **Öffnen:** `http://100.121.161.1:8082` (startet mit `go2 start` im Tab `9:Dashboard`).
- **Dateien:** `src/go2_bridge/go2_dashboard_server.py` (Server auf dem Pi) und `src/go2_bridge/dashboard/go2_dashboard.html` (Seite). Die Seite ist die weiterentwickelte Demo `go2_dashboard_demo.html`.
- **Zwei Modi** (oben „Verbindung“ anklicken):
  - **Simulation:** die Demo im Browser (Labor, Klassenzimmer, simulierter Hund). Braucht weder Pi noch Hund.
  - **Live:** echte Karte `raum3`, Position und Blickrichtung des Hundes (TF `map → base_link`), Nav2-Pfad, Lidar-Punkte, gespeicherte Fenster-Punkte mit Ergebnis (grün = zu, rot = offen), Kamerabild, Pi-CPU/RAM/Temperatur/Drosselung, Programmstatus.
- **Bedienung live:** Rechtsklick auf die Karte → „Ziel hier“ (dann Blickrichtung zeigen und klicken), „Hund steht hier (Startpose)“, „Mein Standort hier“ (für „Call“); Punkte in der Liste anklicken = anfahren; „Punkt +“ speichert die aktuelle Position; W/S und Q/E bzw. Pfeiltasten steuern (max. 0,35 m/s, Totmann 0,5 s, bricht eine laufende Fahrt ab); Leertaste = NOT-AUS, Esc = Fahrt abbrechen; Aufstehen/Sitz.
- **Ohne Hund testen:** `python3 src/go2_bridge/go2_dashboard_server.py --simulation` → `http://localhost:8082`. Der Server spielt einen Hund auf der echten Karte `raum3` vor, die Seite läuft dabei im Live-Modus.
- **Sicherheit:** Totmann 0,5 s (ohne Tastendruck Stillstand), Steuern bricht eine laufende Fahrt ab (Mission `/stopp` + Nav2-Abbruch), im Sitzen keine Fahrbefehle, NOT-AUS sendet dauerhaft Stillstand, bis „FREIGEBEN“ gedrückt wird.
- **Last auf dem Pi:** Der Server nutzt nur die Python-Standardbibliothek und `rclpy`. Karte, Plan und Lidar gehen nur bei Änderungen an den Browser.

### 8.1 Aufbau

```
Browser (Mac) ──HTTP + Server-Sent-Events, Port 8082──► go2_dashboard_server.py ──ROS 2──► Nav2, Mission, Hund
              ◄──Kamerabild direkt, Port 8080 (go2_tuer_stream.py /roh)
```

Der Browser spricht kein ROS. Der Server sammelt alles und schickt dem Browser 10-mal pro Sekunde den Zustand als JSON. Große Teile (Karte, Plan, Lidar, Punkte) gehen nur, wenn sie sich geändert haben. Die Karte wird einmal über `/api/map` geladen.

**Liest:** `/map` (transient local), TF `map → base_link` (Fallback `odom → base_link`, dann orange Hund-Symbol), `/plan`, `/scan`, `/amcl_pose` (Unsicherheit ±m), `/navigate_to_pose/_action/feedback` (Restweg, ETA, Recoveries), Mission `GET :8081/state`, Punkte `maps/raum3_punkte.yaml`, Kamera `GET :8080/status`, `/proc` (CPU, RAM, laufende Programme), Temperatur, `vcgencmd get_throttled`.

**Schreibt:** `/mission_goal` (Ziel), `/initialpose` (Startpose), `/cmd_vel` (**nur** beim Steuern im Dashboard und beim Not-Aus), Nav2-Abbruch (`/navigate_to_pose/_action/cancel_goal`, leere Anfrage = alle Ziele), Mission `/sitz`, `/steh`, `/stopp`, `/fahre`, `{"stop": true}` an den Sport-Sender.

### 8.2 Sicherheit

| Fall | Verhalten |
|---|---|
| Steuern mit WASD | Browser sendet 10×/s. Kommt 0,5 s nichts mehr (Taste los, Tab gewechselt, WLAN weg), schickt der Server 3× Stillstand und gibt `/cmd_vel` wieder frei. |
| Steuern während einer Fahrt | Fahrt wird abgebrochen (Mission `/stopp` + Nav2-Abbruch), dann gilt die Tastatur. |
| Hund sitzt | Fahrbefehle werden abgelehnt („erst Aufstehen“). |
| NOT-AUS (Leertaste/Knopf) | Fahrt abbrechen, StopMove, solange aktiv 10×/s Stillstand auf `/cmd_vel`, keine Ziele. „FREIGEBEN“ hebt ihn auf. |
| Grenzen | vx −0,15 … 0,35 m/s, Drehung ±0,40 rad/s (wie `go2_cmd_vel.py`); kein Seitwärtslaufen. |

### 8.3 HTTP-Schnittstelle

| Methode + Pfad | Inhalt |
|---|---|
| `GET /` | Dashboard-Seite |
| `GET /api/stream` | Server-Sent-Events, Zustand 10×/s |
| `GET /api/state` | Zustand einmalig (zum Testen: `curl http://100.121.161.1:8082/api/state`) |
| `GET /api/map` | Karte: Breite, Höhe, Auflösung, Ursprung, Daten (base64, 0 frei, 100 Wand, 255 unbekannt) |
| `POST /api/ziel` | `{"x", "y", "yaw"}` im Frame `map` |
| `POST /api/fahre` | `{"name": "fenster_1"}` |
| `POST /api/fahren` | `{"vx", "wz"}` Tastatursteuerung |
| `POST /api/abbruch` | Fahrt abbrechen |
| `POST /api/notaus` | `{"an": true}` / `{"an": false}` |
| `POST /api/sitz`, `/api/steh` | über die Mission |
| `POST /api/punkt` | `{"name"}` aktuelle Position + Blickrichtung speichern |
| `POST /api/punkt_loeschen` | `{"name"}` |
| `POST /api/startpose` | `{"x", "y", "yaw"}` → `/initialpose` |

### 8.4 Bedienung

| Aktion | Eingabe |
|---|---|
| Ziel setzen | Rechtsklick auf die Karte → „Ziel hier“, Blickrichtung zeigen, klicken |
| Startpose setzen | Rechtsklick → „Hund steht hier (Startpose)“, Blickrichtung zeigen, klicken, danach `go2 reset` |
| Gespeicherten Punkt anfahren | Punkt in der Liste anklicken |
| Punkt speichern | „Punkt +“ (aktuelle Position + Blickrichtung) |
| Fahren | W/S bzw. Pfeil hoch/runter vor/zurück, Q/E bzw. Pfeil links/rechts drehen (kein Seitwärtslaufen) |
| NOT-AUS / Fahrt abbrechen | Leertaste / Esc |
| Sitzen / Aufstehen | Knöpfe im Bereich „Befehle“ |
| Simulation ↔ Live | oben „Verbindung“ |

**Behoben am 25.11.:** Der Nav2-Pfad wurde bei unveränderter Plan-ID nicht neu an den Browser geschickt. Der Server bildet die ID jetzt aus Plan und Fahrzustand, der Pfad erscheint bei jeder Fahrt.

**Seite ohne Server öffnen:** `dashboard/go2_dashboard.html` per Doppelklick öffnet die reine Simulation. Mit `?pi=100.121.161.1` hinten an der Adresse verbindet sie sich live mit dem Pi.

### 8.5 Ohne Pi und Hund testen

 `python3 go2_dashboard_server.py --simulation` (braucht kein ROS) startet einen simulierten Hund auf `maps/raum3.yaml` mit simulierter Mission (Hinlaufen, Ausrichten, Sitzen, Fenster-Ergebnis). Dann `http://localhost:8082`.

---

## 9. Kamera und Fenstererkennung

**Ablauf im Betrieb:**

```
go2 fahre fenster_1 ──> Mission: aufstehen → hinlaufen → ausrichten → SITZEN
                                                                  │
Mac-Fenster „Go2 Kamera“ (immer offen): Video ── sieht SITTING ──> YOLO an (nur Fenster)
          Kopfzeile: „FENSTER: ZU“ (grün) bzw. „FENSTER: OFFEN“ (rot), wenn 3× in Folge gleich
          YOLO unklar (Rahmen ohne Griff) ──> Qwen3-VL 4B (Ollama, lokal) mit dem Fensterausschnitt
          Ergebnis zurück an den Pi → Topic /window_status (für das Dashboard)
nächstes Ziel ──> Mission: aufstehen ──> Mac: KI aus, nur Video
```

- **Pi:** Tab `7:Kamera` sendet nur Video (`go2_tuer_stream.py auto --ohne-ki`). Immer `auto` verwenden, nicht `eth0`.
- **Mac:** `Go2 Kamera` öffnet sich automatisch mit `Go2` (`~/Downloads/go2_mac_fenster.py`, KI auf dem M3). Für Qwen muss die Ollama-App laufen (`ollama pull qwen3-vl:4b`, einmalig).
- **Bildrate:** Mit `--ohne-ki` reicht der Pi das JPEG der Hundekamera unverändert durch. Im Test am 26.11. kamen am Mac 5,2 Bilder/s an, YOLO schafft auf der Mac-CPU etwa 1,3 Bilder/s bei 1024 px.
- **Fenster-Punkte anlegen:** Hund per Fernbedienung ca. 1–1,5 m vor das Fenster stellen und darauf ausrichten, dann `go2 punkt speichern fenster_1`. Anfahren mit `go2 fahre fenster_1`, Liste mit `go2 punkte`.
- Ohne Mac geht auch die Variante mit KI auf dem Pi (`python3 go2_tuer_stream.py auto`, Browser `http://100.121.161.1:8080`). Sie ist allerdings deutlich langsamer.

### 9.1 Modell `best.onnx` – 6 Klassen

| ID | Klasse | Bedeutung |
|---|---|---|
| 0 | `closed door` | Tür geschlossen |
| 1 | `door handle` | Türgriff |
| 2 | `opened door` | Tür geöffnet |
| 3 | `window` | Fensterrahmen |
| 4 | `window handle horizontal` | Fenstergriff waagerecht → Fenster **offen / gekippt** |
| 5 | `window handle vertical` | Fenstergriff senkrecht → Fenster **geschlossen** |

Annahme für Dreh-Kipp-Fenster: senkrechter Griff = zu, waagerechter Griff = offen. Ein nach oben zeigender Griff (gekippt) sieht für YOLO aus wie „senkrecht“.

Klassen eines Modells prüfen:

```bash
python3 -c "import onnxruntime as o; print(o.InferenceSession('best.onnx').get_modelmeta().custom_metadata_map.get('names'))"
```

Ein älteres Modell `best.pt` hatte 4 Klassen (`closed door`, `closed window`, `opened door`, `opened window`). `go2_tuer_stream.py` deutet beide Varianten: Klassennamen mit `door` werden als Tür gewertet, solche mit `window` als Fenster, ein Name mit `open…` am Anfang als offen.

### 9.2 Kamera-Stream auf dem Pi: `go2_tuer_stream.py`

Läuft auf dem Pi, holt die Bilder über das Unitree SDK2 (`VideoClient`) per LAN-Kabel und stellt sie als Webseite bereit.

```bash
python3 go2_tuer_stream.py auto
```

Browser am Mac: **http://100.121.161.1:8080**

| Option | Standard | Bedeutung |
|---|---|---|
| `auto` / `eth0` | `eth0` | Netzwerk-Interface zum Go2. **Immer `auto` verwenden**, mit `eth0` stürzt das SDK ab („buffer overflow“). |
| `--model` | `best.pt` | Modelldatei, aktuell `--model best.onnx` angeben |
| `--conf` | `0.35` | Mindest-Konfidenz |
| `--frames` | `3` | Ein Objekt muss so viele Bilder hintereinander erkannt werden, bevor ein Rahmen erscheint |
| `--imgsz` | `640` | Bildgröße für YOLO, kleiner (`320`, `416`) = schneller |
| `--port` | `8080` | HTTP-Port |
| `--test` | aus | USB-Kamera am Pi statt Hundekamera |

Beispiel für den Pi: `python3 go2_tuer_stream.py auto --model best.onnx --imgsz 320`

**Erweiterte Fassung (Stand 02.11.2026):** Darin laufen Video und KI getrennt. Das Video läuft mit voller Kamerarate, die KI-Rahmen werden auf jedes Bild gezeichnet, oben steht „Video xx FPS | KI x/s“. Dazu kommen:

| Zusatz | Bedeutung |
|---|---|
| `--ohne-ki` | nur Video senden, keine Erkennung auf dem Pi |
| `/stream` | Videostream mit Rahmen |
| `/roh` | Videostream ohne Rahmen, z. B. für eine Erkennung auf dem MacBook. Mit `--ohne-ki` wird das Original-JPEG der Hundekamera unverändert durchgereicht (kein Dekodieren auf dem Pi) |
| `/roh?w=640&q=60` | wie `/roh`, aber auf 640 px Breite verkleinert und mit JPEG-Qualität 60 (für langsame Verbindungen über Tailscale) |
| `/status` | JSON: Bilder pro Sekunde, Bildgröße, Zuschauer, Fehlerzähler (für das Dashboard) |

Mit `--ohne-ki` reicht der Stream das JPEG der Hundekamera unverändert durch (kein Entpacken und Neupacken auf dem Pi). Nur `?w=` kleiner als 1280 rechnet auf dem Pi. Alle 5 s steht im Tab `7:Kamera`, wie viele Bilder pro Sekunde wirklich ankommen. „Code 3104“ heißt: Zeitüberschreitung, der Videodienst im Hund antwortet nicht (nicht das LAN-Kabel).

Mit der erweiterten Fassung: `python3 go2_tuer_stream.py auto --model best.onnx --frames 2`

- Rahmenfarben: **hellgrün** = geschlossen, **rot** = offen. Oben im Bild stehen FPS und die Anzahl offener/geschlossener Objekte.
- Auf der Pi-CPU braucht `best.onnx` bei voller Größe mehrere Sekunden pro Bild. Mit `--imgsz 320` wird es schneller, aber ungenauer.
- `Address already in use`: Ein alter Stream läuft noch → `pkill -f go2_tuer_stream.py`.
- Die Unitree-App stört den Stream nicht. Der Go2 erlaubt aber nur eine WebRTC-Verbindung gleichzeitig.

### 9.3 Kamera-Fenster am Mac: `go2_mac_fenster.py`

Stand 26.11.2026. Die Datei liegt am Mac unter `~/Downloads/go2_mac_fenster.py`. Das Fenster ist immer offen. Es zeigt das Video vom Pi (`/roh`) und fragt alle 0,5 s den Zustand der Mission ab (`http://100.121.161.1:8081/state`).

| Mission | Anzeige im Fenster | KI |
|---|---|---|
| läuft, richtet sich aus, steht auf | „Hund laeuft zu fenster_1“ + „KI aus – startet beim Hinsetzen“ | aus |
| `SITTING` | „Hund SITZT an fenster_1 – Fenster-Scan aktiv“ + Ergebnis groß | YOLO auf dem jeweils neuesten Bild, nur die Klassen `window`, `window handle horizontal`, `window handle vertical` |

**YOLO:** ONNX Runtime auf der Mac-CPU, Letterbox auf die Eingangsgröße des Modells (1024 × 1024), NMS mit IoU 0,45. Schwellen: Fensterrahmen 0,35, Griffe 0,15 (im Sitzen werden Griffe schräg von unten gesehen und schwächer erkannt).

**Ergebnis:** Hauptfenster = größter Fensterrahmen. Liegt ein Griff darin (+15 % Rand), entscheidet der Griff: senkrecht = **ZU** (grün), waagerecht = **OFFEN** (rot). Ein Rahmen ohne Griff ergibt **UNKLAR** (orange), nichts erkannt **KEIN FENSTER IM BILD**. Gekippte Fenster gibt es in den Projekträumen nicht.

**Stabilisierung:** Ein Ergebnis gilt erst, wenn 3 Auswertungen in Folge gleich sind (keine älter als 8 s). Vorher steht dort „pruefe …“. Jedes neue stabile Ergebnis wird per `POST /fenster` an den Pi gemeldet und erscheint dort auf dem Topic `/window_status` und im Dashboard.

**Zweite Meinung mit Qwen3-VL:** Ist YOLO unklar oder nach 4 s noch ohne Ergebnis, schickt das Programm den Fensterausschnitt (+15 % Rand, höchstens 640 px) an Qwen3-VL 4B über Ollama (`http://localhost:11434`, lokal, ohne Internet). Frage: „open or closed?“, Temperatur 0, höchstens alle 6 s. Die Antwort gilt 15 s und wird als „FENSTER: ZU (Qwen)“ angezeigt und mit `quelle: mac-qwen` gemeldet. Ein eindeutiges YOLO-Ergebnis hat Vorrang.

- `num_predict` steht auf 400. Qwen3-VL schreibt vor der Antwort einen Denktext, mit dem früheren Limit von 24 Tokens blieb die Antwort leer (Fußzeile `Qwen: ''`, Anzeige „Qwen prueft …“ ohne Ende). Bleibt `response` trotzdem leer, wertet das Programm das letzte „open“ bzw. „closed“ im Feld `thinking` aus.
- Zeitlimit pro Anfrage 60 s. Die erste Anfrage dauert länger, weil Ollama das Modell lädt (`keep_alive 30m` hält es danach im Speicher).

**Beispielbilder für Qwen (Few-Shot):** Bilder in `~/Downloads/qwen_beispiele/` mit den Namen `offen_*.jpg` bzw. `zu_*.jpg` (auch `.png`, kein `.heic`) werden jeder Frage als Beispiele mit der richtigen Antwort vorangestellt, je Zustand die 3 neuesten. Im Kamera-Fenster speichert `o` den aktuellen Fensterausschnitt als Beispiel „offen“, `z` als „zu“; das gilt sofort. Jedes Beispiel verlängert die Antwortzeit. Das Programm probiert nacheinander: Chat mit Beispielen, Chat vereinfacht, alle Bilder in einer Frage, ohne Beispiele. Die erste Form, die Ollama annimmt, wird gemerkt (Terminal: `Qwen-Anfrage funktioniert mit: …`). Antwortet Qwen nur ohne Beispiele, steht `[ohne Bsp]` hinter der Antwort.

**Fußzeile:** `Video x FPS | YOLO x FPS (CPU) | Qwen: 'closed' in x s | Bild 1920x1080 Q75`. Im Test am 26.11.: Video 5,2 FPS, YOLO 1,3 FPS.

**Start:** Doppelklick auf `Go2 Kamera` auf dem Mac-Schreibtisch, oder automatisch mit `Go2`. Von Hand:

```bash
source ~/Downloads/go2env/bin/activate
python ~/Downloads/go2_mac_fenster.py
python ~/Downloads/go2_mac_fenster.py --immer-ki      # Test: KI auch, wenn der Hund nicht sitzt
```

| Option | Standard | Bedeutung |
|---|---|---|
| `--pi` | `100.121.161.1` | Tailscale-Adresse des Pi |
| `--model` | `~/Downloads/best.onnx` | YOLO-Modell |
| `--conf` | `0.35` | Schwelle für Fensterrahmen |
| `--conf-griff` | `0.15` | Schwelle für Griffe |
| `--stabil` | `3` | gleiche Auswertungen in Folge, bis ein Ergebnis gilt |
| `--immer-ki` | aus | KI unabhängig vom Zustand (zum Testen) |
| `--stufe` | `1` | Bildqualität vom Pi: 0 = volle Größe, 1 = 1280 px, 2 = 960 px, 3 = 640 px |
| `--coreml` | aus | CoreML statt CPU für YOLO versuchen |
| `--vlm` | `qwen3-vl:4b` | Ollama-Modell für die zweite Meinung |
| `--ohne-vlm` | aus | Qwen abschalten |
| `--vlm-pause` | `6.0` s | Abstand zwischen zwei Qwen-Fragen |

Tasten: `q` beendet, `s` speichert das Rohbild nach `~/Downloads` (zum Nachtrainieren), `k` schaltet die KI von Hand an/aus, `o`/`z` speichert ein Beispiel für Qwen, `-`/`+` Bild kleiner/größer (Stufe 0/1 = Original 1920 px unverändert durchgereicht, 2 = 960 px, 3 = 640 px; ab Stufe 2 rechnet der Pi).

Voraussetzungen am Mac: Python-Umgebung `~/Downloads/go2env` mit `onnxruntime`, `opencv-python`, `numpy`; Ollama-App mit `ollama pull qwen3-vl:4b`.

### 9.4 Inspektion eines Bildes: `go2_integration.py`

```bash
python3 go2_integration.py IMG_1944.JPEG best.onnx
```

Ausgabe: ein Bericht mit Fenster- und Türstatus. Das Bild mit Rahmen wird in `output/annotated_IMG_1944.JPEG` gespeichert.

**Logik:**

- **Fenster:** Das größte erkannte Fenster ist das Hauptfenster. Liegt der Mittelpunkt eines Griffs innerhalb des Fensters (+15 % Rand), gehört der Griff dazu. Ein waagerechter Griff ergibt `OFFEN / GEKIPPT`, ein senkrechter `GESCHLOSSEN`, ohne Griff `UNKLAR`. Gibt es genau ein Fenster und einen Griff, werden beide verknüpft. Wird ein Griff ohne Fensterrahmen erkannt, entscheidet allein der Griff.
- **Tür:** Die größte erkannte Tür im Vordergrund entscheidet zwischen `OFFEN` und `GESCHLOSSEN`.
- **Kein Objekt** ergibt `NICHT_VORHANDEN` (Negativbeispiel).

In eigenem Code:

```python
from go2_integration import GO2InspectionDetector, print_inspection_report

detector = GO2InspectionDetector(model_path="best.onnx", conf_threshold=0.20, imgsz=1024)
report = detector.analyze_image("IMG_1944.JPEG", save_annotated=True)
print(report["window_status"]["state"])   # z. B. "GESCHLOSSEN"
print_inspection_report(report)
```

### 9.5 Kurztests

```bash
python3 test_headless.py      # erkennt Objekte in IMG_1944.JPEG, speichert output_result.jpg
python3 detect.py             # nur mit Bildschirm und Kamera am Pi, Ende mit q
```

`detect.py` lädt `best.pt`. Fehlt diese Datei oder ist sie beschädigt (`invalid load key, '\x0d'`), zum Testen in Zeile 5 `YOLO('best.onnx')` eintragen.

### 9.6 Modell trainieren

Das Notebook `src/yolo_zeugs/Train_YOLO_Models.ipynb` (Google Colab, Vorlage von EJ Technology Consultants) trainiert YOLO11/YOLOv8 mit eigenen Bildern:

1. Bilder aus dem Gebäude aufnehmen (empfohlen 200–500, Griffe aus vielen Blickwinkeln) und in Roboflow mit den 6 Klassen labeln.
2. Datensatz im Notebook hochladen, in Train/Validation aufteilen, trainieren.
3. `best.pt` herunterladen und als ONNX exportieren:
   ```bash
   yolo export model=best.pt format=onnx
   ```
4. `best.onnx` per `scp` nach `~/unitree_ws/src/yolo_zeugs/` auf den Pi kopieren.

Roboflow exportiert die Bilder mit 640 × 640 px. Trainiert wurde im Notebook mit `imgsz=1024` (`epochs=120`), Ultralytics skaliert die Bilder dafür beim Laden. Das Modell erwartet deshalb 1024 × 1024 als Eingang.

### 9.7 Abhängigkeiten auf dem Pi

```bash
sudo apt install -y python3-pip python3-opencv ffmpeg libgstreamer1.0-0 v4l-utils
pip3 install --user --break-system-packages ultralytics onnxruntime
```

Für die Go2-Kamera zusätzlich das Python-SDK `unitree_sdk2py` mit `cyclonedds==0.10.2`:

```bash
export CYCLONEDDS_HOME=~/unitree_ws/cyclonedds/install
pip3 install --user --break-system-packages cyclonedds==0.10.2
```

Getestete Versionen auf dem Pi: Ultralytics 8.4.175, PyTorch 2.14.1, OpenCV 5.0.0, NumPy 2.5.3, Python 3.12.3. Gerechnet wird nur auf der CPU, eine GPU gibt es nicht.

---

## 10. Karten (SLAM)

### 10.1 Vorhandene Karten (`src/go2_bridge/maps/`)

Alle Karten wurden mit `slam_toolbox` (`slam_go2.yaml`) und dem dichten Scan aus `go2_scan_accum.py` aufgenommen. Auflösung **0,05 m pro Pixel**.

| Karte | Dateien | Ursprung (`origin`) | Verwendung |
|---|---|---|---|
| `raum1` | `raum1.pgm`, `raum1.yaml`, `raum1_slam.data`, `raum1_slam.posegraph` | `[-8.398, -4.061, 0]` | Erste Karte vom 02.11.2026: Klassenzimmer, ca. 9 × 8 m, vier Tischreihen als Punktreihen |
| `raum2` | `raum2.pgm`, `raum2.yaml`, `raum2_slam.data`, `raum2_slam.posegraph` | `[-1.961, -9.816, 0]` | Zweite Aufnahme |
| `raum3` | `raum3.pgm`, `raum3.yaml` | `[-8.398, -4.061, 0]` | **Standardkarte für Nav2** und Dashboard |

| Datei | Inhalt | Wofür |
|---|---|---|
| `*.pgm` | Belegungsbild (weiß = frei, schwarz = belegt, grau = unbekannt) | Nav2-Map-Server |
| `*.yaml` | Bild, Auflösung, Ursprung, Schwellwerte (`occupied_thresh 0.65`, `free_thresh 0.196`, `mode: trinary`) | Nav2-Map-Server |
| `*_slam.data`, `*_slam.posegraph` | Serialisierter Posen-Graph von slam_toolbox | Weiterkartieren oder Lokalisieren mit slam_toolbox |

Andere Karte für Nav2: `ros2 launch ~/unitree_ws/src/go2_bridge/go2_nav2.launch.py map:=$HOME/unitree_ws/src/go2_bridge/maps/raum1.yaml` (ohne `map:=` wird `raum3` geladen). Das Dashboard liest immer die Karte, die Nav2 auf `/map` veröffentlicht.

### 10.2 Neue Karte aufnehmen

Bridge und Foxglove-Bridge wie in Abschnitt 6.2 (Fenster 1 und 5), die Bridge aber mit `accum_window:=0.6`, dazu SLAM:

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

**Karte auf den Mac holen** (Mac-Terminal, Anführungszeichen wegen `*`): `scp "fynn@100.121.161.1:~/unitree_ws/src/go2_bridge/maps/raum3.*" ~/Desktop/`

**Hinweise:** Eine Karte wie ein „ausgeleerter Farbeimer“ (strahlenförmige weiße Fläche, kaum Wände) entsteht ohne `go2_scan_accum.py` oder bei zu schnellem Drehen. Der Scan erfasst nur Hindernisse 10–40 cm über dem Boden, Tische erscheinen deshalb nur als Punktreihen (Beine).

---

## 11. Nodes und Parameter

### 11.1 `go2_env.sh`

```bash
source /opt/ros/jazzy/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
# eigenes CycloneDDS aus ~/unitree_ws ausblenden (sonst Absturz "buffer overflow" mit eth0)
export LD_LIBRARY_PATH=$(echo "$LD_LIBRARY_PATH" | tr ':' '\n' | grep -v unitree_ws | paste -sd:)
export CYCLONEDDS_URI='<CycloneDDS><Domain><General><Interfaces><NetworkInterface name="eth0" priority="default" multicast="default"/></Interfaces></General></Domain></CycloneDDS>'
```

**Nicht** für `go2_sport_sender.py` verwenden. Das Unitree-SDK bringt sein eigenes CycloneDDS mit und verträgt die ROS-Einstellungen nicht.

### 11.2 `go2_odom_tf.py`

| Parameter | Standard | Bedeutung |
|---|---|---|
| `odom_topic` | `/utlidar/robot_odom` | Quelle |
| `odom_frame` / `base_frame` | `odom` / `base_link` | Frames des TF |
| `use_pi_time` | `false` (Launch: `true`) | Zeitstempel in Pi-Zeit verschieben |
| `offset_alpha` | `0.02` | Glättungsfaktor (EMA) für den Uhren-Offset |

Bei `use_pi_time:=true` wird der **Hardware-Zeitstempel behalten** und nur um einen geglätteten Offset `Pi-Uhr − Go2-Uhr` verschoben. Springt der Offset um mehr als 0,5 s (Hund neu gestartet), wird er neu gesetzt. Früher wurde jeder Stempel durch die Ankunftszeit ersetzt. Dadurch passte beim Drehen die Odometrie nicht mehr zu den Punktwolken, und die Karte verrutschte.

### 11.3 `go2_scan_accum.py`

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

### 11.4 `go2_bridge.launch.py`

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

### 11.5 `go2_cmd_vel.py` (ROS-Seite der Fahr-Brücke)

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

### 11.6 `go2_sport_sender.py` (SDK-Seite der Fahr-Brücke)

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
- **Hinlegen statt Sitzen:** siehe 7.5.
- **Nicht neu starten, während der Hund sitzt:** Das `BalanceStand()` beim Start lässt ihn aufstehen.

### 11.7 RViz (nur am Pi-Monitor)

```bash
source ~/unitree_ws/src/go2_bridge/go2_env.sh
rviz2 -d ~/unitree_ws/src/go2_bridge/go2.rviz
```

Bei LaserScan und PointCloud2 `Reliability Policy = Best Effort`, bei `/map` Durability `Transient Local`. Über SSH ist die Foxglove-App die bessere Wahl.

---

## 12. Nav2-Konfiguration

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

## 13. Weitere Ordner: Simulation und erstes Nav2-Paket

### 13.1 `src/unitree_go2_navigation/` (abgelöst)

ROS-2-Paket (ament_cmake) aus der Anfangsphase, angelegt am 25.10.2026 von Student 3. Am echten Hund wird es **nicht mehr verwendet**, es bleibt als Referenz im Repo.

| Datei | Inhalt |
|---|---|
| `config/nav2_params.yaml` | MPPI-Controller, NavFn-Planner, AMCL mit `base_footprint`, `odom_topic: /odom`, `robot_radius: 0.22` |
| `config/mapper_params_online_async.yaml` | slam_toolbox-Standardwerte mit `base_frame: base_footprint` |
| `scripts/simple_commander.py` | Schickt den Roboter über `nav2_simple_commander` zu (2,0 / 1,0) im Frame `map` |
| `CMakeLists.txt`, `package.xml` | Paketdefinition |

| Einstellung dort | Am echten Go2 nötig | Umgesetzt in |
|---|---|---|
| `base_footprint` | `base_link` | `make_nav2_params.py`, `slam_go2.yaml` |
| `odom_topic: /odom` | `/utlidar/robot_odom` | `make_nav2_params.py` |
| MPPI-Controller | DWB (leichter für den Pi 5) | `make_nav2_params.py` |
| `robot_radius: 0.22` | rechteckiger Footprint 0,64 × 0,30 m | `make_nav2_params.py` |
| `/scan` und `/tf` vorausgesetzt | werden von der Brücke erzeugt | `go2_odom_tf.py`, `go2_scan_accum.py` |
| Nav2 publiziert `/cmd_vel` | der Go2 hört nur auf die Sport-API | `go2_cmd_vel.py` + `go2_sport_sender.py` |

`simple_commander.py` bricht beim Start ab, weil die letzte Zeile `if _name_ == '_main_':` lautet (je ein statt zwei Unterstriche). Richtig: `if __name__ == '__main__':`.

### 13.2 `src/unitree_go2_ros2_jazzy/` (nur Simulation)

Unveränderte Kopie des Fremdprojekts [RobInLabUJI/unitree_go2_ros2_jazzy](https://github.com/RobInLabUJI/unitree_go2_ros2_jazzy): CHAMP-Regler, URDF des Go2, Gazebo-Harmonic-Welten. Die englische Originalbeschreibung steht im Original-Repo.

- `unitree_go2_sim/launch/unitree_go2_launch.py` und `unitree_go2_launch_TI.py` starten einen **virtuellen** Go2 in Gazebo. Einen Treiber für den echten Hund enthält das Paket nicht.
- Am echten Go2 EDU kommen die Daten direkt per DDS über das LAN-Kabel, TF, `/scan`, Nav2 und Fahrbefehle laufen über `go2_bridge`.
- Nützlich bleibt das URDF (`unitree_go2_description/urdf/`) zur Visualisierung. Die Einbaulage des L1-LiDAR in `go2_bridge.launch.py` stammt aus Unitrees Paket `go2_description` (`radar_joint`).
- Das Paket wird beim `colcon build` mitgebaut. Für die Simulation sind zusätzlich Gazebo Harmonic und die CHAMP-Abhängigkeiten nötig (siehe Original-Repo). Die Lizenzdateien (`LICENSE`) der Fremdpakete bleiben im Ordner.

---

## 14. Git-Workflow

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
- Dokumentation gibt es nur in dieser `README.md`. Neue Unterordner bekommen keine eigene README, sondern einen Abschnitt hier.

---

## 15. Fehlerbilder

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
| Kamera: `Code 3104` | Zeitüberschreitung des Videodienstes im Hund, kein Kabelproblem. Kameraansicht der Unitree-App schließen, `go2 neu kamera`, sonst Hund neu starten. |
| Kamera-Fenster: „Qwen prueft …“ ohne Ende, Fußzeile `Qwen: ''` | Qwen3-VL gibt zuerst Denktext aus, das alte Token-Limit (24) reichte nicht für die Antwort. Behoben am 26.11. (`num_predict 400`, Auswertung des Denktexts). Fenster mit `q` schließen und neu öffnen. |
| Kamera-Fenster: `Ollama nicht erreichbar` | Ollama-App am Mac starten. Steht `(HTTPError)` dahinter, läuft Ollama, lehnt die Anfrage aber ab: aktuelle `go2_mac_fenster.py` verwenden, sie probiert andere Anfrageformen und zeigt `Ollama lehnt ab: …`. |
| Dashboard zeigt keine Daten | Tab `9:Dashboard` läuft nicht (`go2 neu dashboard`) oder die Seite steht auf „Simulation“ (oben „Verbindung“ → Live). |
| In RViz wird nichts angezeigt | Bei LaserScan/PointCloud2 `Reliability Policy = Best Effort`, bei `/map` Durability `Transient Local`. |
| `Control loop missed its desired rate of 10.0000 Hz` | Der Pi schafft knapp 10 Hz. Unkritisch, Abhilfe: `controller_frequency` auf 8. |
| `Failed to get parameters: 'dock_database'` / `'FootprintApproach.max_points'` | Foxglove fragt ungenutzte Parameter ab. Harmlos. |
| Mission: `Nav2 hat das Ziel nicht erreicht (Status 5)` | Ziel wurde abgebrochen (z. B. per `cancel_goal`). |
| Mission: `Nav2 hat das Ziel nicht erreicht (Status 6)` | Nav2 ist gescheitert (kein Fortschritt, keine Route). |
| Sport-Sender: `Aktion 'stand': FEHLER (RiseSit() Rückgabe -1 …)` | Alte Version: Der Hund stand schon. Seit 23.11. wird das als Erfolg gewertet. |
| Programm reagiert nicht auf control + C | Aus einem freien Fenster zwangsweise beenden, z. B. `pkill -9 -f go2_cmd_vel.py`. |
| `Address already in use` (Kamera) | Alter Stream läuft noch: `pkill -f go2_tuer_stream.py`. |

---

## 16. Projektstand und nächste Schritte

**Erreicht (Stand 26.11.2026):** Netzwerk und Fernzugriff, Hardware-Brücke, Karten `raum1`–`raum3`, Nav2 am echten Hund, Rückwärts-Recovery, Missions-Ablauf „Ziel → hinlaufen → ausrichten → sitzen → Fenster prüfen“, feste Fenster-Punkte, Kamera-Stream (Original-JPEG durchgereicht), Fenster-Scan am Mac mit YOLO und Qwen3-VL, Dashboard live und als Simulation.

**Nächste Schritte:**

1. Lokalisierung robuster machen: Startpose sorgfältig setzen, bei Bedarf `xy_goal_tolerance` auf 0,40 m erhöhen.
2. Kollisionsschutz: Höhenband des Scans auf ~0,60 m erweitern, Höchstgeschwindigkeit auf 0,25 m/s senken, Collision Monitor mit kleiner Zone wieder aktivieren.
3. `controller_frequency` von 10 auf 8 Hz senken (der Pi schafft knapp 9,6 Hz).
4. Fenster-Punkte für alle Fenster im Raum anlegen und den Blickwinkel im Sitzen prüfen.
5. YOLO-Modell mit Bildern aus Sitzhöhe nachtrainieren (Griffe werden schräg von unten oft nicht erkannt, dann entscheidet Qwen).
6. Kamera: Code 3104 (Zeitüberschreitung des Videodienstes im Hund) tritt auf, wenn die Kameraansicht der Unitree-App gleichzeitig offen ist. App-Kamera schließen, Hund ggf. neu starten.
7. Für die Vorführung: `go2 start` per systemd automatisch beim Hochfahren des Pi ausführen.
8. Hindernissperre beim manuellen Fahren im Dashboard (Vorwärts sperren, wenn der Scan vor dem Hund < 0,4 m meldet). Aktuell gibt es beim manuellen Fahren keinen Kollisionsschutz.
9. Roboflow-API-Schlüssel aus `Train_YOLO_Models.ipynb` entfernen und in Roboflow neu erzeugen (das Repo ist öffentlich).
10. `go2_mac_fenster.py` und die `.command`-Dateien ins Repo aufnehmen (z. B. `src/mac/`).
