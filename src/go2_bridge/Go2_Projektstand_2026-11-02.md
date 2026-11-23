# Unitree Go2 EDU – Projektstand & Anleitung (Stand 02.11.2026)

DHBW-Projekt Robotertechnik · Go2 EDU + Raspberry Pi 5 · Ubuntu 24.04 · ROS 2 Jazzy

Dieses Dokument fasst zusammen, was seit dem 26.10.2026 eingerichtet und getestet wurde. Wer das Projekt übernimmt, sollte damit ohne Vorwissen weitermachen können.

---

## 1. Überblick: Was funktioniert jetzt

| Thema | Status |
|---|---|
| Pi ↔ Go2 per LAN (`eth0`), Ping ~0,18 ms | ✅ |
| Fernzugriff auf den Pi über Tailscale (`100.121.161.1`) | ✅ |
| Go2-Topics in ROS 2 sichtbar (DDS direkt, kein Treiber) | ✅ |
| TF `odom → base_link` (aus Go2-Odometrie) | ✅ getestet (Drehen/Laufen) |
| `/scan` (LaserScan) für SLAM/Nav2 – **dicht, aus gesammelten Wolken** (`go2_scan_accum.py`) | ✅ |
| RViz-Konfiguration gespeichert | ✅ |
| Kamera-Livestream mit Tür-/Fenstererkennung (6 Klassen, `best.onnx`) | ✅ Video flüssig, KI auf dem Pi ~0,2 Bilder/s |
| KI-Erkennung auf dem MacBook (YOLO + Qwen3-VL) | 🟡 gebaut, Qwen-Teil noch nicht am echten Hund bestätigt |
| Dashboard (Demo, simuliert) für Mac, iPad, iPhone | ✅ als Link veröffentlicht |
| SLAM-Karte mit slam_toolbox, mobil über Foxglove (ohne Pi-Monitor) | ✅ Karte `raum1` (Klassenzimmer, ca. 9 × 8 m) gespeichert |
| Nav2 bewegt den Hund | ❌ fehlt noch `/cmd_vel`-Brücke |
| Dashboard Live-Version (echte Daten) | ❌ nach SLAM/Nav2 |

---

## 2. Hardware & Netzwerk

```
MacBook ──(Tailscale)──> Raspberry Pi 5 ──(LAN eth0)──> Go2 EDU (192.168.123.161)
                          User: fynn
                          Tailscale-IP: 100.121.161.1
                          Hostname: Rasp-Pi-5-Hund
```

- **Verbinden vom Mac:** `ssh fynn@100.121.161.1` (Tailscale muss auf Mac und Pi laufen)
- **Dateien Mac → Pi:** `scp ~/Downloads/DATEI fynn@100.121.161.1:~/`
- **Dateien Pi → Mac:** `scp fynn@100.121.161.1:~/PFAD/DATEI ~/Downloads/`
- **Test, ob der Pi den Hund sieht:** `ping -c 3 192.168.123.161`
- **RViz** braucht einen Bildschirm → am **Monitor direkt am Pi** starten, nicht über SSH.

> Merkhilfe Terminal: Steht vorne `maximilian@MacBook…` → du bist am **Mac**. Steht vorne `fynn@Rasp-Pi-5-Hund` → du bist auf dem **Pi**.
> Programm beenden auf dem Mac: **control + C** (nicht cmd).

---

## 3. Dateien & Ordner

### 3.1 ROS-Brücke – auf dem Pi in `~/unitree_ws/src/go2_bridge/`

| Datei | Zweck |
|---|---|
| `go2_env.sh` | Umgebung setzen (ROS, CycloneDDS auf `eth0`, Workaround für Absturz). **In jedem neuen Terminal zuerst sourcen.** |
| `go2_bridge.launch.py` | Startet die komplette Brücke (TF + `/scan`). Standard: `accum:=true` → dichter Scan über `go2_scan_accum.py` |
| `go2_scan_accum.py` | **Neu:** sammelt 0,6 s von `/utlidar/cloud_deskewed`, rechnet in `base_link` um und veröffentlicht einen dichten `/scan` (8 Hz, 1°-Raster). Ohne das wird die SLAM-Karte zum „Farbeimer“ |
| `go2_odom_tf.py` | Macht aus `/utlidar/robot_odom` den TF `odom → base_link` |
| `go2_cloud_fix.py` | Alter Lösungsversuch, wird **nicht mehr benutzt** (kann bleiben) |
| `go2.rviz` | Gespeicherte RViz-Ansicht |
| `slam_go2.yaml` | slam_toolbox-Einstellungen (Abschnitt 5.4) |
| `maps/raum1.pgm`, `maps/raum1.yaml` | Gespeicherte Karte (für Nav2) |
| `maps/raum1_slam.data`, `maps/raum1_slam.posegraph` | SLAM-Stand (zum Weiterkartieren / Lokalisieren mit slam_toolbox) |
| `README.md` | Kurz-Startanleitung |

### 3.2 Kamera/YOLO – auf dem Pi in `~/unitree_ws/src/yolo_zeugs/`

| Datei | Zweck |
|---|---|
| `best.onnx` | **Aktuelles** YOLO-Modell mit **6 Klassen** (siehe 7.1). Liegt nur lokal, ist **nicht** im Git-Repo („aus Git entfernt, lokal gesichert“) |
| `best.pt` | **Gibt es nicht mehr** (aus Repo und lokal gelöscht) |
| `go2_tuer_stream.py` | Läuft auf dem Pi: Go2-Kamera → (optional YOLO) → Browser-Stream |
| `config.json`, `detect.py`, `go2_integration.py`, `test_headless.py` | Team-Code (Logik „Tür zu → Stopp“, noch `simulation=True`) |
| `Train_YOLO_Models.ipynb` (Repo-Wurzel) | Colab-Notebook, mit dem das Modell trainiert wurde |

### 3.3 Auf dem MacBook (`~/Downloads`)

| Datei | Zweck |
|---|---|
| `go2_mac_erkennung.py` | KI auf dem Mac: YOLO auf jedem Bild + Qwen3-VL prüft Türen/Fenster |
| `best.onnx` | Kopie des Modells vom Pi (für `go2_mac_erkennung.py`) |
| `go2env/` | Python-Umgebung (venv) mit `ultralytics`, `onnxruntime`, `opencv-python` |
| `go2_dashboard_demo.html` | Dashboard-Demo (simuliert), Doppelklick öffnet sie |
| `raum1.pgm`, `raum1.yaml` | Kopie der Karte vom Pi |
| `go2_scan_accum.py`, `go2_bridge.launch.py` | Kopien der neuen Brücken-Dateien |
| Foxglove (App in „Programme“) | Live-Ansicht von Karte und Scan (ersetzt RViz) |
| `go2_tuer_fenster.py` | Ältere Mac-Variante (Webcam-Test / WebRTC), wird nicht mehr gebraucht |

---

## 4. Einmalige Installation

### 4.1 Auf dem Pi
```bash
sudo apt install -y ros-jazzy-pointcloud-to-laserscan ros-jazzy-rviz2 \
  ros-jazzy-slam-toolbox ros-jazzy-navigation2 ros-jazzy-nav2-bringup
```
Für den Kamera-Stream wurde zusätzlich das Python-Paket `cyclonedds` installiert:
```bash
export CYCLONEDDS_HOME=~/unitree_ws/cyclonedds/install
pip install --user --break-system-packages cyclonedds==0.10.2
```

### 4.2 Auf dem Mac (für KI auf dem Mac)
1. **Ollama** installieren: https://ollama.com/download → in „Programme“ ziehen → **starten** (Lama-Symbol in der Menüleiste).
2. Im Mac-Terminal:
```bash
ollama pull qwen3-vl:4b
ollama list                                   # qwen3-vl:4b muss erscheinen
scp fynn@100.121.161.1:~/unitree_ws/src/yolo_zeugs/best.onnx ~/Downloads/
python3 -m venv ~/Downloads/go2env            # nur falls noch nicht vorhanden
source ~/Downloads/go2env/bin/activate
pip install ultralytics onnxruntime opencv-python
```

---

## 5. Starten – Schritt für Schritt

### 5.1 Brücke (TF + /scan) – Pi, Terminal A, muss offen bleiben
```bash
source ~/unitree_ws/src/go2_bridge/go2_env.sh
ros2 launch ~/unitree_ws/src/go2_bridge/go2_bridge.launch.py
```
Erfolgreich, wenn erscheint: `Erste Odometrie empfangen - TF laeuft.`

### 5.2 Kurztest – Pi, Terminal B
```bash
source ~/unitree_ws/src/go2_bridge/go2_env.sh
ros2 topic hz /scan                          # ~14 Hz erwartet, Ende mit control+C
ros2 run tf2_ros tf2_echo odom base_link     # Werte müssen sich beim Bewegen ändern
```

### 5.3 RViz – am Pi-Monitor (Strg+Alt+T für Terminal)
```bash
source ~/unitree_ws/src/go2_bridge/go2_env.sh
rviz2 -d ~/unitree_ws/src/go2_bridge/go2.rviz
```
Falls ohne Config: Fixed Frame `odom`; Add → By topic → `/scan` (LaserScan) und `/utlidar/cloud_deskewed` (PointCloud2); Add → TF.
**Bei LaserScan und PointCloud2 immer `Reliability Policy = Best Effort` einstellen**, sonst wird nichts angezeigt.

### 5.4 SLAM – Karte erstellen (mobil, ohne Pi-Monitor) ✅
Der Pi hängt dabei an der **Powerbank** (LAN-Kabel zum Hund bleibt dran), alles wird vom MacBook über SSH gestartet. Die Karte wird in der **Foxglove-App** auf dem Mac angezeigt. RViz braucht man dafür nicht mehr.

**Einmalig:**
```bash
# Pi
sudo apt install -y ros-jazzy-foxglove-bridge
cp /opt/ros/jazzy/share/slam_toolbox/config/mapper_params_online_async.yaml ~/unitree_ws/src/go2_bridge/slam_go2.yaml
sed -i 's/base_frame: base_footprint/base_frame: base_link/' ~/unitree_ws/src/go2_bridge/slam_go2.yaml
mkdir -p ~/unitree_ws/src/go2_bridge/maps
```
Mac: **Foxglove-App** von foxglove.dev/download installieren und anmelden. Die Web-Version (app.foxglove.dev) funktioniert **nicht**, weil Browser `ws://` von einer `https://`-Seite blockieren.

**Ablauf:**
1. Pi sauber herunterfahren (`sudo shutdown now`), auf Powerbank umstecken, 1–2 min warten. Mac im selben Netz, Tailscale an.
2. Drei Mac-Terminals, jeweils `ssh fynn@100.121.161.1` und `source ~/unitree_ws/src/go2_bridge/go2_env.sh`, dann:
   - **Fenster 1 – Brücke:** `ros2 launch ~/unitree_ws/src/go2_bridge/go2_bridge.launch.py` → warten auf „/scan: XXX/360 Strahlen mit Treffer“ (gut: > 150)
   - **Fenster 2 – SLAM:** `ros2 launch slam_toolbox online_async_launch.py slam_params_file:=$HOME/unitree_ws/src/go2_bridge/slam_go2.yaml` → warten auf „Registering sensor“
   - **Fenster 3 – Foxglove:** `ros2 launch foxglove_bridge foxglove_bridge_launch.xml`
3. Foxglove-App: **Open connection → Foxglove WebSocket → `ws://100.121.161.1:8765`**. Im 3D-Panel: Display frame `map`, Topics `/map` und `/scan` einschalten (Auge-Symbol). Das rote Symbol bei `/scan` („Infinity values“) ist harmlos.
4. Hund per Fernbedienung **langsam** fahren, **besonders langsam drehen**: erst an den Wänden entlang, zurück zum Start (Schleife schließen), dann Mitte/Gänge.
5. **Neue Karte** = SLAM in Fenster 2 mit control + C beenden und neu starten.
6. **Speichern** (viertes Fenster, SLAM läuft noch!):
```bash
source ~/unitree_ws/src/go2_bridge/go2_env.sh
ros2 run nav2_map_server map_saver_cli -f ~/unitree_ws/src/go2_bridge/maps/raum1
ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph "{filename: '$HOME/unitree_ws/src/go2_bridge/maps/raum1_slam'}"
ls ~/unitree_ws/src/go2_bridge/maps/     # raum1.pgm, raum1.yaml, raum1_slam.data, raum1_slam.posegraph
```
7. Erst dann Fenster 1–3 mit control + C beenden, Pi mit `sudo shutdown now` herunterfahren.

Karte auf den Mac holen (**Mac-Terminal, ohne ssh**, Anführungszeichen wegen `*` nötig):
```bash
scp "fynn@100.121.161.1:~/unitree_ws/src/go2_bridge/maps/raum1.*" ~/Downloads/
```

**Ergebnis 02.11.2026:** `raum1` – Klassenzimmer, geschlossene Wände (ca. 9 × 8 m), vier Tischreihen als Punktreihen sichtbar, Tür/Nische unten rechts, unten links Wand minimal doppelt (unkritisch). Auflösung 0,05 m/Pixel.

### 5.5 Kamera-Livestream

**Variante A – alles auf dem Pi (Video flüssig, Rahmen nur alle paar Sekunden):**
```bash
cd ~/unitree_ws/src/yolo_zeugs && python3 go2_tuer_stream.py auto
```
Browser am Mac: **http://100.121.161.1:8080**

**Variante B – KI auf dem Mac (empfohlen):**
1. Pi: nur Video senden
   ```bash
   cd ~/unitree_ws/src/yolo_zeugs && python3 go2_tuer_stream.py auto --ohne-ki
   ```
2. Mac (Ollama-App muss laufen):
   ```bash
   source ~/Downloads/go2env/bin/activate && python ~/Downloads/go2_mac_erkennung.py
   ```
   Es öffnet sich ein Fenster. `q` = beenden, `s` = Screenshot. Nur **einmal** starten!

Wichtig: immer `auto` verwenden, **nicht** `eth0` (siehe 9.2).

### 5.6 Dashboard-Demo
- **Link (Mac, iPad, iPhone):** https://claude.ai/artifact/LE7bsdHYR95GqzDxwy8jzb – privat, für andere über „Teilen“ freigeben.
- **Datei:** `~/Downloads/go2_dashboard_demo.html` per Doppelklick.

---

## 6. Wichtige technische Erkenntnisse

### 6.1 Das Repo `unitree_go2_ros2_jazzy` ist nur Simulation
Es enthält CHAMP-Regler, URDF und Gazebo-Welten. Die Launch-Dateien (`unitree_go2_sim/launch/unitree_go2_launch.py`, `…_TI.py`) starten nur einen **virtuellen** Hund. Für den echten Hund gibt es darin **keinen Treiber**.

### 6.2 Der Go2 EDU braucht keinen Treiber
Er sendet seine Daten direkt per DDS über das LAN-Kabel. Mit `rmw_cyclonedds_cpp` und Interface `eth0` (siehe `go2_env.sh`) zeigt `ros2 topic list` u. a.:
- `/utlidar/cloud`, `/utlidar/cloud_base`, `/utlidar/cloud_deskewed` (3D-LiDAR, ~14 Hz)
- `/utlidar/robot_odom` (Odometrie, `odom → base_link`)
- `/utlidar/imu`, `/sportmodestate`, `/lowstate` u. v. m.

Es gibt aber **kein `/tf`** und **kein `/scan`** → beides liefert unsere Brücke.

### 6.3 Die drei Punktwolken – nur zwei sind brauchbar
| Topic | Frame im Header | Bewertung |
|---|---|---|
| `/utlidar/cloud` | `utlidar_lidar` | ❌ **Header ist falsch** – Daten sind weltausgerichtet, nicht im Sensor-Frame. Führte dazu, dass sich in RViz die Wände mit dem Hund drehten. |
| `/utlidar/cloud_base` | `base_link` | ✅ alte `/scan`-Quelle (`accum:=false`) |
| `/utlidar/cloud_deskewed` | `odom` | ✅ Anzeige **und** Quelle für den dichten `/scan` (`go2_scan_accum.py`) |

### 6.4 TF-Baum
```
map ──(slam_toolbox)──> odom ──(go2_odom_tf.py)──> base_link ──(statisch)──> utlidar_lidar
```

### 6.5 Einstellungen für Nav2 / slam_toolbox
- slam_toolbox: `base_frame: base_link`, `odom_frame: odom`, `map_frame: map`, `scan_topic: /scan`
- Nav2: `robot_base_frame: base_link`, `global_frame: map`, `odom_topic: /utlidar/robot_odom`

### 6.6 Stellschrauben der Brücke (Launch-Argumente)
| Argument | Standard | Bedeutung |
|---|---|---|
| `accum` | `true` | `true` = dichter Scan aus gesammelten Wolken (für SLAM nötig), `false` = alter Weg über pointcloud_to_laserscan |
| `accum_window` | `0.6` | Sammelzeit in s (mehr = dichter, aber beim Drehen eher verschmiert) |
| `cloud_topic` | `/utlidar/cloud_base` | Quelle für `/scan`, nur bei `accum:=false` |
| `min_height` | `-0.10` | Unterkante des Höhenbands relativ zu `base_link` (zu tief → Boden im Scan) |
| `max_height` | `0.40` | Oberkante des Höhenbands |
| `range_min` / `range_max` | `0.30` / `20.0` | Entfernungsgrenzen in m |
| `use_pi_time` | `false` | `true`, falls Uhren von Hund und Pi auseinanderlaufen |

Beispiel: `ros2 launch …/go2_bridge.launch.py min_height:=0.0`

---

## 7. Kamera & Bilderkennung

### 7.1 Modell `best.onnx` – 6 Klassen
| ID | Klasse | Bedeutung | Rahmenfarbe |
|---|---|---|---|
| 0 | `closed door` | Tür geschlossen | hellgrün |
| 1 | `door handle` | Türgriff | blau |
| 2 | `opened door` | Tür geöffnet | rot |
| 3 | `window` | Fenster | übernimmt den Zustand vom Griff in seinem Rahmen, sonst grau |
| 4 | `window handle horizontal` | Fenstergriff waagerecht = **offen** | rot |
| 5 | `window handle vertical` | Fenstergriff senkrecht = **zu** | hellgrün |

Annahme: senkrecht = zu, waagerecht = offen (Dreh-Kipp-Fenster). Ein nach oben zeigender Griff (gekippt) sieht für YOLO aus wie „senkrecht“. Gekippte Fenster erkennt nur Qwen3-VL (orange).

Klassen prüfen: `python3 -c "import onnxruntime as o; print(o.InferenceSession('best.onnx').get_modelmeta().custom_metadata_map.get('names'))"`

### 7.2 Stream auf dem Pi (`go2_tuer_stream.py`)
- Bildquelle: Go2-Frontkamera über `unitree_sdk2py` (`VideoClient`) per LAN.
- **Video und KI laufen getrennt:** Das Video läuft mit voller Kamerarate, die KI rechnet daneben und ihre Rahmen werden auf jedes Bild gezeichnet. Oben im Bild: „Video xx FPS | KI x/s“.
- Endpunkte: `http://100.121.161.1:8080` (Seite), `/stream` (mit Rahmen), `/roh` (ohne Rahmen, für den Mac).
- Optionen: `--ohne-ki`, `--model best.onnx`, `--conf 0.35`, `--frames 2`, `--imgsz 640`, `--test` (USB-Kamera).
- **Leistung:** `best.onnx` braucht auf der Pi-CPU ca. **5 s pro Bild** → KI auf dem Pi nur als Notlösung.

### 7.3 KI auf dem Mac (`go2_mac_erkennung.py`)
- **YOLO** auf jedem Bild (M3 Pro, ONNX Runtime) → flüssige Rahmen.
- **Qwen3-VL 4B** (lokal über Ollama) prüft reihum je **einen** Tür-/Fenster-Ausschnitt: „offen / zu / gekippt“ und überschreibt die YOLO-Farbe.
  - Label **„VL“** = Qwen bestätigt YOLO, **„VL!“** = Qwen hat korrigiert, **orange** = gekippt.
  - Qwen braucht ca. 1–3 s pro Objekt und prüft **nur Objekte, die YOLO gefunden hat**.
- Oben im Fenster: Zeile 1 „YOLO xx FPS“, Zeile 2 „VL: …“ (Status von Qwen).
- Optionen: `--ohne-vlm`, `--vlm qwen3-vl:8b` (genauer, langsamer), `--vlm-pause 0.3`, `--url …/roh`.

### 7.4 Erkennung verbessern
Mit eigenen Bildern aus dem Gebäude nachtrainieren (200–500 Bilder, besonders Griffe aus vielen Blickwinkeln) im Colab-Notebook `Train_YOLO_Models.ipynb`.

---

## 8. Dashboard (HMI)

### 8.1 Demo (fertig)
Vollständig simuliert, unabhängig vom Hund. Inhalte:
- **Karte** mit SLAM-Aufdeckung, Türen/Fenster grün/rot, Pfad, Wegpunkte, Ziel, „Mein Standort“
- **Rechtsklick / lange drücken** auf die Karte: Ziel setzen, Wegpunkt setzen, Mein Standort hier (+ Simulation: Hindernis platzieren, Tür öffnen/schließen)
- **Navigation** mit Ausweichen (A*-Planer mit Sicherheitsabstand), Neuplanung bei Hindernissen
- **Call**: Hund kommt zu „Mein Standort“ (hält ~1 m Abstand)
- **Kamera** (simuliert) mit Rahmen um Türen/Fenster, Tische und Stühle sichtbar
- **Steuerung**: Joystick + WASD/Q/E, Tempo-Regler, Not-Aus (Leertaste), Esc = Fahrt abbrechen
- **Befehle** (Unitree Sport-API): Aufstehen, Platz, Sitz, Balance, Hallo, Strecken, Herz, Tanzen, Hüftschwung, Handstand ⚠, Sprung ⚠, Motoren aus ⚠
- **System**: Akku, Temperaturen, Ping, CPU/RAM, WLAN/Tailscale, Topic-Raten, Ereignis-Log
- **Kartengenerator**: „Neues Klassenzimmer“ (Tafel, Lehrerpult, 3–4 Tischreihen mit Stühlen, Fensterfront, Flur mit Türen) oder „Labor“
- **Geräte**: Mac/iPad quer (Volldarstellung), iPad hochkant (zweispaltig), iPhone (Tabs: Fahren, Karte, Befehle, System), Zwei-Finger-Zoom
- **Design**: Dark Mode

### 8.2 Live-Version (geplant)
Gleiches Design, echte Daten über rosbridge (roslib.js), Kamera-Stream, Steuer-Node (`/cmd_vel` → Sport-API), Nav2-Aktionen (NavigateToPose / NavigateThroughPoses). Echtzeit und Sicherheit (Steuer-Node, Not-Aus, Nav2-Controller) bleiben auf dem Pi; KI und Visualisierung auf dem Mac.

---

## 9. Netzwerk-Konfiguration des Pi (Ubuntu Desktop)

- WLAN wird von **NetworkManager** verwaltet (Dateien `/etc/netplan/90-NM-*.yaml`).
- Hotspot hinzufügen:
  ```bash
  sudo nmcli device wifi connect "HOTSPOT_NAME" password "PASSWORT"
  sudo nmcli connection modify "HOTSPOT_NAME" connection.autoconnect yes connection.autoconnect-priority 10
  ```
- Reste `60-hotspot.yaml.save*` in `/etc/netplan/` sind harmlos und können gelöscht werden.

---

## 10. Bekannte Probleme & Lösungen

### 10.1 `ModuleNotFoundError: No module named 'cyclonedds'`
Python-Paket fehlt → siehe Abschnitt 4.1.

### 10.2 `*** buffer overflow detected ***: terminated`
Tritt auf, sobald CycloneDDS aus `~/unitree_ws/install/cyclonedds` mit explizitem Interface `eth0` gestartet wird.
- **ROS 2:** `go2_env.sh` entfernt `~/unitree_ws` aus `LD_LIBRARY_PATH` → ROS nutzt sein eigenes CycloneDDS → funktioniert.
- **Python-SDK (Kamera):** Interface nicht angeben → `go2_tuer_stream.py auto`.
- Hintergrund: `colcon build` warnt, dass das Workspace-Paket `cyclonedds` das von ROS überschreibt.

### 10.3 In RViz wird nichts angezeigt
`Reliability Policy` auf **Best Effort** stellen (bei `/map`: Durability `Transient Local`).

### 10.4 Wände drehen sich in RViz mit dem Hund mit
Falsche Wolke verwendet (`/utlidar/cloud`). Für die Anzeige `/utlidar/cloud_deskewed`, für `/scan` `/utlidar/cloud_base` nehmen (Standard in der Launch-Datei).

### 10.5 Boden erscheint im Scan (rote Fläche/Strahlen um den Hund)
`min_height` erhöhen, z. B. `min_height:=0.0`.

### 10.6 `Address already in use` beim Kamera-Stream
Altes Programm läuft noch: `pkill -f go2_tuer_stream.py`

### 10.7 Kamera-Stream mit 0,2 FPS
Alte Version von `go2_tuer_stream.py`, bei der das Video auf die KI wartet. Neue Version auf den Pi kopieren (Video und KI getrennt) oder Variante B (KI auf dem Mac) nutzen.

### 10.8 Qwen3-VL reagiert nicht
- Läuft die **neue** `go2_mac_erkennung.py`? Oben im Fenster muss eine zweite Zeile **„VL: …“** stehen.
- Läuft das Programm **zweimal** (zwei Terminal-Fenster)? Beide mit control + C beenden, nur einmal starten.
- Ist **Ollama** installiert und gestartet (Lama in der Menüleiste)? `ollama list` muss `qwen3-vl:4b` zeigen.
- Ist eine **Tür oder ein Fenster im Bild**? Qwen prüft nur, was YOLO vorher gefunden hat.

### 10.9 `scp: … No such file or directory`
Befehl im falschen Terminal (auf dem Pi statt am Mac) oder Datei liegt nicht in `~/Downloads`.

### 10.10 Link zum Dashboard / Python-Server geht nicht
- Artifact-Link ist privat → über „Teilen“ freigeben bzw. mit demselben claude.ai-Account anmelden.
- Lokaler Python-Server (`python3 -m http.server`): immer `http://` vor die Adresse; Hochschul-/Gäste-WLAN blockiert Geräte untereinander oft.

### 10.11 Häufige Tippfehler
`unitree` (nicht `untiree`), `install` (nicht `isntall`), `head -3` (mit Leerzeichen), `rviz2` (mit 2).

### 10.12 Unitree-App
Die App stört die LAN/DDS-Verbindung normalerweise nicht. Für WebRTC-Verbindungen muss sie aber geschlossen sein – der Go2 erlaubt nur eine WebRTC-Verbindung.

### 10.13 SLAM-Karte sieht aus wie ein „ausgeleerter Farbeimer“
Weiße strahlenförmige Fläche, kaum Wände. Ursache: Der L1-LiDAR liefert pro Wolke nur wenige Punkte → jeder einzelne `/scan` fast leer → slam_toolbox findet keine Wände und verrutscht. Lösung: `go2_scan_accum.py` (Standard in der Brücke, `accum:=true`). Außerdem langsam drehen.

### 10.14 Foxglove im Browser: „WebSocket SSL Error“
Browser blockieren `ws://` von `https://app.foxglove.dev`. → Foxglove-**App** für den Mac nehmen.

### 10.15 `zsh: no matches found` bei `scp …raum1.*`
Die Mac-Konsole (zsh) will das `*` selbst auflösen. → Pfad in Anführungszeichen setzen.

---

## 11. Offene Punkte / nächste Schritte

1. ~~Karte aufnehmen~~ ✅ `raum1` gespeichert (02.11.2026).
2. **`/cmd_vel`-Brücke schreiben:** Nav2 gibt Fahrbefehle auf `/cmd_vel` aus, der Go2 hört aber nur auf die Unitree-Sport-API.
3. **Nav2** mit gespeicherter Karte konfigurieren (Frames siehe 6.5).
4. **Qwen3-VL am echten Hund testen** (Ollama installieren, Abschnitt 4.2).
5. **Erkennung verbessern** (Nachtraining mit eigenen Bildern).
6. **Dashboard Live-Version** auf Basis der Demo.
7. `go2_integration.py` von `simulation=True` auf echte SDK-Befehle umstellen (Stopp bei geschlossener Tür).
8. Optional: Workspace-Paket `cyclonedds` sauber vom ROS-CycloneDDS trennen.
9. Dateien auf GitHub hochladen (`src/go2_bridge`, `src/yolo_zeugs` – ohne `best.onnx`, falls bewusst lokal gehalten).

---

## 12. Schnellstart (Spickzettel)

```bash
# Mac → Pi verbinden
ssh fynn@100.121.161.1

# Pi – Terminal A: Brücke (TF + /scan)
source ~/unitree_ws/src/go2_bridge/go2_env.sh
ros2 launch ~/unitree_ws/src/go2_bridge/go2_bridge.launch.py

# Pi – Terminal B: SLAM
source ~/unitree_ws/src/go2_bridge/go2_env.sh
ros2 launch slam_toolbox online_async_launch.py slam_params_file:=$HOME/unitree_ws/src/go2_bridge/slam_go2.yaml

# Pi – Terminal C: Foxglove (Mac: Foxglove-App → ws://100.121.161.1:8765)
source ~/unitree_ws/src/go2_bridge/go2_env.sh
ros2 launch foxglove_bridge foxglove_bridge_launch.xml

# Pi – Karte speichern (SLAM muss laufen)
ros2 run nav2_map_server map_saver_cli -f ~/unitree_ws/src/go2_bridge/maps/raum1

# Pi-Monitor: RViz
source ~/unitree_ws/src/go2_bridge/go2_env.sh
rviz2 -d ~/unitree_ws/src/go2_bridge/go2.rviz

# Pi: Kamera nur Video (KI auf dem Mac)
cd ~/unitree_ws/src/yolo_zeugs && python3 go2_tuer_stream.py auto --ohne-ki

# Mac: KI (YOLO + Qwen3-VL), Ollama-App muss laufen
source ~/Downloads/go2env/bin/activate && python ~/Downloads/go2_mac_erkennung.py
```
