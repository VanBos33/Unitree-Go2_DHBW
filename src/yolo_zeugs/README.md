# yolo_zeugs – Bilderkennung Türen und Fenster

Erkennt mit einem eigenen YOLO-Modell Türen, Fenster und deren Griffe in den Bildern der Go2-Frontkamera und leitet daraus ab, ob ein Fenster **offen/gekippt** oder **geschlossen** ist. Im Projekt zählt vor allem der Fensterzustand.

Der Ordner ist **kein colcon-Paket**. Die Skripte werden direkt mit `python3` aus diesem Ordner gestartet:

```bash
cd ~/unitree_ws/src/yolo_zeugs
```

---

## 1. Dateien

| Datei | Zweck |
|---|---|
| `go2_tuer_stream.py` | Live-Stream: Go2-Frontkamera → (YOLO) → Browser/Mac (MJPEG über HTTP, Port 8080) |
| `go2_mac_fenster.py` (Kopie, läuft auf dem MacBook in `~/Downloads`) | Kamera-Fenster am Mac: Video immer, Fensterprüfung mit YOLO nur, solange der Hund sitzt |
| `go2_integration.py` | Klasse `GO2InspectionDetector`: wertet ein Bild aus und liefert einen Inspektionsbericht (Fenster- und Türstatus) |
| `detect.py` | Minimalbeispiel: YOLO auf einer USB-/CSI-Kamera (`cv2.VideoCapture(0)`), Anzeige im Fenster, Ende mit `q` |
| `test_headless.py` | Test ohne Bildschirm: wertet `IMG_1944.JPEG` aus, speichert `output_result.jpg` |
| `config.json` | Einstellungen für die Missions-Logik: Modell `best.onnx`, `conf_threshold 0.35`, `required_frames 3`, RTSP-Adresse, Logging |
| `mission_log.txt` | Protokoll der Testläufe vom 24.–25.10.2026 (Erkennung „opened window“, 86,4 %) |
| `IMG_1647.JPEG`, `IMG_1673.JPEG`, `IMG_1685.JPEG`, `IMG_1790.JPEG`, `IMG_1944.JPEG` | Testbilder aus dem Gebäude |
| `output_result.jpg` | Ergebnisbild von `test_headless.py` |
| `runs/detect/predict/bus.jpg` | Beispielausgabe von Ultralytics (Installationstest) |

**Nicht im Git:** die Modelldateien `best.onnx` und `best.pt` (per `.gitignore` ausgeschlossen). Sie liegen lokal auf dem Pi in diesem Ordner. Auf einen anderen Rechner kopiert man sie mit `scp`, im Mac-Terminal:

```bash
scp fynn@100.121.161.1:~/unitree_ws/src/yolo_zeugs/best.onnx ~/Desktop/
scp ~/Desktop/best.onnx fynn@100.121.161.1:~/unitree_ws/src/yolo_zeugs/
```

---

## 2. Modell `best.onnx` – 6 Klassen

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

---

## 3. Live-Stream im Browser: `go2_tuer_stream.py`

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
| `/roh` | Videostream ohne Rahmen, z. B. für eine Erkennung auf dem MacBook |
| `/roh?w=640&q=60` | wie `/roh`, aber auf 640 px Breite verkleinert und mit JPEG-Qualität 60 (für langsame Verbindungen über Tailscale) |
| `/status` | JSON: Bilder pro Sekunde, Bildgröße, Zuschauer, Fehlerzähler (für das Dashboard) |

Mit `--ohne-ki` reicht der Stream das JPEG der Hundekamera unverändert durch (kein Entpacken und Neupacken auf dem Pi). Nur `?w=` kleiner als 1280 rechnet auf dem Pi. Alle 5 s steht im Tab `7:Kamera`, wie viele Bilder pro Sekunde wirklich ankommen. „Code 3104“ heißt: Zeitüberschreitung, der Videodienst im Hund antwortet nicht (nicht das LAN-Kabel).

Mit der erweiterten Fassung: `python3 go2_tuer_stream.py auto --model best.onnx --frames 2`

- Rahmenfarben: **hellgrün** = geschlossen, **rot** = offen. Oben im Bild stehen FPS und die Anzahl offener/geschlossener Objekte.
- Auf der Pi-CPU braucht `best.onnx` bei voller Größe mehrere Sekunden pro Bild. Mit `--imgsz 320` wird es schneller, aber ungenauer.
- `Address already in use`: Ein alter Stream läuft noch → `pkill -f go2_tuer_stream.py`.
- Die Unitree-App stört den Stream nicht. Der Go2 erlaubt aber nur eine WebRTC-Verbindung gleichzeitig.

---

## 4. Kamera-Fenster am Mac mit automatischem Fenster-Scan: `go2_mac_fenster.py`

Das Fenster ist immer offen. Es zeigt das Video vom Pi (`/roh`) und fragt alle 0,5 s den Zustand der Mission ab (`http://100.121.161.1:8081/state`).

| Mission | Anzeige im Fenster | KI |
|---|---|---|
| läuft, richtet sich aus, steht auf | „Hund laeuft zu fenster_1 – KI aus“ | aus |
| `SITTING` | „Hund SITZT an fenster_1 – Fenster-Scan aktiv“ + Ergebnis groß | YOLO auf jedem Bild, nur die Klassen `window`, `window handle horizontal`, `window handle vertical` |

**Ergebnis:** Hauptfenster = größter Fensterrahmen. Liegt ein Griff darin (+15 % Rand), entscheidet der Griff: senkrecht = **ZU** (grün), waagerecht = **OFFEN** (rot). Ein Rahmen ohne Griff ergibt **UNKLAR** (orange), nichts erkannt **KEIN FENSTER IM BILD**. Gekippte Fenster gibt es in den Projekträumen nicht.

**Stabilisierung:** Ein Ergebnis gilt erst, wenn es 2 s lang (mindestens 3 Bilder) gleich bleibt. Vorher steht dort „pruefe …“. Jedes neue stabile Ergebnis wird per `POST /fenster` an den Pi gemeldet und erscheint dort auf dem Topic `/window_status`.

**Verbindung:** Ruckelt das Video (unter 7 FPS), fordert das Programm vom Pi ein kleineres Bild an (960 → 640 → 480 px). Läuft es 30 s flüssig, geht es wieder eine Stufe hoch. Unten im Fenster stehen Video-FPS, KI-FPS und die aktuelle Bildgröße.

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
| `--conf` | `0.35` | Mindest-Konfidenz |
| `--imgsz` | `640` | Bildgröße für YOLO |
| `--stabil` | `2.0` s | Zeit, bis ein Ergebnis gilt |
| `--immer-ki` | aus | KI unabhängig vom Zustand (zum Testen) |

Tasten: `q` beendet, `s` speichert einen Screenshot nach `~/Downloads`. Die Python-Umgebung `~/Downloads/go2env` enthält `ultralytics`, `onnxruntime` und `opencv-python`.

## 5. Inspektion eines Bildes: `go2_integration.py`

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

---

## 6. Kurztests

```bash
python3 test_headless.py      # erkennt Objekte in IMG_1944.JPEG, speichert output_result.jpg
python3 detect.py             # nur mit Bildschirm und Kamera am Pi, Ende mit q
```

`detect.py` lädt `best.pt`. Fehlt diese Datei oder ist sie beschädigt (`invalid load key, '\x0d'`), zum Testen in Zeile 5 `YOLO('best.onnx')` eintragen.

---

## 7. Modell trainieren

Das Notebook `Train_YOLO_Models.ipynb` in der Repo-Wurzel (Google Colab, Vorlage von EJ Technology Consultants) trainiert YOLO11/YOLOv8 mit eigenen Bildern:

1. Bilder aus dem Gebäude aufnehmen (empfohlen 200–500, Griffe aus vielen Blickwinkeln) und in Roboflow mit den 6 Klassen labeln.
2. Datensatz im Notebook hochladen, in Train/Validation aufteilen, trainieren.
3. `best.pt` herunterladen und als ONNX exportieren:
   ```bash
   yolo export model=best.pt format=onnx
   ```
4. `best.onnx` per `scp` nach `~/unitree_ws/src/yolo_zeugs/` auf den Pi kopieren.

---

## 8. Abhängigkeiten

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
