"""
Go2 Kamera-Stream mit Tür-/Fenstererkennung (läuft AUF DEM PI, ohne Unitree-App)
--------------------------------------------------------------------------------
  Go2-Frontkamera --(unitree_sdk2py, LAN)--> Pi --> MJPEG-Stream --(Tailscale)--> Browser/Mac

Das Video läuft immer mit voller Kamerarate. Die KI rechnet daneben so schnell sie kann,
die Rahmen werden auf jedes Bild gezeichnet (sie aktualisieren sich nur seltener).

Modell-Klassen (best.onnx): 0 closed door, 1 door handle, 2 opened door, 3 window,
                            4 window handle horizontal, 5 window handle vertical
Farben: zu = hellgrün, offen = rot, Türgriff = blau, Fenster ohne erkannten Griff = grau
        (Fenster übernehmen den Zustand vom Griff in ihrem Rahmen: senkrecht = zu, waagerecht = offen)

Start auf dem Pi:
    cd ~/unitree_ws/src/yolo_zeugs
    python3 go2_tuer_stream.py auto                # Video + KI auf dem Pi
    python3 go2_tuer_stream.py auto --ohne-ki      # nur Video (KI läuft dann auf dem Mac)
Browser am Mac:
    http://100.121.161.1:8080          Seite mit Video
    http://100.121.161.1:8080/roh      rohes Kamerabild (für go2_mac_fenster.py)
    http://100.121.161.1:8080/roh?w=640&q=60   verkleinert/stärker komprimiert (langsames Netz)
    http://100.121.161.1:8080/status   Bilder/s, Bildgröße, Fehler (für das Dashboard)
Mit --ohne-ki wird das JPEG der Hundekamera unverändert durchgereicht (kein Umrechnen, spart CPU).

Optionen: --model best.onnx  --conf 0.35  --frames 2  --imgsz 640  --port 8080  --test (USB-Kamera)
Beenden: control + C
"""

import argparse
import json
import threading
import time
from urllib.parse import parse_qs, urlparse
from collections import defaultdict
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import cv2
import numpy as np

HELLGRUEN = (144, 238, 144)  # BGR
ROT = (0, 0, 255)
BLAU = (246, 130, 59)
GRAU = (180, 180, 180)

lock = threading.Lock()
roh_bild, roh_id = None, 0     # neuestes Kamerabild + Zähler
anzeige = []                   # aktuelle Rahmen: (x1, y1, x2, y2, text, farbe)
zaehler = {"offen": 0, "zu": 0}
ki_fps, cam_fps = 0.0, 0.0
fertig_jpeg = None             # Bild mit Rahmen
roh_jpeg = None                # Bild ohne Rahmen
orig_jpeg, orig_id = None, 0   # JPEG genau so, wie es von der Hundekamera kommt (ohne Umkodieren)
OHNE_KI = False                # --ohne-ki: Bilder werden auf dem Pi gar nicht erst dekodiert
statistik = {"bilder": 0, "fps": 0.0, "clients": 0, "fehler": 0, "letzter_fehler": ""}
status = "Starte ..."
stop = threading.Event()


# --------------------------------------------------------------------------- Kamera
def neues_bild(img):
    global roh_bild, roh_id
    with lock:
        roh_bild = img
        roh_id += 1


def kamera_go2(interface: str):
    global status
    from unitree_sdk2py.core.channel import ChannelFactoryInitialize
    from unitree_sdk2py.go2.video.video_client import VideoClient
    if interface == "auto":
        ChannelFactoryInitialize(0)
    else:
        ChannelFactoryInitialize(0, interface)
    client = VideoClient()
    client.SetTimeout(3.0)
    client.Init()
    print(f"Verbinde mit Go2-Kamera über {interface} ...")
    fehler = 0
    while not stop.is_set():
        code, data = client.GetImageSample()
        if code != 0:
            fehler += 1
            statistik["fehler"] += 1
            statistik["letzter_fehler"] = f"Code {code}"
            status = (f"Keine Kamera-Daten (Code {code}" +
                      (" = Zeitüberschreitung: Videodienst im Hund antwortet nicht" if code == 3104 else "") + ")")
            if fehler % 10 == 1:
                print(status)
            time.sleep(0.3)
            continue
        fehler = 0
        neues_jpeg(bytes(data))


def neues_jpeg(jpg):
    """Kamera-JPEG merken. Ohne KI wird es NICHT dekodiert/neu kodiert (spart viel CPU auf dem Pi)."""
    global orig_jpeg, orig_id, roh_jpeg, fertig_jpeg, status
    with lock:
        orig_jpeg, orig_id = jpg, orig_id + 1
    statistik["bilder"] += 1
    if OHNE_KI:
        roh_jpeg = fertig_jpeg = jpg
        status = "Läuft"
        return
    img = cv2.imdecode(np.frombuffer(jpg, dtype=np.uint8), cv2.IMREAD_COLOR)
    if img is not None:
        neues_bild(img)


def roh_dekodiert():
    """-> (bild, id) des neuesten Kamerabilds; dekodiert bei Bedarf aus dem Original-JPEG."""
    with lock:
        if roh_bild is not None and not OHNE_KI:
            return roh_bild, roh_id
        jpg, nr = orig_jpeg, orig_id
    if jpg is None:
        return None, nr
    return cv2.imdecode(np.frombuffer(jpg, dtype=np.uint8), cv2.IMREAD_COLOR), nr


def statistik_ausgeben():
    """Alle 5 s eine Zeile im Terminal (Tab 7:Kamera): wie viele Bilder kommen wirklich an?"""
    while not stop.is_set():
        stop.wait(5.0)
        n, statistik["bilder"] = statistik["bilder"], 0
        statistik["fps"] = round(n / 5.0, 1)
        print(time.strftime("%H:%M:%S"), f"Kamera {n / 5.0:4.1f} Bilder/s | "
              f"Bildgroesse {len(orig_jpeg or b'') // 1024} KB | Zuschauer {statistik['clients']}", flush=True)


def kamera_usb(index: int = 0):
    global status
    cap = cv2.VideoCapture(index)
    if not cap.isOpened():
        status = "USB-Kamera nicht gefunden"
        print(status)
        return
    while not stop.is_set():
        ok, img = cap.read()
        if ok:
            neues_bild(img)


# --------------------------------------------------------------------------- Klassen-Logik
def klasse_deuten(name: str):
    """-> (Text, Zustand)  Zustand: 'offen', 'zu' oder None"""
    n = name.lower()
    if n == "closed door":               return "Tuer zu", "zu"
    if n == "opened door":               return "Tuer offen", "offen"
    if n == "door handle":               return "Tuergriff", None
    if n == "window handle vertical":    return "Griff senkrecht", "zu"
    if n == "window handle horizontal":  return "Griff waagerecht", "offen"
    if n == "window":                    return "Fenster", None
    return name, None


def farbe(zustand):
    return ROT if zustand == "offen" else HELLGRUEN if zustand == "zu" else BLAU


def boxen_zu_anzeige(sichtbar, names):
    """sichtbar: [(cls, conf, x1, y1, x2, y2)] -> ([(x1,y1,x2,y2,text,farbe)], n_offen, n_zu)"""
    griffe = []
    for cls, conf, x1, y1, x2, y2 in sichtbar:
        text, zustand = klasse_deuten(names[cls])
        if text.startswith("Griff"):
            griffe.append((zustand, (x1, y1, x2, y2)))
    out, n_offen, n_zu = [], 0, 0
    for cls, conf, x1, y1, x2, y2 in sichtbar:
        text, zustand = klasse_deuten(names[cls])
        if text == "Fenster":
            zustand = None
            for z, (gx1, gy1, gx2, gy2) in griffe:
                cx, cy = (gx1 + gx2) / 2, (gy1 + gy2) / 2
                if x1 <= cx <= x2 and y1 <= cy <= y2:
                    zustand = z
                    break
            text = "Fenster offen" if zustand == "offen" else "Fenster zu" if zustand == "zu" else "Fenster"
            col = farbe(zustand) if zustand else GRAU
        else:
            col = farbe(zustand)
        out.append((x1, y1, x2, y2, f"{text} {conf:.0%}", col))
        if text.startswith(("Tuer ", "Fenster ")):
            n_offen += zustand == "offen"
            n_zu += zustand == "zu"
    return out, n_offen, n_zu


def zeichne_box(img, x1, y1, x2, y2, text, col):
    cv2.rectangle(img, (x1, y1), (x2, y2), col, 3)
    (tw, th), _ = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 2)
    yt = max(y1, th + 8)
    cv2.rectangle(img, (x1, yt - th - 8), (x1 + tw + 8, yt), col, -1)
    cv2.putText(img, text, (x1 + 4, yt - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 2)


# --------------------------------------------------------------------------- KI (eigener Thread)
def erkennung(args):
    global anzeige, zaehler, ki_fps
    from ultralytics import YOLO
    model = YOLO(args.model, task="detect")
    print(f"Modell geladen: {model.names}")
    streak = defaultdict(int)
    letzte = time.time()
    while not stop.is_set():
        with lock:
            img = None if roh_bild is None else roh_bild.copy()
        if img is None:
            time.sleep(0.2)
            continue
        res = model.predict(img, conf=args.conf, imgsz=args.imgsz, verbose=False)[0]
        boxen, gesehen = [], set()
        for b in res.boxes:
            cls = int(b.cls)
            gesehen.add(cls)
            boxen.append((cls, float(b.conf), *map(int, b.xyxy[0].tolist())))
        for cls in list(streak):
            if cls not in gesehen:
                streak[cls] = 0
        for cls in gesehen:
            streak[cls] += 1
        sichtbar = [b for b in boxen if streak[b[0]] >= args.frames]
        out, n_offen, n_zu = boxen_zu_anzeige(sichtbar, model.names)
        with lock:
            anzeige = out
            zaehler = {"offen": n_offen, "zu": n_zu}
        jetzt = time.time()
        ki_fps = 0.8 * ki_fps + 0.2 / max(jetzt - letzte, 1e-6)
        letzte = jetzt


# --------------------------------------------------------------------------- Bilder für den Stream
def darstellung(args):
    """Zeichnet die letzten Rahmen auf JEDES neue Kamerabild -> flüssiges Video."""
    global fertig_jpeg, roh_jpeg, cam_fps, status
    zuletzt_id, letzte = -1, time.time()
    while not stop.is_set():
        with lock:
            img = None if roh_bild is None or roh_id == zuletzt_id else roh_bild.copy()
            zuletzt_id = roh_id
            boxen, z = list(anzeige), dict(zaehler)
        if img is None:
            if roh_bild is None:
                leer = np.zeros((360, 640, 3), np.uint8)
                cv2.putText(leer, status, (10, 180), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
                fertig_jpeg = roh_jpeg = cv2.imencode(".jpg", leer)[1].tobytes()
            time.sleep(0.01)
            continue
        status = "Läuft"
        roh_jpeg = cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, 80])[1].tobytes()
        for x1, y1, x2, y2, text, col in boxen:
            zeichne_box(img, x1, y1, x2, y2, text, col)
        jetzt = time.time()
        cam_fps = 0.9 * cam_fps + 0.1 / max(jetzt - letzte, 1e-6)
        letzte = jetzt
        ki = "aus (Mac)" if args.ohne_ki else f"{ki_fps:.1f}/s"
        info = f"Video {cam_fps:4.1f} FPS | KI {ki} | offen: {z['offen']} | zu: {z['zu']}"
        cv2.putText(img, info, (10, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 4)
        cv2.putText(img, info, (10, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        fertig_jpeg = cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, 80])[1].tobytes()


# --------------------------------------------------------------------------- Webserver
SEITE = b"""<!doctype html><html><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Go2 Kamera</title>
<style>body{margin:0;background:#111;color:#eee;font-family:sans-serif;text-align:center}
img{max-width:100%;height:auto;margin-top:8px}
.l span{display:inline-block;margin:6px 10px}.g{color:#90ee90}.r{color:#f44}.b{color:#3b82f6}.y{color:#bbb}</style></head>
<body><div class="l"><b>Go2 - Tueren &amp; Fenster</b>
<span class="g">&#9632; zu</span><span class="r">&#9632; offen</span><span class="b">&#9632; Tuergriff</span><span class="y">&#9632; Fenster (Zustand unbekannt)</span></div>
<img src="/stream"></body></html>"""


class Handler(BaseHTTPRequestHandler):
    def log_message(self, *a):
        pass

    def _mjpeg(self, quelle):
        statistik["clients"] += 1
        try:
            self._mjpeg_senden(quelle)
        finally:
            statistik["clients"] -= 1

    def _mjpeg_senden(self, quelle):
        self.send_response(200)
        self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
        self.send_header("Cache-Control", "no-cache")
        self.end_headers()
        last = None
        try:
            while not stop.is_set():
                jpg = quelle()
                if jpg and jpg is not last:
                    last = jpg
                    self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\n"
                                     + f"Content-Length: {len(jpg)}\r\n\r\n".encode() + jpg + b"\r\n")
                time.sleep(0.01)
        except (BrokenPipeError, ConnectionResetError):
            pass

    def _mjpeg_angepasst(self, breite, qualitaet):
        """Rohbild pro Client verkleinern/komprimieren (für langsame Verbindungen über Tailscale)."""
        cache = {"id": -1, "jpg": None}

        def quelle():
            with lock:
                nr = orig_id
            if nr == cache["id"]:
                return cache["jpg"]
            img, cache["id"] = roh_dekodiert()
            if img is None:
                return cache["jpg"]
            h, w = img.shape[:2]
            if breite and w > breite:
                img = cv2.resize(img, (breite, int(h * breite / w)), interpolation=cv2.INTER_AREA)
            cache["jpg"] = cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, qualitaet])[1].tobytes()
            return cache["jpg"]
        self._mjpeg(quelle)

    def do_GET(self):
        if self.path.startswith("/status"):           # für das Dashboard
            body = json.dumps({"fps": statistik["fps"], "kb": len(orig_jpeg or b"") // 1024,
                               "zuschauer": statistik["clients"], "fehler": statistik["fehler"],
                               "letzter_fehler": statistik["letzter_fehler"], "status": status}).encode()
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.send_header("Access-Control-Allow-Origin", "*")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)
        elif self.path.startswith("/stream"):
            self._mjpeg(lambda: fertig_jpeg)
        elif self.path.startswith("/roh"):
            q = parse_qs(urlparse(self.path).query)
            breite = int(q.get("w", ["0"])[0] or 0)
            qualitaet = max(20, min(95, int(q.get("q", ["80"])[0] or 80)))
            if 0 < breite < 1280:                       # nur echtes Verkleinern kostet Rechenzeit
                self._mjpeg_angepasst(breite, qualitaet)
            else:                                       # Original-JPEG der Kamera durchreichen
                self._mjpeg(lambda: orig_jpeg if OHNE_KI else roh_jpeg)
        else:
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.end_headers()
            self.wfile.write(SEITE)


def main():
    p = argparse.ArgumentParser(description="Go2 Kamera-Stream mit Tuer-/Fenstererkennung")
    p.add_argument("interface", nargs="?", default="auto", help="'auto' (empfohlen) oder z. B. eth0")
    p.add_argument("--test", action="store_true", help="USB-Kamera am Pi statt Hundekamera")
    p.add_argument("--ohne-ki", dest="ohne_ki", action="store_true", help="nur Video, KI läuft auf dem Mac")
    p.add_argument("--model", default="best.onnx")
    p.add_argument("--conf", type=float, default=0.35)
    p.add_argument("--frames", type=int, default=2, help="Objekt erst nach so vielen KI-Durchläufen in Folge zeigen")
    p.add_argument("--imgsz", type=int, default=640)
    p.add_argument("--port", type=int, default=8080)
    args = p.parse_args()
    global OHNE_KI
    OHNE_KI = args.ohne_ki and not args.test

    quelle = (lambda: kamera_usb(0)) if args.test else (lambda: kamera_go2(args.interface))
    threading.Thread(target=quelle, daemon=True).start()
    if not OHNE_KI:
        threading.Thread(target=darstellung, args=(args,), daemon=True).start()
    threading.Thread(target=statistik_ausgeben, daemon=True).start()
    if not args.ohne_ki:
        threading.Thread(target=erkennung, args=(args,), daemon=True).start()

    server = ThreadingHTTPServer(("0.0.0.0", args.port), Handler)
    print(f"Stream läuft:  http://100.121.161.1:{args.port}   (control + C zum Beenden)")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        stop.set()
        server.shutdown()


if __name__ == "__main__":
    main()
