"""
Go2 Tür-/Fenstererkennung auf dem Raspberry Pi -> Live-Stream im Browser
------------------------------------------------------------------------
Läuft AUF DEM PI (der per LAN-Kabel am Go2 hängt). Ohne Unitree-App.

  Go2-Frontkamera --(unitree_sdk2py, LAN)--> Pi: YOLO best.pt --> MJPEG-Stream --(Tailscale)--> Browser am Mac

Rahmen: hellgrün = geschlossen, rot = offen
Modell-Klassen: 0 closed door, 1 closed window, 2 opened door, 3 opened window

Start auf dem Pi:
    cd ~/unitree_ws/src/yolo_zeugs
    python3 go2_tuer_stream.py eth0
Dann am Mac im Browser öffnen:
    http://100.121.161.1:8080

Optionen:
    --model best.pt   --conf 0.35   --frames 3   --imgsz 640   --port 8080
    --test            nimmt statt der Hundekamera eine USB-Kamera am Pi (zum Testen)
Beenden: Strg+C
"""

import argparse
import threading
import time
from collections import defaultdict
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import cv2
import numpy as np
from ultralytics import YOLO

HELLGRUEN = (144, 238, 144)  # BGR
ROT = (0, 0, 255)

lock = threading.Lock()
roh_bild = None        # neuestes Kamerabild
fertig_jpeg = None     # neuestes Bild mit Rahmen, als JPEG
status = "Starte ..."
stop = threading.Event()


# --------------------------------------------------------------------------- Kamera
def kamera_go2(interface: str):
    """Holt Bilder über das offizielle Unitree SDK2 (wie example/go2/front_camera)."""
    global roh_bild, status
    from unitree_sdk2py.core.channel import ChannelFactoryInitialize
    from unitree_sdk2py.go2.video.video_client import VideoClient

    if interface == "auto":
        ChannelFactoryInitialize(0)  # DDS sucht das Interface selbst
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
            status = f"Keine Kamera-Daten (Code {code}) - LAN-Kabel / IP von {interface} prüfen"
            if fehler % 10 == 1:
                print(status)
            time.sleep(0.3)
            continue
        fehler = 0
        img = cv2.imdecode(np.frombuffer(bytes(data), dtype=np.uint8), cv2.IMREAD_COLOR)
        if img is not None:
            with lock:
                roh_bild = img


def kamera_usb(index: int = 0):
    global roh_bild, status
    cap = cv2.VideoCapture(index)
    if not cap.isOpened():
        status = "USB-Kamera nicht gefunden"
        print(status)
        return
    while not stop.is_set():
        ok, img = cap.read()
        if ok:
            with lock:
                roh_bild = img


# --------------------------------------------------------------------------- Erkennung
def klasse_deuten(name: str):
    n = name.lower()
    art = "Tuer" if "door" in n else "Fenster" if "window" in n else name
    return art, n.startswith("open")


def zeichne_box(img, x1, y1, x2, y2, text, farbe):
    cv2.rectangle(img, (x1, y1), (x2, y2), farbe, 3)
    (tw, th), _ = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 2)
    yt = max(y1, th + 8)
    cv2.rectangle(img, (x1, yt - th - 8), (x1 + tw + 8, yt), farbe, -1)
    cv2.putText(img, text, (x1 + 4, yt - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 2)


def erkennung(args):
    global fertig_jpeg, status
    model = YOLO(args.model)
    print(f"Modell geladen: {model.names}")
    streak = defaultdict(int)
    letzte, fps = time.time(), 0.0

    while not stop.is_set():
        with lock:
            img = None if roh_bild is None else roh_bild.copy()
        if img is None:
            leer = np.zeros((360, 640, 3), np.uint8)
            cv2.putText(leer, status, (10, 180), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
            fertig_jpeg = cv2.imencode(".jpg", leer)[1].tobytes()
            time.sleep(0.3)
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

        n_offen = n_zu = 0
        for cls, conf, x1, y1, x2, y2 in boxen:
            if streak[cls] < args.frames:
                continue
            art, offen = klasse_deuten(model.names[cls])
            zeichne_box(img, x1, y1, x2, y2, f"{art} {'offen' if offen else 'zu'} {conf:.0%}",
                        ROT if offen else HELLGRUEN)
            n_offen += offen
            n_zu += not offen

        jetzt = time.time()
        fps = 0.9 * fps + 0.1 / max(jetzt - letzte, 1e-6)
        letzte = jetzt
        info = f"{fps:4.1f} FPS | offen: {n_offen} | zu: {n_zu}"
        cv2.putText(img, info, (10, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 0), 4)
        cv2.putText(img, info, (10, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        status = "Läuft"
        fertig_jpeg = cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, 80])[1].tobytes()


# --------------------------------------------------------------------------- Webserver
SEITE = b"""<!doctype html><html><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Go2 Kamera</title>
<style>body{margin:0;background:#111;color:#eee;font-family:sans-serif;text-align:center}
img{max-width:100%;height:auto;margin-top:8px}
.l span{display:inline-block;margin:6px 10px}.g{color:#90ee90}.r{color:#f44}</style></head>
<body><div class="l"><b>Go2 - Tueren &amp; Fenster</b>
<span class="g">&#9632; geschlossen</span><span class="r">&#9632; offen</span></div>
<img src="/stream"></body></html>"""


class Handler(BaseHTTPRequestHandler):
    def log_message(self, *a):
        pass

    def do_GET(self):
        if self.path == "/stream":
            self.send_response(200)
            self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
            self.send_header("Cache-Control", "no-cache")
            self.end_headers()
            try:
                while not stop.is_set():
                    jpg = fertig_jpeg
                    if jpg:
                        self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\n"
                                         + f"Content-Length: {len(jpg)}\r\n\r\n".encode()
                                         + jpg + b"\r\n")
                    time.sleep(0.05)
            except (BrokenPipeError, ConnectionResetError):
                pass
        else:
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.end_headers()
            self.wfile.write(SEITE)


def main():
    p = argparse.ArgumentParser(description="Go2 Tuer-/Fenstererkennung als Browser-Stream")
    p.add_argument("interface", nargs="?", default="eth0", help="Netzwerk-Interface zum Go2 (meist eth0) oder 'auto'")
    p.add_argument("--test", action="store_true", help="USB-Kamera am Pi statt Hundekamera")
    p.add_argument("--model", default="best.pt")
    p.add_argument("--conf", type=float, default=0.35)
    p.add_argument("--frames", type=int, default=3)
    p.add_argument("--imgsz", type=int, default=640, help="kleiner (z. B. 416) = schneller")
    p.add_argument("--port", type=int, default=8080)
    args = p.parse_args()

    quelle = (lambda: kamera_usb(0)) if args.test else (lambda: kamera_go2(args.interface))
    threading.Thread(target=quelle, daemon=True).start()
    threading.Thread(target=erkennung, args=(args,), daemon=True).start()

    server = ThreadingHTTPServer(("0.0.0.0", args.port), Handler)
    print(f"Stream läuft:  http://100.121.161.1:{args.port}   (Strg+C zum Beenden)")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        stop.set()
        server.shutdown()


if __name__ == "__main__":
    main()
