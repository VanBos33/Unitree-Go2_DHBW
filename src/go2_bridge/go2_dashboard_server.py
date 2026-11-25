#!/usr/bin/env python3
"""
Dashboard-Server für den Go2 (läuft AUF DEM PI, Tab 9:Dashboard in go2)
-----------------------------------------------------------------------
Liefert die Dashboard-Seite aus und verbindet sie mit ROS 2, ohne dass der Browser ROS können muss:

  Browser (Mac)  --HTTP/Server-Sent-Events, Port 8082-->  dieser Server  --ROS 2-->  Nav2, Mission, Hund
                 <--Kamera direkt vom Pi, Port 8080 (go2_tuer_stream.py)

Was der Server liest:  /map, TF map->base_link, /plan, /scan, /amcl_pose, Nav2-Feedback,
                       Mission (HTTP 8081), Kamera-Status (HTTP 8080), CPU/RAM/Temperatur
Was er schickt:        /mission_goal (Ziel), /cmd_vel (nur beim Steuern mit WASD), /initialpose,
                       Nav2-Abbruch, Mission /sitz /steh /stopp /fahre, StopMove an den Sport-Sender

Sicherheit
  - Steuern mit der Tastatur: der Browser schickt 10x pro Sekunde einen Befehl. Kommt 0,5 s nichts
    mehr (Taste los, Tab weg, WLAN weg), bleibt der Hund stehen (Totmann).
  - Steuern bricht eine laufende Nav2-Fahrt ab. Sitzt der Hund, wird nicht gelaufen.
  - NOT-AUS: Fahrt abbrechen, StopMove, solange aktiv nur Stillstand auf /cmd_vel, keine neuen Ziele.
  - Foxglove bleibt voll nutzbar; der Server schreibt nur auf /cmd_vel, wenn im Dashboard gesteuert
    wird oder der Not-Aus aktiv ist.

Start (mit go2_env.sh, nach Nav2 und Mission):
    python3 ~/unitree_ws/src/go2_bridge/go2_dashboard_server.py
Ohne Pi/Hund testen (am Mac oder Pi, braucht kein ROS): simulierter Hund auf der Karte raum3
    python3 go2_dashboard_server.py --simulation
Dann im Browser:  http://100.121.161.1:8082   (Simulation lokal: http://localhost:8082)

HTTP-Schnittstelle (alles JSON)
  GET  /                    Dashboard-Seite (dashboard/go2_dashboard.html)
  GET  /api/stream          Server-Sent-Events, 10x pro Sekunde der Zustand
  GET  /api/state           Zustand einmalig
  GET  /api/map             Karte {version, breite, hoehe, aufloesung, ursprung, daten(base64)}
  GET  /api/ping            {"ok": true}
  POST /api/ziel            {"x", "y", "yaw"}         Ziel (Frame map) -> Mission
  POST /api/fahre           {"name"}                  gespeicherten Punkt anfahren
  POST /api/fahren          {"vx", "wz"}              Tastatursteuerung (Totmann 0,5 s)
  POST /api/abbruch                                   Fahrt abbrechen, Hund bleibt stehen
  POST /api/notaus          {"an": true/false}
  POST /api/sitz | /api/steh
  POST /api/punkt           {"name"}                  aktuelle Position als Punkt speichern
  POST /api/punkt_loeschen  {"name"}
  POST /api/startpose       {"x", "y", "yaw"}         /initialpose für AMCL
"""
import argparse
import base64
import json
import math
import os
import random
import socket
import subprocess
import sys
import threading
import time
import urllib.request
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlparse

HIER = os.path.dirname(os.path.abspath(__file__))
SPORT_PORT = 9870
FAHR_STATES = ("NAVIGATING", "ALIGNING", "SETTLING")
SITZ_STATES = ("SITTING", "SITTING_DOWN", "STANDING_UP")
MAX_VX, MAX_RUECK, MAX_WZ = 0.35, 0.15, 0.40      # wie go2_cmd_vel.py
TOTMANN = 0.5


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def http_json(url, timeout=1.5):
    with urllib.request.urlopen(url, timeout=timeout) as r:
        return json.loads(r.read() or b"{}")


# =============================================================================== Karte lesen
def lade_karte_datei(yaml_pfad):
    """raum3.yaml + .pgm ohne Zusatzpakete lesen -> dict wie /map (Werte 0..100, 255 = unbekannt)."""
    info = {}
    with open(yaml_pfad) as f:
        for zeile in f:
            if ":" in zeile:
                k, v = zeile.split(":", 1)
                info[k.strip()] = v.strip()
    pgm = os.path.join(os.path.dirname(yaml_pfad), info["image"])
    with open(pgm, "rb") as f:
        roh = f.read()
    teile, pos = [], 0
    while len(teile) < 4:                                   # Kopf: P5 breite hoehe maxwert
        while roh[pos:pos + 1].isspace():
            pos += 1
        if roh[pos:pos + 1] == b"#":
            pos = roh.index(b"\n", pos) + 1
            continue
        ende = pos
        while not roh[ende:ende + 1].isspace():
            ende += 1
        teile.append(roh[pos:ende])
        pos = ende
    pos += 1
    b, h = int(teile[1]), int(teile[2])
    pixel = roh[pos:pos + b * h]
    occ, free = float(info.get("occupied_thresh", 0.65)), float(info.get("free_thresh", 0.196))
    daten = bytearray(b * h)
    for zeile in range(h):                                  # PGM: oben zuerst, /map: unten zuerst
        for sp in range(b):
            p = pixel[zeile * b + sp]
            wahrsch = (255 - p) / 255.0
            v = 100 if wahrsch > occ else 0 if wahrsch < free else 255
            daten[(h - 1 - zeile) * b + sp] = v
    ursprung = [float(x) for x in info["origin"].strip("[] ").split(",")]
    return {"breite": b, "hoehe": h, "aufloesung": float(info["resolution"]),
            "ursprung": ursprung[:3], "daten": bytes(daten)}


# =============================================================================== System-Werte
class System:
    def __init__(self):
        self.werte = {}
        self._cpu_alt = None
        self._throttle_zeit = 0
        threading.Thread(target=self.schleife, daemon=True).start()

    def schleife(self):
        while True:
            try:
                self.messen()
            except Exception:
                pass
            time.sleep(1.0)

    def messen(self):
        w = {}
        if os.path.exists("/proc/stat"):
            with open("/proc/stat") as f:
                z = [int(x) for x in f.readline().split()[1:]]
            leer, gesamt = z[3] + z[4], sum(z)
            if self._cpu_alt:
                dl, dg = leer - self._cpu_alt[0], gesamt - self._cpu_alt[1]
                w["cpu"] = round(100 * (1 - dl / max(dg, 1)), 1)
            self._cpu_alt = (leer, gesamt)
            mem = {}
            with open("/proc/meminfo") as f:
                for zeile in f:
                    k, v = zeile.split(":")
                    mem[k] = int(v.split()[0])
            w["ram_gb"] = round((mem["MemTotal"] - mem["MemAvailable"]) / 1048576, 2)
            w["ram_max_gb"] = round(mem["MemTotal"] / 1048576, 1)
            w["last"] = os.getloadavg()[0]
        try:
            with open("/sys/class/thermal/thermal_zone0/temp") as f:
                w["temp"] = round(int(f.read()) / 1000, 1)
        except Exception:
            pass
        if time.time() - self._throttle_zeit > 10:
            self._throttle_zeit = time.time()
            try:
                out = subprocess.run(["vcgencmd", "get_throttled"], capture_output=True, text=True, timeout=2).stdout
                bits = int(out.strip().split("=")[1], 16)
                self.werte["drossel"] = {"jetzt": bool(bits & 0x4), "unterspannung": bool(bits & 0x1),
                                         "zu_heiss": bool(bits & 0x8), "seit_start": bool(bits & 0x70000)}
            except Exception:
                pass
        w["drossel"] = self.werte.get("drossel")
        w["programme"] = self.programme()
        self.werte = w

    MUSTER = {"Bridge": "go2_bridge.launch", "Sport": "go2_sport_sender", "cmd_vel": "go2_cmd_vel.py",
              "Nav2": "go2_nav2.launch", "Foxglove": "foxglove_bridge", "Mission": "go2_mission.py",
              "Kamera": "go2_tuer_stream"}

    def programme(self):
        if not os.path.isdir("/proc/1"):
            return None
        zeilen = []
        for pid in os.listdir("/proc"):
            if pid.isdigit():
                try:
                    with open(f"/proc/{pid}/cmdline", "rb") as f:
                        zeilen.append(f.read().replace(b"\0", b" ").decode(errors="ignore"))
                except Exception:
                    pass
        return {name: any(m in z for z in zeilen) for name, m in self.MUSTER.items()}


# =============================================================================== echte Daten (ROS 2)
class RosQuelle:
    """Alles, was mit ROS zu tun hat. Läuft in einem eigenen Thread (rclpy.spin)."""

    def __init__(self, args):
        import rclpy
        import numpy as np
        from rclpy.node import Node
        from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, qos_profile_sensor_data
        from rclpy.time import Time
        from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
        from nav_msgs.msg import OccupancyGrid, Path
        from sensor_msgs.msg import LaserScan
        from action_msgs.srv import CancelGoal
        from nav2_msgs.action import NavigateToPose
        import tf2_ros
        self.np, self.Time = np, Time
        self.PoseStamped, self.PoseWithCovarianceStamped, self.Twist = PoseStamped, PoseWithCovarianceStamped, Twist
        self.CancelGoal = CancelGoal
        self.mission_url = args.mission
        self.kamera_url = args.kamera
        self.simulation = False

        rclpy.init()
        self.node = n = Node("go2_dashboard")
        self.lock = threading.Lock()
        self.karte, self.karte_version = None, 0
        self.pose, self.pose_quelle, self.pose_zeit = None, None, 0.0
        self.v = [0.0, 0.0]
        self.plan, self.plan_id = [], 0
        self.scan_basis, self.scan_id, self.scan_zeit, self.scan_hz = None, 0, 0.0, 0.0
        self.amcl_std, self.feedback, self.feedback_zeit = None, None, 0.0

        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        n.create_subscription(OccupancyGrid, "/map", self.on_map, latched)
        n.create_subscription(Path, "/plan", self.on_plan, 5)
        n.create_subscription(LaserScan, "/scan", self.on_scan, qos_profile_sensor_data)
        n.create_subscription(PoseWithCovarianceStamped, "/amcl_pose", self.on_amcl, 5)
        n.create_subscription(NavigateToPose.Impl.FeedbackMessage, "/navigate_to_pose/_action/feedback",
                              self.on_feedback, 5)
        self.pub_ziel = n.create_publisher(PoseStamped, "/mission_goal", 10)
        self.pub_cmd = n.create_publisher(Twist, "/cmd_vel", 10)
        self.pub_start = n.create_publisher(PoseWithCovarianceStamped, "/initialpose", 10)
        self.cancel = n.create_client(CancelGoal, "/navigate_to_pose/_action/cancel_goal")
        self.tfbuf = tf2_ros.Buffer()
        self.tfl = tf2_ros.TransformListener(self.tfbuf, n)
        n.create_timer(0.1, self.tf_lesen)
        self.udp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        threading.Thread(target=rclpy.spin, args=(n,), daemon=True).start()
        n.get_logger().info("Dashboard-Server: ROS verbunden")

    # ---------- ROS-Eingänge ----------
    def on_map(self, m):
        np = self.np
        d = np.array(m.data, dtype=np.int16)
        d[d < 0] = 255
        o = m.info.origin
        with self.lock:
            self.karte = {"breite": m.info.width, "hoehe": m.info.height, "aufloesung": m.info.resolution,
                          "ursprung": [o.position.x, o.position.y, yaw_of(o.orientation)],
                          "daten": d.astype(np.uint8).tobytes()}
            self.karte_version += 1
        self.node.get_logger().info(f"Karte empfangen: {m.info.width}x{m.info.height} @ {m.info.resolution} m")

    def on_plan(self, msg):
        pts, letzter = [], None
        for ps in msg.poses:
            p = (round(ps.pose.position.x, 2), round(ps.pose.position.y, 2))
            if letzter is None or math.hypot(p[0] - letzter[0], p[1] - letzter[1]) >= 0.10:
                pts.append(p)
                letzter = p
        if msg.poses:
            e = msg.poses[-1].pose.position
            pts.append((round(e.x, 2), round(e.y, 2)))
        with self.lock:
            self.plan = pts[:300]
            self.plan_id += 1

    def on_scan(self, s):
        jetzt = time.time()
        if self.scan_zeit:
            self.scan_hz = 0.8 * self.scan_hz + 0.2 / max(jetzt - self.scan_zeit, 1e-3)
        if jetzt - self.scan_zeit < 0.25:          # max. 4x pro Sekunde verarbeiten
            return
        self.scan_zeit = jetzt
        np = self.np
        r = np.asarray(s.ranges, dtype=np.float32)
        a = s.angle_min + np.arange(len(r)) * s.angle_increment
        schritt = max(1, len(r) // 180)
        r, a = r[::schritt], a[::schritt]
        ok = np.isfinite(r) & (r > s.range_min) & (r < min(s.range_max, 12.0))
        with self.lock:
            self.scan_basis = (r[ok] * np.cos(a[ok]), r[ok] * np.sin(a[ok]))
            self.scan_id += 1

    def on_amcl(self, m):
        c = m.pose.covariance
        self.amcl_std = round(math.sqrt(max(c[0], 0) + max(c[7], 0)), 3)

    def on_feedback(self, m):
        f = m.feedback
        self.feedback = {"rest_m": round(f.distance_remaining, 2),
                         "eta_s": round(f.estimated_time_remaining.sec + f.estimated_time_remaining.nanosec * 1e-9, 1),
                         "recoveries": int(f.number_of_recoveries)}
        self.feedback_zeit = time.time()

    def tf_lesen(self):
        for ziel, quelle in (("map", "map"), ("odom", "odom")):
            try:
                tf = self.tfbuf.lookup_transform(ziel, "base_link", self.Time())
            except Exception:
                continue
            t = tf.transform
            neu = (t.translation.x, t.translation.y, yaw_of(t.rotation))
            jetzt = time.time()
            with self.lock:
                if self.pose and self.pose_quelle == quelle and jetzt - self.pose_zeit < 0.5:
                    dt = max(jetzt - self.pose_zeit, 1e-3)
                    dx, dy = neu[0] - self.pose[0], neu[1] - self.pose[1]
                    vx = (dx * math.cos(neu[2]) + dy * math.sin(neu[2])) / dt
                    wz = wrap(neu[2] - self.pose[2]) / dt
                    self.v = [0.7 * self.v[0] + 0.3 * vx, 0.7 * self.v[1] + 0.3 * wz]
                self.pose, self.pose_quelle, self.pose_zeit = neu, quelle, jetzt
            return

    # ---------- Daten für den Browser ----------
    def daten(self):
        with self.lock:
            pose, quelle, alter = self.pose, self.pose_quelle, time.time() - self.pose_zeit
            scan = None
            if self.scan_basis is not None and pose:
                bx, by = self.scan_basis
                c, s = math.cos(pose[2]), math.sin(pose[2])
                xs, ys = pose[0] + bx * c - by * s, pose[1] + bx * s + by * c
                scan = [round(float(v), 2) for xy in zip(xs, ys) for v in xy]
            fb = self.feedback if time.time() - self.feedback_zeit < 1.5 else None
            return {"pose": None if pose is None or alter > 2 else
                    {"x": round(pose[0], 3), "y": round(pose[1], 3), "yaw": round(pose[2], 4), "frame": quelle},
                    "v": [round(self.v[0], 2), round(self.v[1], 2)],
                    "plan": (self.plan_id, self.plan if fb else []),
                    "scan": (self.scan_id, scan),
                    "scan_hz": round(self.scan_hz, 1) if time.time() - self.scan_zeit < 2 else 0.0,
                    "amcl_std": self.amcl_std, "nav": fb, "karte_version": self.karte_version}

    def karte_holen(self):
        with self.lock:
            return self.karte, self.karte_version

    # ---------- Befehle ----------
    def mission(self, pfad):
        return http_json(self.mission_url + pfad)

    def ziel(self, x, y, yaw):
        m = self.PoseStamped()
        m.header.frame_id = "map"
        m.header.stamp = self.node.get_clock().now().to_msg()
        m.pose.position.x, m.pose.position.y = float(x), float(y)
        m.pose.orientation.z, m.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        self.pub_ziel.publish(m)

    def fahrbefehl(self, vx, wz):
        t = self.Twist()
        t.linear.x, t.angular.z = float(vx), float(wz)
        self.pub_cmd.publish(t)

    def nav_abbrechen(self):
        if self.cancel.service_is_ready():
            self.cancel.call_async(self.CancelGoal.Request())     # leere Anfrage = alle Ziele abbrechen

    def sport_stopp(self):
        for _ in range(3):
            self.udp.sendto(b'{"stop": true}', ("127.0.0.1", SPORT_PORT))

    def startpose(self, x, y, yaw):
        m = self.PoseWithCovarianceStamped()
        m.header.frame_id = "map"
        m.header.stamp = self.node.get_clock().now().to_msg()
        m.pose.pose.position.x, m.pose.pose.position.y = float(x), float(y)
        m.pose.pose.orientation.z, m.pose.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        cov = [0.0] * 36
        cov[0] = cov[7] = 0.25
        cov[35] = 0.07
        m.pose.covariance = cov
        self.pub_start.publish(m)

    def kamera_status(self):
        try:
            return http_json(self.kamera_url + "/status", timeout=1.0)
        except Exception:
            return None

    def punkte_laden(self):
        sys.path.insert(0, HIER)
        import go2_punkte
        return go2_punkte.laden()

    def punkt_speichern(self, name, x, y, yaw):
        sys.path.insert(0, HIER)
        import go2_punkte
        p = go2_punkte.laden()
        p[name] = {"x": round(x, 3), "y": round(y, 3), "yaw_deg": round(math.degrees(yaw), 1),
                   "gespeichert": time.strftime("%Y-%m-%d %H:%M")}
        go2_punkte.schreiben(p)

    def punkt_loeschen(self, name):
        sys.path.insert(0, HIER)
        import go2_punkte
        p = go2_punkte.laden()
        if p.pop(name, None) is not None:
            go2_punkte.schreiben(p)


# =============================================================================== simulierter Hund
class SimQuelle:
    """Gleiche Schnittstelle wie RosQuelle, aber ein simulierter Hund auf der echten Karte.
    Zum Testen des Dashboards ohne Pi und Hund (braucht weder ROS noch numpy)."""

    def __init__(self, args):
        self.simulation = True
        self.lock = threading.Lock()
        self.karte = lade_karte_datei(args.karte)
        self.karte_version = 1
        k = self.karte
        self.res, self.ox, self.oy = k["aufloesung"], k["ursprung"][0], k["ursprung"][1]
        frei = [(i % k["breite"], i // k["breite"]) for i, v in enumerate(k["daten"]) if v == 0]
        mx = sum(c[0] for c in frei) / len(frei)
        my = sum(c[1] for c in frei) / len(frei)
        sx, sy = min(frei, key=lambda c: (c[0] - mx) ** 2 + (c[1] - my) ** 2 + (0 if self.platz_frei_zelle(*c) else 1e9))
        self.x, self.y, self.th = self.ox + (sx + .5) * self.res, self.oy + (sy + .5) * self.res, 0.0
        self.v = [0.0, 0.0]
        self.cmd, self.cmd_zeit = (0.0, 0.0), 0.0
        self.state, self.t_state, self.ziel_name, self.fenster = "IDLE", time.time(), "", None
        self.target, self.nach_aufstehen = None, "nav"
        self.plan, self.plan_id = [], 0
        self.scan, self.scan_id = None, 0
        self.punkte = {}
        pfad = os.path.join(os.path.dirname(args.karte), "raum3_punkte.yaml")
        if os.path.exists(pfad):
            try:
                import yaml
                with open(pfad) as f:
                    self.punkte = yaml.safe_load(f) or {}
            except Exception:
                pass
        if not self.punkte:                               # Beispielpunkte für die Simulation
            for i, (dx, dy, yaw) in enumerate(((1.5, 0.5, 90), (-1.5, 0.8, 180), (0.5, -1.2, -90)), 1):
                px, py = self.frei_nahe(self.x + dx, self.y + dy)
                self.punkte[f"fenster_{i}"] = {"x": round(px, 2), "y": round(py, 2), "yaw_deg": yaw,
                                               "gespeichert": "Simulation"}
        threading.Thread(target=self.schleife, daemon=True).start()

    # ---------- Karte ----------
    def zelle(self, x, y):
        k = self.karte
        ix, iy = int((x - self.ox) / self.res), int((y - self.oy) / self.res)
        if 0 <= ix < k["breite"] and 0 <= iy < k["hoehe"]:
            return k["daten"][iy * k["breite"] + ix]
        return 255

    def platz_frei_zelle(self, ix, iy):
        return self.platz_frei(self.ox + (ix + .5) * self.res, self.oy + (iy + .5) * self.res)

    def platz_frei(self, x, y, r=0.25):
        for a in range(0, 360, 30):
            if self.zelle(x + r * math.cos(math.radians(a)), y + r * math.sin(math.radians(a))) == 100:
                return False
        return self.zelle(x, y) == 0

    def frei_nahe(self, x, y):
        for r in range(0, 60):
            for a in range(0, 360, 20):
                px, py = x + r * 0.05 * math.cos(math.radians(a)), y + r * 0.05 * math.sin(math.radians(a))
                if self.platz_frei(px, py, 0.3):
                    return px, py
        return x, y

    # ---------- Bewegung + Mission ----------
    def set_state(self, s):
        if s != self.state:
            print(f"[Sim] {self.state} -> {s}", flush=True)
        self.state, self.t_state = s, time.time()

    def schleife(self):
        letzte, scan_t = time.time(), 0
        while True:
            time.sleep(0.05)
            jetzt = time.time()
            dt, letzte = jetzt - letzte, jetzt
            with self.lock:
                vx, wz = self.schritt(jetzt)
                self.v = [0.7 * self.v[0] + 0.3 * vx, 0.7 * self.v[1] + 0.3 * wz]
                self.th = wrap(self.th + self.v[1] * dt)
                nx = self.x + self.v[0] * math.cos(self.th) * dt
                ny = self.y + self.v[0] * math.sin(self.th) * dt
                # Navigation fährt durch (echte Karten sind verrauscht, Nav2 plant drum herum);
                # beim Steuern von Hand bremsen Wände den simulierten Hund.
                if self.state in FAHR_STATES or self.zelle(nx, ny) != 100:
                    self.x, self.y = nx, ny
                else:
                    self.v[0] = 0.0
                if jetzt - scan_t > 0.25:
                    scan_t = jetzt
                    self.scan = self.scan_rechnen()
                    self.scan_id += 1

    def schritt(self, jetzt):
        dt_state = jetzt - self.t_state
        if jetzt - self.cmd_zeit < 0.3 and self.state not in SITZ_STATES:
            return self.cmd                                # Tastatur / Not-Aus (0, 0)
        if self.state == "NAVIGATING" and self.target:
            tx, ty, _ = self.target
            d = math.hypot(tx - self.x, ty - self.y)
            if d < 0.12:
                self.set_state("ALIGNING")
                return 0.0, 0.0
            err = wrap(math.atan2(ty - self.y, tx - self.x) - self.th)
            return (0.0 if abs(err) > 0.5 else min(0.35, 0.2 + d)), max(-0.6, min(0.6, 1.8 * err))
        if self.state == "ALIGNING":
            err = wrap(self.target[2] - self.th)
            if abs(err) < 0.08:
                self.set_state("SITTING_DOWN")
                return 0.0, 0.0
            return 0.0, max(-0.4, min(0.4, 1.5 * err))
        if self.state == "SITTING_DOWN" and dt_state > 1.5:
            self.set_state("SITTING")
        elif self.state == "SITTING" and self.fenster is None and dt_state > 3.0:
            self.fenster = {"zustand": random.choice(["ZU", "ZU", "OFFEN"]), "sicherheit": 0.87,
                            "ziel": self.ziel_name, "quelle": "simulation", "zeit": time.strftime("%H:%M:%S")}
        elif self.state == "STANDING_UP" and dt_state > 1.5:
            if self.nach_aufstehen == "nav" and self.target:
                self.set_state("NAVIGATING")
            else:
                self.set_state("IDLE")
        return 0.0, 0.0

    def scan_rechnen(self):
        pts = []
        for i in range(120):
            a = self.th + i * 2 * math.pi / 120
            ca, sa = math.cos(a), math.sin(a)
            r = 0.2
            while r < 8:
                if self.zelle(self.x + ca * r, self.y + sa * r) == 100:
                    pts += [round(self.x + ca * r, 2), round(self.y + sa * r, 2)]
                    break
                r += 0.05
        return pts

    def neues_ziel(self, x, y, yaw, name):
        self.target, self.ziel_name, self.fenster = (x, y, yaw), name, None
        self.plan, self.plan_id = [(round(self.x, 2), round(self.y, 2)), (round(x, 2), round(y, 2))], self.plan_id + 1
        self.nach_aufstehen = "nav"
        if self.state in ("SITTING", "SITTING_DOWN"):
            self.set_state("STANDING_UP")
        elif self.state not in ("STANDING_UP",):
            self.set_state("NAVIGATING")

    def name_fuer(self, x, y, standard="Ziel"):
        for n, p in self.punkte.items():
            if math.hypot(p["x"] - x, p["y"] - y) < 0.5:
                return n
        return standard

    # ---------- gleiche Schnittstelle wie RosQuelle ----------
    def daten(self):
        with self.lock:
            nav = None
            if self.state == "NAVIGATING" and self.target:
                d = math.hypot(self.target[0] - self.x, self.target[1] - self.y)
                nav = {"rest_m": round(d, 2), "eta_s": round(d / 0.3, 1), "recoveries": 0}
            return {"pose": {"x": round(self.x, 3), "y": round(self.y, 3), "yaw": round(self.th, 4), "frame": "map"},
                    "v": [round(self.v[0], 2), round(self.v[1], 2)],
                    "plan": (self.plan_id, self.plan if self.state == "NAVIGATING" else []),
                    "scan": (self.scan_id, self.scan), "scan_hz": 4.0, "amcl_std": 0.08,
                    "nav": nav, "karte_version": self.karte_version}

    def karte_holen(self):
        return self.karte, self.karte_version

    def mission(self, pfad):
        with self.lock:
            if pfad == "/state":
                return {"state": self.state, "ziel": self.ziel_name,
                        "seit": time.strftime("%H:%M:%S", time.localtime(self.t_state)), "fenster": self.fenster}
            if pfad == "/punkte":
                return dict(self.punkte)
            if pfad == "/sitz" and self.state not in ("SITTING", "SITTING_DOWN"):
                self.target, self.ziel_name, self.fenster = None, self.name_fuer(self.x, self.y, "von Hand"), None
                self.set_state("SITTING_DOWN")
            elif pfad == "/steh" and self.state == "SITTING":
                self.nach_aufstehen = "idle"
                self.set_state("STANDING_UP")
            elif pfad == "/stopp":
                if self.state in FAHR_STATES:
                    self.set_state("IDLE")
                elif self.state == "STANDING_UP":
                    self.nach_aufstehen = "idle"
            elif pfad.startswith("/fahre?name="):
                name = urllib.request.unquote(pfad.split("=", 1)[1])
                p = self.punkte.get(name)
                if p is None:
                    raise ValueError(f"Punkt '{name}' unbekannt")
                self.neues_ziel(p["x"], p["y"], math.radians(p["yaw_deg"]), name)
            return {"ok": True}

    def ziel(self, x, y, yaw):
        with self.lock:
            self.neues_ziel(x, y, yaw, self.name_fuer(x, y))

    def fahrbefehl(self, vx, wz):
        self.cmd, self.cmd_zeit = (vx, wz), time.time()

    def nav_abbrechen(self):
        with self.lock:
            if self.state in FAHR_STATES:
                self.set_state("IDLE")

    def sport_stopp(self):
        self.fahrbefehl(0.0, 0.0)

    def startpose(self, x, y, yaw):
        with self.lock:
            self.x, self.y, self.th = x, y, yaw

    def kamera_status(self):
        return {"fps": 14.8, "kb": 92, "simulation": True}

    def punkte_laden(self):
        with self.lock:
            return dict(self.punkte)

    def punkt_speichern(self, name, x, y, yaw):
        with self.lock:
            self.punkte[name] = {"x": round(x, 3), "y": round(y, 3), "yaw_deg": round(math.degrees(yaw), 1),
                                 "gespeichert": time.strftime("%Y-%m-%d %H:%M")}

    def punkt_loeschen(self, name):
        with self.lock:
            self.punkte.pop(name, None)


# =============================================================================== Steuerung + Zustand
class Dashboard:
    def __init__(self, quelle):
        self.q = quelle
        self.sys = System()
        self.notaus = False
        self.manuell, self.manuell_zeit, self.manuell_aktiv = (0.0, 0.0), 0.0, False
        self.mission_state, self.mission_fehler = {}, "verbinde ..."
        self.punkte, self.punkte_id = {}, 0
        self.fenster_liste, self.fenster_id, self._letztes_fenster = {}, 0, None
        self.kamera = None
        self.meldungen, self.meldung_id = [], 0
        self.zustand, self.zustand_lock = {}, threading.Lock()
        threading.Thread(target=self.regler, daemon=True).start()
        threading.Thread(target=self.langsam, daemon=True).start()
        threading.Thread(target=self.zustand_bauen, daemon=True).start()

    def melde(self, text, art="info"):
        self.meldung_id += 1
        self.meldungen = (self.meldungen + [{"id": self.meldung_id, "zeit": time.strftime("%H:%M:%S"),
                                             "text": text, "art": art}])[-30:]
        print(time.strftime("%H:%M:%S"), text, flush=True)

    # ---------- 10 Hz: Totmann, Not-Aus ----------
    def regler(self):
        nullen = 0
        while True:
            time.sleep(0.1)
            try:
                if self.notaus:
                    self.q.fahrbefehl(0.0, 0.0)
                elif time.time() - self.manuell_zeit < TOTMANN:
                    self.q.fahrbefehl(*self.manuell)
                    self.manuell_aktiv, nullen = True, 3
                elif nullen > 0:                          # nach dem Loslassen 3x Stillstand
                    self.q.fahrbefehl(0.0, 0.0)
                    nullen -= 1
                    if nullen == 0:
                        self.manuell_aktiv = False
            except Exception as e:
                print("Regler:", e, flush=True)

    # ---------- 1-2 Hz: Mission, Punkte, Kamera ----------
    def langsam(self):
        n = 0
        while True:
            try:
                self.mission_state, self.mission_fehler = self.q.mission("/state"), ""
            except Exception as e:
                self.mission_state, self.mission_fehler = {}, f"Mission nicht erreichbar ({e.__class__.__name__})"
            f = self.mission_state.get("fenster")
            if f and f != self._letztes_fenster:
                self._letztes_fenster = f
                self.fenster_liste[f.get("ziel") or "?"] = f
                self.fenster_id += 1
                self.melde(f"Fenster {f.get('ziel')}: {f.get('zustand')}", "w" if f.get("zustand") == "OFFEN" else "info")
            if n % 4 == 0:
                try:
                    p = self.q.punkte_laden()
                    if p != self.punkte:
                        self.punkte, self.punkte_id = p, self.punkte_id + 1
                except Exception:
                    pass
                self.kamera = self.q.kamera_status()
            n += 1
            time.sleep(0.5)

    def zustand_bauen(self):
        while True:
            try:
                d = self.q.daten()
                d.update(zeit=time.time(), simulation=self.q.simulation, notaus=self.notaus,
                         manuell=self.manuell_aktiv, mission=self.mission_state, mission_fehler=self.mission_fehler,
                         system=self.sys.werte, kamera=self.kamera,
                         punkte=(self.punkte_id, self.punkte), fenster=(self.fenster_id, self.fenster_liste),
                         meldungen=(self.meldung_id, self.meldungen[-10:]))
                with self.zustand_lock:
                    self.zustand = d
            except Exception as e:
                print("Zustand:", e, flush=True)
            time.sleep(0.1)

    # ---------- Befehle aus dem Browser ----------
    def pose(self):
        p = self.zustand.get("pose")
        return p if p and p.get("frame") == "map" else None

    def befehl(self, pfad, d):
        st = self.mission_state.get("state")
        if pfad == "/api/notaus":
            an = bool(d.get("an", True))
            if an and not self.notaus:
                self.notaus = True
                self.abbrechen()
                self.q.sport_stopp()
                self.melde("NOT-AUS ausgelöst", "e")
            elif not an and self.notaus:
                self.notaus = False
                self.melde("Not-Aus freigegeben")
            return {"ok": True, "notaus": self.notaus}
        if pfad == "/api/abbruch":
            self.abbrechen()
            self.melde("Fahrt abgebrochen")
            return {"ok": True}
        if self.notaus and pfad not in ("/api/punkt", "/api/punkt_loeschen", "/api/startpose"):
            return {"ok": False, "fehler": "Not-Aus ist aktiv – erst freigeben."}
        if pfad == "/api/fahren":
            vx = max(-MAX_RUECK, min(MAX_VX, float(d.get("vx", 0))))
            wz = max(-MAX_WZ, min(MAX_WZ, float(d.get("wz", 0))))
            if st in SITZ_STATES and (vx or wz):
                return {"ok": False, "fehler": "Der Hund sitzt – erst „Aufstehen“."}
            if (vx or wz) and not self.manuell_aktiv and st in FAHR_STATES:
                self.abbrechen()
                self.melde("Manuelle Übernahme – Navigation abgebrochen", "w")
            self.manuell, self.manuell_zeit = (vx, wz), time.time()
            return {"ok": True}
        if pfad == "/api/ziel":
            x, y, yaw = float(d["x"]), float(d["y"]), float(d.get("yaw", 0))
            self.q.ziel(x, y, yaw)
            self.melde(f"Ziel gesetzt ({x:.2f}, {y:.2f}, {math.degrees(yaw):.0f}°)")
            return {"ok": True}
        if pfad == "/api/fahre":
            name = str(d.get("name", ""))
            self.q.mission("/fahre?name=" + urllib.request.quote(name))
            self.melde(f"Fahre zu {name}")
            return {"ok": True}
        if pfad in ("/api/sitz", "/api/steh"):
            self.q.mission("/" + pfad.rsplit("/", 1)[1])
            self.melde("Hinsetzen" if pfad.endswith("sitz") else "Aufstehen")
            return {"ok": True}
        if pfad == "/api/punkt":
            name = "".join(c for c in str(d.get("name", "")).strip().replace(" ", "_") if c.isalnum() or c in "_-")
            if not name:
                return {"ok": False, "fehler": "Name fehlt"}
            p = self.pose()
            if p is None:
                return {"ok": False, "fehler": "Keine Position auf der Karte – Startpose gesetzt?"}
            self.q.punkt_speichern(name, p["x"], p["y"], p["yaw"])
            self.melde(f"Punkt „{name}“ gespeichert")
            self.punkte_id += 1
            self.punkte = self.q.punkte_laden()
            return {"ok": True}
        if pfad == "/api/punkt_loeschen":
            self.q.punkt_loeschen(str(d.get("name", "")))
            self.punkte, self.punkte_id = self.q.punkte_laden(), self.punkte_id + 1
            return {"ok": True}
        if pfad == "/api/startpose":
            self.q.startpose(float(d["x"]), float(d["y"]), float(d.get("yaw", 0)))
            self.melde("Startpose gesetzt – danach ggf. „go2 reset“")
            return {"ok": True}
        return None

    def abbrechen(self):
        try:
            self.q.mission("/stopp")
        except Exception:
            pass
        self.q.nav_abbrechen()
        self.manuell, self.manuell_zeit = (0.0, 0.0), 0.0


# =============================================================================== HTTP
def handler_fuer(db, html_pfad):
    class Handler(BaseHTTPRequestHandler):
        protocol_version = "HTTP/1.1"

        def log_message(self, *a):
            pass

        def kopf(self, code, art, laenge=None):
            self.send_response(code)
            self.send_header("Content-Type", art)
            self.send_header("Access-Control-Allow-Origin", "*")
            self.send_header("Cache-Control", "no-store")
            if laenge is not None:
                self.send_header("Content-Length", str(laenge))
            self.end_headers()

        def json(self, code, daten):
            body = json.dumps(daten, ensure_ascii=False, separators=(",", ":")).encode()
            self.kopf(code, "application/json; charset=utf-8", len(body))
            self.wfile.write(body)

        def do_OPTIONS(self):
            self.send_response(204)
            self.send_header("Access-Control-Allow-Origin", "*")
            self.send_header("Access-Control-Allow-Methods", "GET, POST, OPTIONS")
            self.send_header("Access-Control-Allow-Headers", "Content-Type")
            self.send_header("Content-Length", "0")
            self.end_headers()

        def do_GET(self):
            pfad = urlparse(self.path).path
            if pfad in ("/", "/index.html", "/go2_dashboard.html"):
                try:
                    with open(html_pfad, "rb") as f:
                        body = f.read()
                except FileNotFoundError:
                    self.json(404, {"fehler": f"{html_pfad} fehlt"})
                    return
                self.kopf(200, "text/html; charset=utf-8", len(body))
                self.wfile.write(body)
            elif pfad == "/api/ping":
                self.json(200, {"ok": True, "zeit": time.time()})
            elif pfad == "/api/state":
                self.json(200, self.ohne_ids(dict(db.zustand)))
            elif pfad == "/api/map":
                k, version = db.q.karte_holen()
                if k is None:
                    self.json(503, {"fehler": "Noch keine Karte (/map) – läuft Nav2?"})
                    return
                self.json(200, {**{a: b for a, b in k.items() if a != "daten"}, "version": version,
                                "daten": base64.b64encode(k["daten"]).decode()})
            elif pfad == "/api/stream":
                self.stream()
            else:
                self.json(404, {"fehler": "unbekannter Pfad"})

        def do_POST(self):
            pfad = urlparse(self.path).path
            try:
                n = int(self.headers.get("Content-Length", 0))
                d = json.loads(self.rfile.read(n) or b"{}") if n else {}
                antwort = db.befehl(pfad, d)
            except Exception as e:
                self.json(400, {"ok": False, "fehler": f"{e.__class__.__name__}: {e}"})
                return
            if antwort is None:
                self.json(404, {"ok": False, "fehler": "unbekannter Befehl"})
            else:
                self.json(200 if antwort.get("ok") else 409, antwort)

        @staticmethod
        def ohne_ids(z):
            for k in ("plan", "scan", "punkte", "fenster", "meldungen"):
                if isinstance(z.get(k), tuple):
                    z[k] = z[k][1]
            return z

        def stream(self):
            """Server-Sent-Events: 10x pro Sekunde der Zustand. Große Teile nur, wenn sie sich geändert haben."""
            self.send_response(200)
            self.send_header("Content-Type", "text/event-stream")
            self.send_header("Cache-Control", "no-store")
            self.send_header("Access-Control-Allow-Origin", "*")
            self.send_header("X-Accel-Buffering", "no")
            self.end_headers()
            gesendet = {}
            try:
                while True:
                    z = dict(db.zustand)
                    for k in ("plan", "scan", "punkte", "fenster", "meldungen"):
                        if isinstance(z.get(k), tuple):
                            nr, wert = z[k]
                            if gesendet.get(k) == nr:
                                z.pop(k)
                            else:
                                gesendet[k], z[k] = nr, wert
                    daten = json.dumps(z, ensure_ascii=False, separators=(",", ":"))
                    self.wfile.write(f"data: {daten}\n\n".encode())
                    self.wfile.flush()
                    time.sleep(0.1)
            except (BrokenPipeError, ConnectionResetError, OSError):
                pass
    return Handler


def main():
    ap = argparse.ArgumentParser(description="Go2 Dashboard-Server")
    ap.add_argument("--port", type=int, default=8082)
    ap.add_argument("--simulation", action="store_true", help="simulierter Hund, kein ROS nötig")
    ap.add_argument("--karte", default=os.path.join(HIER, "maps", "raum3.yaml"), help="Karte für --simulation")
    ap.add_argument("--html", default=os.path.join(HIER, "dashboard", "go2_dashboard.html"))
    ap.add_argument("--mission", default="http://127.0.0.1:8081")
    ap.add_argument("--kamera", default="http://127.0.0.1:8080")
    args, _ = ap.parse_known_args()      # --ros-args usw. ignorieren

    quelle = SimQuelle(args) if args.simulation else RosQuelle(args)
    db = Dashboard(quelle)
    server = ThreadingHTTPServer(("0.0.0.0", args.port), handler_fuer(db, args.html))
    server.daemon_threads = True
    print(f"Dashboard bereit: http://100.121.161.1:{args.port}   (lokal: http://localhost:{args.port})"
          + ("   [SIMULATION]" if args.simulation else ""), flush=True)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        try:
            if db.manuell_aktiv or db.notaus:
                quelle.fahrbefehl(0.0, 0.0)
        except Exception:
            pass
        server.shutdown()
        print("Dashboard beendet.", flush=True)


if __name__ == "__main__":
    main()
