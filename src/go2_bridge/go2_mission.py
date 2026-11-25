#!/usr/bin/env python3
"""
Missions-Steuerung: Ziel auf der Karte -> hinlaufen -> ausrichten -> hinsetzen -> warten.

Ablauf
  1. In Foxglove mit "Publish pose" einen Punkt + Blickrichtung auf /mission_goal setzen.
  2. Sitzt der Hund gerade: aufstehen (über go2_sport_sender.py), kurz warten.
  3. Nav2 (NavigateToPose) fährt hin.
  4. Angekommen: auf der Stelle genau auf die gewünschte Blickrichtung drehen.
  5. Hinsetzen und sitzen bleiben, bis ein neues Ziel kommt.
  Ein neues Ziel unterbricht jederzeit (auch während der Fahrt).
  Scheitert Nav2, bleibt der Hund stehen und wartet auf ein neues Ziel.

Schnittstellen für Kamera-Fenster (Mac) und Dashboard
  HTTP (Port 8081, aus dem Tailscale-Netz erreichbar):
    GET  /state                -> {"state", "ziel", "seit", "fenster"}
    GET  /punkte               -> gespeicherte Punkte (maps/raum3_punkte.yaml)
    GET  /fahre?name=fenster_1 -> gespeicherten Punkt anfahren (wie ein Ziel aus Foxglove)
    GET  /sitz                 -> Hund dort hinsetzen, wo er gerade steht (Kamera-KI startet)
    GET  /steh                 -> Hund aufstehen lassen, ohne loszulaufen
    GET  /stopp                -> laufende Fahrt/Ausrichtung abbrechen, Hund bleibt stehen (Dashboard)
    POST /fenster              <- Ergebnis der Fenstererkennung {"zustand", "sicherheit", ...}
  ROS:
    /mission_state  (std_msgs/String, JSON)  Zustand + Zielname, bei jeder Änderung und 1x pro Sekunde
    /window_status  (std_msgs/String, JSON)  letztes Ergebnis der Fenstererkennung

Start (mit go2_env.sh, NACH Bridge, Sport-Sender, cmd_vel und Nav2):
  python3 ~/unitree_ws/src/go2_bridge/go2_mission.py
  python3 ~/unitree_ws/src/go2_bridge/go2_mission.py --ros-args -p sit_on_arrival:=false
"""
import json
import math
import os
import socket
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import parse_qs, urlparse

import yaml

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.time import Time
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import String
from nav2_msgs.action import NavigateToPose
import tf2_ros

PORT = 9870
HIER = os.path.dirname(os.path.abspath(__file__))


def punkt_zu_pose(p, frame):
    """{"x", "y", "yaw_deg"} -> PoseStamped"""
    ps = PoseStamped()
    ps.header.frame_id = frame
    ps.pose.position.x = float(p["x"])
    ps.pose.position.y = float(p["y"])
    yaw = math.radians(float(p["yaw_deg"]))
    ps.pose.orientation.z = math.sin(yaw / 2.0)
    ps.pose.orientation.w = math.cos(yaw / 2.0)
    return ps


def lade_punkte(datei):
    try:
        with open(datei) as f:
            return yaml.safe_load(f) or {}
    except FileNotFoundError:
        return {}


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def wrap(a):
    return (a + math.pi) % (2.0 * math.pi) - math.pi


class Mission(Node):
    def __init__(self):
        super().__init__("go2_mission")
        p = self.declare_parameter
        self.goal_topic = p("goal_topic", "/mission_goal").value
        self.map_frame = p("map_frame", "map").value
        self.base_frame = p("base_frame", "base_link").value
        self.sit_on_arrival = bool(p("sit_on_arrival", True).value)
        self.align_tol = float(p("align_tolerance", 0.10).value)      # rad (~6°)
        self.align_timeout = float(p("align_timeout", 15.0).value)    # s
        self.align_max = float(p("align_max_rate", 0.40).value)       # rad/s
        self.settle = float(p("stand_settle", 1.0).value)             # s nach dem Aufstehen
        self.ack_timeout = float(p("ack_timeout", 12.0).value)        # s
        self.http_port = int(p("http_port", 8081).value)
        self.punkte_datei = p("punkte_datei", os.path.join(HIER, "maps", "raum3_punkte.yaml")).value
        self.name_radius = float(p("name_radius", 0.5).value)         # m: Ziel nahe an Punkt -> dessen Name

        self.state = "IDLE"        # IDLE, NAVIGATING, ALIGNING, SITTING_DOWN, SITTING, STANDING_UP, SETTLING
        self.target = None         # PoseStamped (letztes Ziel)
        self.goal_waiting = False  # neues Ziel kam, während wir uns hingesetzt haben
        self.nav_seq = 0
        self.nav_handle = None
        self.t_state = time.time()
        self.align_flips = 0
        self.last_err_sign = 0
        self.ziel_name = ""        # Name des gespeicherten Punkts (oder "Ziel")
        self.fenster = None        # letztes Ergebnis der Fenstererkennung
        self.lock = threading.Lock()
        self.http_ziele = []       # über HTTP angeforderte Ziele (werden in tick() abgearbeitet)
        self.http_befehle = []     # "sitz" / "steh" von Hand
        self.nach_aufstehen = "nav"   # "nav" = danach zum Ziel laufen, "idle" = stehen bleiben

        self.udp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.udp.bind(("127.0.0.1", 0))
        self.udp.setblocking(False)

        self.tfbuf = tf2_ros.Buffer()
        self.tfl = tf2_ros.TransformListener(self.tfbuf, self)
        self.nav = ActionClient(self, NavigateToPose, "navigate_to_pose")
        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.state_pub = self.create_publisher(String, "/mission_state", 10)
        self.fenster_pub = self.create_publisher(String, "/window_status", 10)
        self.create_subscription(PoseStamped, self.goal_topic, self.on_goal, 10)
        self.create_timer(0.1, self.tick)
        self.create_timer(1.0, self.publish_state)
        self.starte_http()
        self.get_logger().info(
            f"Bereit. Ziele auf {self.goal_topic} (Foxglove 'Publish pose'). "
            f"Hinsetzen am Ziel: {'ja' if self.sit_on_arrival else 'nein'}. "
            f"HTTP-Status auf Port {self.http_port}")

    # ---------- Hilfen ----------
    def set_state(self, s):
        geaendert = s != self.state
        if geaendert:
            self.get_logger().info(f"[{self.state}] -> [{s}]")
        self.state = s
        self.t_state = time.time()
        if geaendert:
            self.publish_state()

    def state_dict(self):
        with self.lock:
            fenster = self.fenster
        return {"state": self.state, "ziel": self.ziel_name,
                "seit": time.strftime("%H:%M:%S", time.localtime(self.t_state)),
                "fenster": fenster}

    def publish_state(self):
        self.state_pub.publish(String(data=json.dumps(self.state_dict())))

    def name_fuer(self, msg, standard="Ziel"):
        """Name des gespeicherten Punkts, der dem Ziel am nächsten liegt (sonst standard)."""
        x, y = msg.pose.position.x, msg.pose.position.y
        bester, d_min = standard, self.name_radius
        for name, pkt in lade_punkte(self.punkte_datei).items():
            d = math.hypot(float(pkt["x"]) - x, float(pkt["y"]) - y)
            if d < d_min:
                bester, d_min = name, d
        return bester

    # ---------- HTTP (für Mac-Kamerafenster und Dashboard) ----------
    def starte_http(self):
        node = self

        class Handler(BaseHTTPRequestHandler):
            def log_message(self, *a):
                pass

            def antworte(self, code, daten):
                body = json.dumps(daten, ensure_ascii=False).encode()
                self.send_response(code)
                self.send_header("Content-Type", "application/json; charset=utf-8")
                self.send_header("Access-Control-Allow-Origin", "*")
                self.send_header("Content-Length", str(len(body)))
                self.end_headers()
                self.wfile.write(body)

            def do_GET(self):
                u = urlparse(self.path)
                if u.path == "/state":
                    self.antworte(200, node.state_dict())
                elif u.path == "/punkte":
                    self.antworte(200, lade_punkte(node.punkte_datei))
                elif u.path in ("/sitz", "/steh", "/stopp"):
                    with node.lock:
                        node.http_befehle.append(u.path[1:])
                    self.antworte(200, {"ok": True, "befehl": u.path[1:], "state": node.state})
                elif u.path == "/fahre":
                    name = parse_qs(u.query).get("name", [""])[0]
                    punkte = lade_punkte(node.punkte_datei)
                    if name not in punkte:
                        self.antworte(404, {"fehler": f"Punkt '{name}' unbekannt", "punkte": list(punkte)})
                        return
                    with node.lock:
                        node.http_ziele.append((name, punkte[name]))
                    self.antworte(200, {"ok": True, "fahre": name})
                else:
                    self.antworte(404, {"fehler": "unbekannter Pfad",
                                        "pfade": ["/state", "/punkte", "/fahre?name=...", "/sitz", "/steh", "/stopp",
                                                  "POST /fenster"]})

            def do_POST(self):
                if urlparse(self.path).path != "/fenster":
                    self.antworte(404, {"fehler": "unbekannter Pfad"})
                    return
                try:
                    n = int(self.headers.get("Content-Length", 0))
                    daten = json.loads(self.rfile.read(n) or b"{}")
                except Exception as e:
                    self.antworte(400, {"fehler": str(e)})
                    return
                daten.setdefault("ziel", node.ziel_name)
                daten.setdefault("zeit", time.strftime("%H:%M:%S"))
                with node.lock:
                    node.fenster = daten
                node.fenster_pub.publish(String(data=json.dumps(daten, ensure_ascii=False)))
                node.get_logger().info(
                    f"Fenster an {daten.get('ziel')}: {daten.get('zustand')} "
                    f"({float(daten.get('sicherheit', 0)):.0%})")
                self.antworte(200, {"ok": True})

            def do_OPTIONS(self):
                self.send_response(204)
                self.send_header("Access-Control-Allow-Origin", "*")
                self.send_header("Access-Control-Allow-Methods", "GET, POST, OPTIONS")
                self.send_header("Access-Control-Allow-Headers", "Content-Type")
                self.end_headers()

        try:
            self.http = ThreadingHTTPServer(("0.0.0.0", self.http_port), Handler)
        except OSError as e:
            self.get_logger().error(f"HTTP-Port {self.http_port} belegt ({e}) - läuft die Mission schon?")
            self.http = None
            return
        threading.Thread(target=self.http.serve_forever, daemon=True).start()

    def send_action(self, action):
        self.udp.sendto(json.dumps({"action": action}).encode(), ("127.0.0.1", PORT))

    def read_ack(self):
        try:
            data, _ = self.udp.recvfrom(1024)
            return json.loads(data)
        except BlockingIOError:
            return None

    def current_pose(self):
        try:
            tf = self.tfbuf.lookup_transform(self.map_frame, self.base_frame, Time())
        except Exception:
            return None
        ps = PoseStamped()
        ps.header.frame_id = self.map_frame
        ps.pose.position.x = tf.transform.translation.x
        ps.pose.position.y = tf.transform.translation.y
        ps.pose.orientation = tf.transform.rotation
        return ps

    def befehl_von_hand(self, befehl):
        if befehl == "stopp":
            self.goal_waiting = False
            if self.state in ("NAVIGATING", "ALIGNING", "SETTLING"):
                self.nav_seq += 1                  # Ergebnis des alten Ziels ignorieren
                if self.nav_handle is not None:
                    try:
                        self.nav_handle.cancel_goal_async()
                    except Exception:
                        pass
                self.turn(0.0)
                self.get_logger().info(f"Stopp von Hand (war {self.state})")
                self.set_state("IDLE")
            elif self.state == "STANDING_UP":
                self.nach_aufstehen = "idle"       # aufstehen, aber nicht mehr loslaufen
            return
        if befehl == "sitz":
            if self.state in ("SITTING", "SITTING_DOWN"):
                return
            if self.state in ("NAVIGATING", "ALIGNING") and self.nav_handle is not None:
                self.nav_seq += 1                  # laufendes Nav2-Ziel verwerfen
                try:
                    self.nav_handle.cancel_goal_async()
                except Exception:
                    pass
            self.turn(0.0)
            pose = self.current_pose()
            self.ziel_name = self.name_fuer(pose, "von Hand") if pose else "von Hand"
            with self.lock:
                self.fenster = None
            self.get_logger().info(f"Hinsetzen von Hand ({self.ziel_name})")
            self.send_action("sit")
            self.set_state("SITTING_DOWN")
        elif befehl == "steh":
            if self.state != "SITTING":
                self.get_logger().info(f"Aufstehen von Hand ignoriert (Zustand {self.state})")
                return
            self.get_logger().info("Aufstehen von Hand")
            self.nach_aufstehen = "idle"
            self.send_action("stand")
            self.set_state("STANDING_UP")

    def current_yaw(self):
        try:
            tf = self.tfbuf.lookup_transform(self.map_frame, self.base_frame, Time())
            return yaw_of(tf.transform.rotation)
        except Exception as e:
            self.get_logger().warn(f"Kein TF {self.map_frame}->{self.base_frame}: {e}",
                                   throttle_duration_sec=3.0)
            return None

    def turn(self, z):
        t = Twist()
        t.angular.z = float(z)
        self.cmd_pub.publish(t)

    # ---------- Ziel empfangen ----------
    def on_goal(self, msg: PoseStamped):
        if not msg.header.frame_id:
            msg.header.frame_id = self.map_frame
        self.target = msg
        if not getattr(self, "_name_gesetzt", False):
            self.ziel_name = self.name_fuer(msg)
        self._name_gesetzt = False
        with self.lock:
            self.fenster = None
        p, yaw = msg.pose.position, yaw_of(msg.pose.orientation)
        self.get_logger().info(
            f"Neues Ziel '{self.ziel_name}': x={p.x:.2f} y={p.y:.2f} Blick {math.degrees(yaw):.0f}° "
            f"(Frame {msg.header.frame_id})")
        if msg.header.frame_id != self.map_frame:
            self.get_logger().warn(f"Ziel ist im Frame '{msg.header.frame_id}', erwartet '{self.map_frame}'. "
                                   "In Foxglove den Display-Frame auf 'map' stellen!")

        self.nach_aufstehen = "nav"
        if self.state == "SITTING":
            self.send_action("stand")
            self.set_state("STANDING_UP")
        elif self.state == "SITTING_DOWN":
            self.goal_waiting = True         # erst fertig hinsetzen, dann aufstehen
        elif self.state in ("STANDING_UP", "SETTLING"):
            pass                             # läuft gleich zum neuesten Ziel los
        else:                                # IDLE, NAVIGATING, ALIGNING
            if self.state == "ALIGNING":
                self.turn(0.0)
            self.start_nav()

    # ---------- Nav2 ----------
    def start_nav(self):
        if self.target is None:
            self.set_state("IDLE")
            return
        if not self.nav.wait_for_server(timeout_sec=2.0):
            self.get_logger().error("Nav2 (navigate_to_pose) nicht erreichbar - läuft Nav2?")
            self.set_state("IDLE")
            return
        goal = NavigateToPose.Goal()
        goal.pose = self.target
        goal.pose.header.stamp = Time().to_msg()     # neuester TF
        self.nav_seq += 1
        seq = self.nav_seq
        self.set_state("NAVIGATING")
        fut = self.nav.send_goal_async(goal)
        fut.add_done_callback(lambda f: self.on_accepted(f, seq))

    def on_accepted(self, fut, seq):
        gh = fut.result()
        if seq != self.nav_seq:
            return
        if not gh.accepted:
            self.get_logger().error("Nav2 hat das Ziel abgelehnt.")
            self.set_state("IDLE")
            return
        self.nav_handle = gh
        gh.get_result_async().add_done_callback(lambda f: self.on_result(f, seq))

    def on_result(self, fut, seq):
        if seq != self.nav_seq:
            return                           # von neuerem Ziel abgelöst
        status = fut.result().status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info("Ziel erreicht.")
            if self.sit_on_arrival:
                self.align_flips = 0
                self.last_err_sign = 0
                self.set_state("ALIGNING")
            else:
                self.set_state("IDLE")
        else:
            self.get_logger().warn(f"Nav2 hat das Ziel nicht erreicht (Status {status}). "
                                   "Bleibe stehen und warte auf ein neues Ziel.")
            self.set_state("IDLE")

    # ---------- Zyklus (10 Hz) ----------
    def tick(self):
        with self.lock:
            ziele, self.http_ziele = self.http_ziele, []
        with self.lock:
            befehle, self.http_befehle = self.http_befehle, []
        for b in befehle:
            self.befehl_von_hand(b)
        for name, pkt in ziele[-1:]:          # nur das neueste
            self.ziel_name = name
            self._name_gesetzt = True
            self.on_goal(punkt_zu_pose(pkt, self.map_frame))
        ack = self.read_ack()
        dt = time.time() - self.t_state

        if self.state == "ALIGNING":
            self.do_align(dt)

        elif self.state == "SITTING_DOWN":
            if ack and ack.get("ack") == "sit":
                if ack.get("ok"):
                    self.get_logger().info("Sitzt. Warte auf neues Ziel.")
                    self.set_state("SITTING")
                    if self.goal_waiting:
                        self.goal_waiting = False
                        self.send_action("stand")
                        self.set_state("STANDING_UP")
                else:
                    self.get_logger().error(f"Hinsetzen fehlgeschlagen: {ack.get('info')}")
                    self.set_state("IDLE")
                    if self.goal_waiting:
                        self.goal_waiting = False
                        self.start_nav()
            elif dt > self.ack_timeout:
                self.get_logger().error("Keine Antwort vom Sport-Sender (sit). Läuft go2_sport_sender.py?")
                self.set_state("IDLE")

        elif self.state == "STANDING_UP":
            if ack and ack.get("ack") == "stand":
                if ack.get("ok"):
                    self.set_state("IDLE" if self.nach_aufstehen == "idle" else "SETTLING")
                else:
                    self.get_logger().error(f"Aufstehen fehlgeschlagen: {ack.get('info')} - bleibe sitzen.")
                    self.set_state("SITTING")
            elif dt > self.ack_timeout:
                self.get_logger().error("Keine Antwort vom Sport-Sender (stand). Läuft go2_sport_sender.py?")
                self.set_state("SITTING")

        elif self.state == "SETTLING":
            if dt > self.settle:
                self.start_nav()

    def do_align(self, dt):
        if dt < 0.5:
            return                               # Nav2 erst ausrollen lassen
        yaw = self.current_yaw()
        if yaw is None or self.target is None:
            return
        err = wrap(yaw_of(self.target.pose.orientation) - yaw)
        sign = 1 if err > 0 else -1
        if self.last_err_sign and sign != self.last_err_sign:
            self.align_flips += 1                # über das Ziel hinausgedreht
        self.last_err_sign = sign
        done = abs(err) < self.align_tol or self.align_flips >= 3
        if done or dt > self.align_timeout:
            if not done:
                self.get_logger().warn(f"Ausrichten: Zeit abgelaufen, Restfehler {math.degrees(err):.0f}°")
            else:
                self.get_logger().info(f"Ausgerichtet (Restfehler {math.degrees(err):.0f}°). Setze mich hin.")
            for _ in range(3):
                self.turn(0.0)
            self.send_action("sit")
            self.set_state("SITTING_DOWN")
            return
        # go2_cmd_vel.py hebt kleine Werte ohnehin auf die Mindestdrehrate (0,30) an
        self.turn(max(-self.align_max, min(self.align_max, 1.5 * err)))


def main():
    rclpy.init()
    node = Mission()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.turn(0.0)
        except Exception:
            pass                     # nach Strg+C ist der ROS-Kontext schon zu
        if node.http:
            node.http.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
