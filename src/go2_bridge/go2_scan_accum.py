#!/usr/bin/env python3
"""
Dichter 2D-Scan fuer SLAM/Nav2 aus dem duennen Go2-L1-LiDAR.

Problem: Eine einzelne L1-Wolke hat nur wenige Punkte -> /scan fast leer ->
         slam_toolbox findet keine Waende -> Karte verschmiert ("Farbeimer").
Loesung: Wolken der letzten `window` Sekunden aus /utlidar/cloud_deskewed
         (Frame odom, d.h. weltfest) sammeln, in base_link umrechnen,
         Hoehenband ausschneiden und als einen dichten LaserScan auf /scan senden.

Fixes fuer Nav2 (Lokalisierung ging beim Drehen verloren):
  * use_pi_time: Hardware-Zeitstempel bleiben erhalten und werden nur um einen
    geglaetteten Uhren-Offset (Pi - Go2) verschoben. Der Offset wird wie in
    go2_odom_tf.py aus /utlidar/robot_odom berechnet -> Wolken und TF passen zeitlich
    exakt zusammen (kein Ankunfts-Jitter mehr).
  * Beim Drehen (|wz| > rot_rate oder Gierwinkel-Aenderung ueber den Puffer > rot_yaw)
    werden nur die Wolken der letzten `window_rot` Sekunden genutzt -> kein Verschmieren.
  * TF-Lookup zuerst zum Zeitpunkt der neuesten Wolke, sonst neuester verfuegbarer TF.
"""
import math
from collections import deque

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, qos_profile_sensor_data
from rclpy.time import Time
from rclpy.duration import Duration
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2, LaserScan
from sensor_msgs_py import point_cloud2
import tf2_ros


def quat_to_rot(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def quat_to_yaw(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def wrap(a):
    return (a + math.pi) % (2.0 * math.pi) - math.pi


class ClockOffset:
    """Geglaetteter Offset Pi-Uhr minus Go2-Uhr (ns). Gleiche Logik wie in go2_odom_tf.py."""

    def __init__(self, alpha=0.02, jump_ns=500_000_000):
        self.alpha = alpha
        self.jump_ns = jump_ns
        self.offset_ns = None

    def update(self, now_ns, stamp_ns):
        diff = now_ns - stamp_ns
        if self.offset_ns is None or abs(diff - self.offset_ns) > self.jump_ns:
            self.offset_ns = float(diff)
        else:
            self.offset_ns = (1.0 - self.alpha) * self.offset_ns + self.alpha * diff
        return int(self.offset_ns)


class ScanAccum(Node):
    def __init__(self):
        super().__init__("go2_scan_accum")
        p = self.declare_parameter
        self.cloud_topic = p("cloud_topic", "/utlidar/cloud_deskewed").value
        self.odom_topic = p("odom_topic", "/utlidar/robot_odom").value
        self.fixed_frame = p("fixed_frame", "odom").value
        self.base_frame = p("base_frame", "base_link").value
        self.window = float(p("window", 0.30).value)          # Sekunden sammeln (geradeaus)
        self.window_rot = float(p("window_rot", 0.18).value)  # Sekunden sammeln beim Drehen
        self.rot_rate = float(p("rot_rate", 0.15).value)      # rad/s: ab hier gilt "dreht"
        self.rot_yaw = float(p("rot_yaw", 0.08).value)        # rad Gieraenderung ueber den Puffer
        self.min_h = float(p("min_height", -0.10).value)
        self.max_h = float(p("max_height", 0.40).value)
        self.r_min = float(p("range_min", 0.30).value)
        self.r_max = float(p("range_max", 15.0).value)
        self.inc = math.radians(float(p("angle_increment_deg", 1.0).value))
        self.rate = float(p("rate", 8.0).value)               # Scans pro Sekunde
        # True: Zeitstempel in die Pi-Uhr verschieben (passend zu go2_odom_tf mit use_pi_time:=true).
        self.use_pi_time = bool(p("use_pi_time", False).value)
        self.clock_off = ClockOffset(alpha=float(p("offset_alpha", 0.02).value))

        self.n = int(round(2 * math.pi / self.inc))
        self.buf = deque()          # (stamp_sec [Pi-Zeit], Nx3 Punkte im fixed_frame)
        self.last_stamp = None      # Stempel der neuesten Wolke (Pi-Zeit, builtin Time msg)
        self.yaw_hist = deque()     # (stamp_sec, yaw) aus der Odometrie
        self.wz = 0.0               # aktuelle Drehrate aus der Odometrie
        self.rotating = False

        self.tfbuf = tf2_ros.Buffer(cache_time=Duration(seconds=10))
        self.tfl = tf2_ros.TransformListener(self.tfbuf, self)

        qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST)
        self.create_subscription(PointCloud2, self.cloud_topic, self.on_cloud, qos)
        self.create_subscription(Odometry, self.odom_topic, self.on_odom, qos_profile_sensor_data)
        self.pub = self.create_publisher(LaserScan, "/scan", 10)
        self.create_timer(1.0 / self.rate, self.publish_scan)
        self.count = 0
        self.get_logger().info(
            f"Sammle {self.cloud_topic} ({self.window}s, beim Drehen {self.window_rot}s, "
            f"{math.degrees(self.inc):.1f} Grad) -> /scan, use_pi_time={self.use_pi_time}")

    # ---------- Zeit ----------
    def to_pi_ns(self, stamp):
        """Go2-Stempel -> Pi-Zeit (ns). None, solange noch kein Offset bekannt ist."""
        ns = Time.from_msg(stamp).nanoseconds
        if not self.use_pi_time:
            return ns
        if self.clock_off.offset_ns is None:
            return None
        return ns + int(self.clock_off.offset_ns)

    # ---------- Odometrie: Uhren-Offset + Drehrate ----------
    def on_odom(self, msg: Odometry):
        if self.use_pi_time:
            self.clock_off.update(self.get_clock().now().nanoseconds,
                                  Time.from_msg(msg.header.stamp).nanoseconds)
        t_ns = self.to_pi_ns(msg.header.stamp)
        t = t_ns * 1e-9
        self.wz = msg.twist.twist.angular.z
        self.yaw_hist.append((t, quat_to_yaw(msg.pose.pose.orientation)))
        while self.yaw_hist and t - self.yaw_hist[0][0] > 2.0:
            self.yaw_hist.popleft()

    def yaw_at(self, t):
        """Gierwinkel zum Zeitpunkt t (naechster Odometrie-Wert)."""
        if not self.yaw_hist:
            return None
        return min(self.yaw_hist, key=lambda e: abs(e[0] - t))[1]

    # ---------- Wolken sammeln ----------
    def on_cloud(self, msg):
        if msg.header.frame_id and msg.header.frame_id != self.fixed_frame:
            self.get_logger().warn(
                f"Wolke hat Frame '{msg.header.frame_id}', erwartet '{self.fixed_frame}'",
                throttle_duration_sec=5.0)
        t_ns = self.to_pi_ns(msg.header.stamp)
        if t_ns is None:
            self.get_logger().warn(f"Noch keine Odometrie auf {self.odom_topic} - Uhren-Offset unbekannt",
                                   throttle_duration_sec=5.0)
            return
        pts = point_cloud2.read_points_numpy(msg, field_names=("x", "y", "z"), skip_nans=True)
        if pts.size == 0:
            return
        pts = pts.astype(np.float32)
        self.buf.append((t_ns * 1e-9, pts))
        self.last_stamp = Time(nanoseconds=t_ns).to_msg()
        newest = self.buf[-1][0]
        keep = max(self.window, self.window_rot)
        while self.buf and newest - self.buf[0][0] > keep:
            self.buf.popleft()

    def select_clouds(self):
        """Beim Drehen nur die juengsten Wolken nehmen, sonst das volle Fenster."""
        newest = self.buf[-1][0]
        rotating = abs(self.wz) > self.rot_rate
        if not rotating and self.yaw_hist:
            y_old = self.yaw_at(self.buf[0][0])
            y_new = self.yaw_hist[-1][1]
            if y_old is not None and abs(wrap(y_new - y_old)) > self.rot_yaw:
                rotating = True
        if rotating != self.rotating:
            self.rotating = rotating
            self.get_logger().info(
                f"{'Drehung erkannt' if rotating else 'Geradeaus'} -> Fenster "
                f"{self.window_rot if rotating else self.window} s",
                throttle_duration_sec=2.0)
        win = self.window_rot if rotating else self.window
        return [b[1] for b in self.buf if newest - b[0] <= win]

    # ---------- Scan erzeugen ----------
    def publish_scan(self):
        if not self.buf or self.last_stamp is None:
            return
        try:
            tf = self.tfbuf.lookup_transform(self.base_frame, self.fixed_frame,
                                             Time.from_msg(self.last_stamp),
                                             timeout=Duration(seconds=0.03))
        except (tf2_ros.LookupException, tf2_ros.ExtrapolationException,
                tf2_ros.ConnectivityException):
            try:
                # Fallback: neuester verfuegbarer TF
                tf = self.tfbuf.lookup_transform(self.base_frame, self.fixed_frame, Time())
            except Exception as e:
                self.get_logger().warn(f"Kein TF {self.fixed_frame}->{self.base_frame}: {e}",
                                       throttle_duration_sec=5.0)
                return
        q = tf.transform.rotation
        tr = tf.transform.translation
        R = quat_to_rot(q.x, q.y, q.z, q.w)
        T = np.array([tr.x, tr.y, tr.z])

        clouds = self.select_clouds()
        if not clouds:
            return
        P = np.vstack(clouds) @ R.T + T
        z = P[:, 2]
        P = P[(z >= self.min_h) & (z <= self.max_h)]
        r = np.hypot(P[:, 0], P[:, 1])
        keep = (r >= self.r_min) & (r <= self.r_max)
        P, r = P[keep], r[keep]

        ranges = np.full(self.n, np.inf, dtype=np.float32)
        if r.size:
            a = np.arctan2(P[:, 1], P[:, 0])
            idx = ((a + math.pi) / self.inc).astype(np.int64) % self.n
            np.minimum.at(ranges, idx, r.astype(np.float32))

        s = LaserScan()
        # Stempel = Zeitpunkt des TF, mit dem tatsaechlich umgerechnet wurde
        s.header.stamp = tf.header.stamp
        s.header.frame_id = self.base_frame
        s.angle_min = -math.pi
        s.angle_increment = self.inc
        s.angle_max = -math.pi + self.inc * (self.n - 1)
        s.time_increment = 0.0
        s.scan_time = 1.0 / self.rate
        s.range_min = self.r_min
        s.range_max = self.r_max
        s.ranges = ranges.tolist()
        self.pub.publish(s)

        self.count += 1
        if self.count == 1 or self.count % 80 == 0:
            filled = int(np.isfinite(ranges).sum())
            self.get_logger().info(f"/scan: {filled}/{self.n} Strahlen mit Treffer "
                                   f"({len(clouds)} Wolken)")


def main():
    rclpy.init()
    node = ScanAccum()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
