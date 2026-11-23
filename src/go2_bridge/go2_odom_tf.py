#!/usr/bin/env python3
"""
Macht aus der Go2-Odometrie (/utlidar/robot_odom) den TF  odom -> base_link.

use_pi_time:=true
  Die Uhr des Hundes weicht von der Pi-Uhr ab (teils > 1 min). Frueher wurde jeder
  Zeitstempel durch die Ankunftszeit ersetzt -> Netzwerk-Jitter landete im TF und
  Odometrie und Punktwolken passten beim Drehen nicht mehr zusammen.
  Jetzt: Hardware-Zeitstempel behalten und nur um einen geglaetteten Uhren-Offset
  verschieben (EMA). Die relativen Zeiten zwischen Odometrie und Wolken bleiben exakt.
  go2_scan_accum.py rechnet den Offset mit derselben Methode aus derselben Odometrie.
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster


class ClockOffset:
    """Geglaetteter Offset Pi-Uhr minus Go2-Uhr (ns). Gleiche Logik wie in go2_scan_accum.py."""

    def __init__(self, alpha=0.02, jump_ns=500_000_000):
        self.alpha = alpha
        self.jump_ns = jump_ns      # > 0,5 s Abweichung: Uhr des Hundes wurde verstellt -> neu setzen
        self.offset_ns = None

    def update(self, now_ns, stamp_ns):
        diff = now_ns - stamp_ns
        if self.offset_ns is None or abs(diff - self.offset_ns) > self.jump_ns:
            self.offset_ns = float(diff)
        else:
            self.offset_ns = (1.0 - self.alpha) * self.offset_ns + self.alpha * diff
        return int(self.offset_ns)


class Go2OdomTF(Node):
    def __init__(self):
        super().__init__("go2_odom_tf")
        self.declare_parameter("odom_topic", "/utlidar/robot_odom")
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("use_pi_time", False)   # True, falls Uhren von Hund und Pi abweichen
        self.declare_parameter("offset_alpha", 0.02)   # EMA-Faktor fuer den Uhren-Offset
        self.odom_frame = self.get_parameter("odom_frame").value
        self.base_frame = self.get_parameter("base_frame").value
        self.use_pi_time = bool(self.get_parameter("use_pi_time").value)
        self.clock_off = ClockOffset(alpha=float(self.get_parameter("offset_alpha").value))
        self.br = TransformBroadcaster(self)
        topic = self.get_parameter("odom_topic").value
        self.create_subscription(Odometry, topic, self.cb, qos_profile_sensor_data)
        self.count = 0
        self.get_logger().info(f"Warte auf {topic} -> TF {self.odom_frame} -> {self.base_frame}")

    def cb(self, msg: Odometry):
        t = TransformStamped()
        if self.use_pi_time:
            stamp_ns = Time.from_msg(msg.header.stamp).nanoseconds
            off = self.clock_off.update(self.get_clock().now().nanoseconds, stamp_ns)
            t.header.stamp = Time(nanoseconds=stamp_ns + off).to_msg()
        else:
            t.header.stamp = msg.header.stamp
        t.header.frame_id = self.odom_frame
        t.child_frame_id = self.base_frame
        p = msg.pose.pose.position
        t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = p.x, p.y, p.z
        t.transform.rotation = msg.pose.pose.orientation
        self.br.sendTransform(t)
        self.count += 1
        if self.count == 1:
            self.get_logger().info("Erste Odometrie empfangen - TF laeuft.")
            if self.use_pi_time:
                self.get_logger().info(
                    f"Uhren-Offset Pi - Go2: {self.clock_off.offset_ns * 1e-9:+.3f} s")


def main():
    rclpy.init()
    node = Go2OdomTF()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
