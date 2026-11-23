#!/usr/bin/env python3
"""
ROS-Seite der Fahr-Brücke: /cmd_vel -> UDP (localhost:9870) -> go2_sport_sender.py

Warum zwei Programme? ROS 2 (rmw_cyclonedds) und das Unitree-SDK (eigene CycloneDDS)
lassen sich nicht im selben Prozess starten ("create domain error").

Regeln:
  * Vorwärts bis max_vx.
  * Rückwärts nur langsam (bis max_reverse, Standard 0,15 m/s) – damit Nav2s BackUp-Recovery
    und DWB den Hund aus der Wand-Inflation befreien können. Sehr kleine Rückwärtsbefehle
    werden auf min_reverse (0,10 m/s) angehoben, sonst lehnt sich der Hund nur nach hinten.
  * Drehen: Totband 0,06 rad/s, auf der Stelle mindestens 0,30 rad/s (darunter dreht der Go2 nicht), maximal 0,40 rad/s.
  * Seitwärts nur mit allow_strafe:=true (Totband 0,08, max. 0,15 m/s).

Start (mit go2_env.sh):
  python3 ~/unitree_ws/src/go2_bridge/go2_cmd_vel.py
  python3 ~/unitree_ws/src/go2_bridge/go2_cmd_vel.py --ros-args -p allow_strafe:=true
  python3 ~/unitree_ws/src/go2_bridge/go2_cmd_vel.py --ros-args -p allow_reverse:=false
"""
import json
import math
import socket

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

PORT = 9870


class CmdVelForwarder(Node):
    def __init__(self):
        super().__init__("go2_cmd_vel_bridge")
        p = self.declare_parameter
        self.max_vx = float(p("max_vx", 0.35).value)
        self.deadband = float(p("deadband", 0.02).value)
        # Rückwärts: langsam erlaubt (für BackUp-Recovery)
        self.allow_reverse = bool(p("allow_reverse", True).value)
        self.max_reverse = float(p("max_reverse", 0.15).value)
        self.min_reverse = float(p("min_reverse", 0.10).value)
        self.reverse_boost_below = float(p("reverse_boost_below", 0.08).value)
        # Seitwärts: standardmäßig gesperrt
        self.allow_strafe = bool(p("allow_strafe", False).value)
        self.max_vy = float(p("max_vy", 0.15).value)
        self.vy_deadband = float(p("vy_deadband", 0.08).value)
        # Drehen: Totband 0,06; min. 0,30 (getestet: bei 0,22 dreht der Go2 nicht)
        self.max_vyaw = float(p("max_vyaw", 0.40).value)
        self.yaw_deadband = float(p("yaw_deadband", 0.06).value)
        self.min_yaw = float(p("min_yaw", 0.30).value)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.n = 0
        self.create_subscription(Twist, "/cmd_vel", self.on_cmd, 10)
        self.get_logger().info(
            f"Bereit: /cmd_vel -> UDP {PORT}. vx max {self.max_vx}, "
            f"rückwärts {'bis -' + str(self.max_reverse) if self.allow_reverse else 'GESPERRT'}, "
            f"seitwärts {'bis ' + str(self.max_vy) if self.allow_strafe else 'GESPERRT'}, "
            f"Drehung {self.min_yaw}..{self.max_vyaw} rad/s (Totband {self.yaw_deadband})")

    def shape_x(self, v):
        if abs(v) < self.deadband:
            return 0.0
        if v > 0.0:
            return min(v, self.max_vx)
        if not self.allow_reverse:
            return 0.0
        # Rückwärts: sehr kleine Werte anheben, große begrenzen
        if v > -self.reverse_boost_below:
            v = -self.min_reverse
        return max(v, -self.max_reverse)

    def shape_y(self, v):
        if not self.allow_strafe or abs(v) < self.vy_deadband:
            return 0.0
        return max(-self.max_vy, min(self.max_vy, v))

    def shape_yaw(self, z, x):
        if abs(z) < self.yaw_deadband:
            return 0.0
        z = max(-self.max_vyaw, min(self.max_vyaw, z))
        # Mindestdrehrate nur auf der Stelle: beim Laufen reagiert der Go2 auch auf kleine
        # Drehbefehle, dort würde ein Mindestwert die Kurven von DWB verfälschen.
        if abs(x) < 0.05 and abs(z) < self.min_yaw:
            z = math.copysign(self.min_yaw, z)
        return z

    def on_cmd(self, m: Twist):
        x = self.shape_x(m.linear.x)
        y = self.shape_y(m.linear.y)
        z = self.shape_yaw(m.angular.z, x)
        cmd = {"x": x, "y": y, "z": z}
        self.sock.sendto(json.dumps(cmd).encode(), ("127.0.0.1", PORT))
        self.n += 1
        if self.n == 1 or self.n % 100 == 0:
            self.get_logger().info(f"/cmd_vel #{self.n}: {cmd}")


def main():
    rclpy.init()
    node = CmdVelForwarder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.sock.sendto(b'{"stop": true}', ("127.0.0.1", PORT))
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
