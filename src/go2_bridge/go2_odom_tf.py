#!/usr/bin/env python3
"""Macht aus der Go2-Odometrie (/utlidar/robot_odom) den TF  odom -> base_link."""
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster


class Go2OdomTF(Node):
    def __init__(self):
        super().__init__("go2_odom_tf")
        self.declare_parameter("odom_topic", "/utlidar/robot_odom")
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("use_pi_time", False)  # True, falls Uhren von Hund und Pi abweichen
        self.odom_frame = self.get_parameter("odom_frame").value
        self.base_frame = self.get_parameter("base_frame").value
        self.use_pi_time = self.get_parameter("use_pi_time").value
        self.br = TransformBroadcaster(self)
        topic = self.get_parameter("odom_topic").value
        self.create_subscription(Odometry, topic, self.cb, qos_profile_sensor_data)
        self.count = 0
        self.get_logger().info(f"Warte auf {topic} -> TF {self.odom_frame} -> {self.base_frame}")

    def cb(self, msg: Odometry):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg() if self.use_pi_time else msg.header.stamp
        t.header.frame_id = self.odom_frame
        t.child_frame_id = self.base_frame
        p = msg.pose.pose.position
        t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = p.x, p.y, p.z
        t.transform.rotation = msg.pose.pose.orientation
        self.br.sendTransform(t)
        self.count += 1
        if self.count == 1:
            self.get_logger().info("Erste Odometrie empfangen - TF laeuft.")


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
