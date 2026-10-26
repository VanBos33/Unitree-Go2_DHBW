#!/usr/bin/env python3
"""Der Go2 liefert /utlidar/cloud bereits in odom-Koordinaten, der Header sagt aber 'utlidar_lidar'.
Dieser Node setzt den Frame korrekt auf 'odom' und publiziert /utlidar/cloud_odom."""
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2


class CloudFix(Node):
    def __init__(self):
        super().__init__("go2_cloud_fix")
        self.declare_parameter("cloud_in", "/utlidar/cloud")
        self.declare_parameter("cloud_out", "/utlidar/cloud_odom")
        self.declare_parameter("frame", "odom")
        self.frame = self.get_parameter("frame").value
        self.pub = self.create_publisher(PointCloud2, self.get_parameter("cloud_out").value, qos_profile_sensor_data)
        self.create_subscription(PointCloud2, self.get_parameter("cloud_in").value, self.cb, qos_profile_sensor_data)
        self.first = True

    def cb(self, msg: PointCloud2):
        msg.header.frame_id = self.frame
        self.pub.publish(msg)
        if self.first:
            self.first = False
            self.get_logger().info(f"Punktwolke laeuft -> /utlidar/cloud_odom (Frame '{self.frame}')")


def main():
    rclpy.init()
    n = CloudFix()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        n.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
