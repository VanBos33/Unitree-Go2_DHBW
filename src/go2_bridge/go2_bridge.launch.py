"""
Go2 EDU -> Nav2/SLAM-Bruecke
  * TF  odom -> base_link          (aus /utlidar/robot_odom)
  * TF  base_link -> utlidar_lidar (statisch, Einbaulage L1-LiDAR laut Unitree-URDF)
  * /scan (LaserScan)              (aus /utlidar/cloud_base via pointcloud_to_laserscan)
    Hinweis: /utlidar/cloud ist trotz Header 'utlidar_lidar' NICHT im Sensor-Frame.
    Sauber sind: /utlidar/cloud_base (Frame base_link) und /utlidar/cloud_deskewed (Frame odom).

Start:  source ~/go2_env.sh && ros2 launch ~/go2_bridge.launch.py
Optional: ros2 launch ~/go2_bridge.launch.py min_height:=-0.1 max_height:=0.5 use_pi_time:=true
"""
import os
import sys

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

HERE = os.path.dirname(os.path.realpath(__file__))


def generate_launch_description():
    lc = LaunchConfiguration
    args = [
        # Einbaulage des L1 relativ zu base_link (Unitree go2_description, "radar_joint")
        DeclareLaunchArgument("lidar_x", default_value="0.28945"),
        DeclareLaunchArgument("lidar_y", default_value="0.0"),
        DeclareLaunchArgument("lidar_z", default_value="-0.046825"),
        DeclareLaunchArgument("lidar_roll", default_value="0.0"),
        DeclareLaunchArgument("lidar_pitch", default_value="2.8782"),
        DeclareLaunchArgument("lidar_yaw", default_value="0.0"),
        # Hoehenband (relativ zu base_link), das in den 2D-Scan uebernommen wird
        DeclareLaunchArgument("min_height", default_value="-0.10"),
        DeclareLaunchArgument("max_height", default_value="0.40"),
        DeclareLaunchArgument("range_min", default_value="0.30"),
        DeclareLaunchArgument("range_max", default_value="20.0"),
        DeclareLaunchArgument("use_pi_time", default_value="false"),
        DeclareLaunchArgument("cloud_topic", default_value="/utlidar/cloud_base"),
    ]

    static_lidar = Node(
        package="tf2_ros", executable="static_transform_publisher", name="base_to_utlidar",
        arguments=["--x", lc("lidar_x"), "--y", lc("lidar_y"), "--z", lc("lidar_z"),
                   "--roll", lc("lidar_roll"), "--pitch", lc("lidar_pitch"), "--yaw", lc("lidar_yaw"),
                   "--frame-id", "base_link", "--child-frame-id", "utlidar_lidar"],
    )

    odom_tf = ExecuteProcess(
        cmd=[sys.executable, os.path.join(HERE, "go2_odom_tf.py"),
             "--ros-args", "-p", ["use_pi_time:=", lc("use_pi_time")]],
        output="screen",
    )

    cloud_fix = ExecuteProcess(
        cmd=[sys.executable, os.path.join(HERE, "go2_cloud_fix.py")],
        output="screen",
    )

    cloud_to_scan = Node(
        package="pointcloud_to_laserscan", executable="pointcloud_to_laserscan_node",
        name="go2_cloud_to_scan", output="screen",
        remappings=[("cloud_in", lc("cloud_topic")), ("scan", "/scan")],
        parameters=[{
            "target_frame": "base_link",
            "transform_tolerance": 0.1,
            "min_height": lc("min_height"),
            "max_height": lc("max_height"),
            "angle_min": -3.14159,
            "angle_max": 3.14159,
            "angle_increment": 0.0087,  # 0,5 Grad
            "scan_time": 0.07,           # L1 liefert ~14 Hz
            "range_min": lc("range_min"),
            "range_max": lc("range_max"),
            "use_inf": True,
        }],
    )

    return LaunchDescription(args + [static_lidar, odom_tf, cloud_to_scan])
