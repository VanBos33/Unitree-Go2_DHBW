"""
Nav2 für den Go2: Karte raum3 + nav2_go2_params.yaml (aus make_nav2_params.py)
  ros2 launch ~/unitree_ws/src/go2_bridge/go2_nav2.launch.py
  ros2 launch ~/unitree_ws/src/go2_bridge/go2_nav2.launch.py map:=/pfad/zu/karte.yaml
"""
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

HERE = os.path.dirname(os.path.abspath(__file__))


def generate_launch_description():
    bringup = get_package_share_directory("nav2_bringup")
    return LaunchDescription([
        DeclareLaunchArgument("map", default_value=os.path.join(HERE, "maps", "raum3.yaml")),
        DeclareLaunchArgument("params_file", default_value=os.path.join(HERE, "nav2_go2_params.yaml")),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(bringup, "launch", "bringup_launch.py")),
            launch_arguments={
                "map": LaunchConfiguration("map"),
                "params_file": LaunchConfiguration("params_file"),
                "use_sim_time": "false",
                "autostart": "true",
                "use_composition": "False",
            }.items(),
        ),
    ])
