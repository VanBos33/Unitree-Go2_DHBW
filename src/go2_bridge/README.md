# Go2 EDU ↔ ROS 2 Jazzy – Brücke für SLAM/Nav2

Stand: 26.10.2026 – getestet am echten Go2 EDU (Pi 5, Ubuntu 24.04, ROS 2 Jazzy, LAN-Kabel Pi ↔ Hund).

## Was die Brücke liefert
| Ausgabe | Quelle |
|---|---|
| TF `odom → base_link` | `/utlidar/robot_odom` (go2_odom_tf.py) |
| TF `base_link → utlidar_lidar` | statisch (Unitree-URDF) |
| `/scan` (LaserScan, ~14 Hz) | `/utlidar/cloud_base` via pointcloud_to_laserscan |

## Wichtige Erkenntnisse
- Der Go2 sendet seine Topics direkt per DDS über `eth0` – **kein Treiber nötig**, aber **kein `/tf`**.
- `/utlidar/cloud` hat den Header `utlidar_lidar`, liegt aber **nicht** im Sensor-Frame → nicht verwenden.
- Sauber: `/utlidar/cloud_base` (Frame `base_link`) und `/utlidar/cloud_deskewed` (Frame `odom`).
- Das CycloneDDS aus `~/unitree_ws/install` stürzt mit `eth0` ab („buffer overflow“) → `go2_env.sh` blendet es aus.
- Der Go2 hört **nicht** auf `/cmd_vel` → für Nav2 fehlt noch eine Brücke `/cmd_vel` → Sport-API.

## Start (jedes Mal)
Terminal A – Brücke:
```bash
source ~/unitree_ws/src/go2_bridge/go2_env.sh
ros2 launch ~/unitree_ws/src/go2_bridge/go2_bridge.launch.py
```
Terminal B – SLAM:
```bash
source ~/unitree_ws/src/go2_bridge/go2_env.sh
ros2 launch slam_toolbox online_async_launch.py slam_params_file:=$HOME/unitree_ws/src/go2_bridge/slam_go2.yaml
```
RViz (am Pi-Monitor, nicht über SSH):
```bash
source ~/unitree_ws/src/go2_bridge/go2_env.sh
rviz2 -d ~/unitree_ws/src/go2_bridge/go2.rviz
```
In RViz bei LaserScan/PointCloud2: Reliability Policy = **Best Effort**.

## Einstellungen für Nav2 / slam_toolbox
- slam_toolbox: `base_frame: base_link`, `odom_frame: odom`, `scan_topic: /scan`
- Nav2: `robot_base_frame: base_link`, `global_frame: map`, `odom_topic: /utlidar/robot_odom`

## Stellschrauben (Launch-Argumente)
`min_height` (Standard -0.10), `max_height` (0.40), `range_min` (0.30), `range_max` (20.0),
`cloud_topic` (/utlidar/cloud_base), `use_pi_time` (false).
Beispiel: `ros2 launch .../go2_bridge.launch.py min_height:=0.0`

## Einmalige Installation
```bash
sudo apt install -y ros-jazzy-pointcloud-to-laserscan ros-jazzy-rviz2 \
  ros-jazzy-slam-toolbox ros-jazzy-navigation2 ros-jazzy-nav2-bringup
```

## Kamera + Tür-/Fenstererkennung (yolo_zeugs)
```bash
cd ~/unitree_ws/src/yolo_zeugs && python3 go2_tuer_stream.py auto --imgsz 320
```
Browser am Mac: http://100.121.161.1:8080 (Tailscale). Mit `eth0` statt `auto` stürzt es ab (CycloneDDS-Problem).
