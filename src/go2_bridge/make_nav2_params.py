#!/usr/bin/env python3
"""
Erzeugt nav2_go2_params.yaml aus den Standard-Parametern der installierten Nav2-Version
und passt nur das an, was für den Go2 anders ist. So bleiben alle Nodes, die die
Jazzy-Bringup-Datei startet (z. B. docking_server, smoother_server), gültig konfiguriert.

Aufruf auf dem Pi:
  python3 ~/unitree_ws/src/go2_bridge/make_nav2_params.py
"""
import os
import sys
import yaml

SRC = "/opt/ros/jazzy/share/nav2_bringup/params/nav2_params.yaml"
DST = os.path.join(os.path.dirname(os.path.abspath(__file__)), "nav2_go2_params.yaml")
if len(sys.argv) > 1:
    SRC = sys.argv[1]
if len(sys.argv) > 2:
    DST = sys.argv[2]

# Go2 ca. 0,70 m lang, 0,31 m breit (Rumpf, base_link mittig) – etwas knapper, damit enge
# Gänge im Klassenzimmer den Hund nicht einsperren
FOOTPRINT = "[[0.32, 0.15], [0.32, -0.15], [-0.32, -0.15], [-0.32, 0.15]]"
# DWB fährt vorwärts + dreht (VX_MIN = 0), kein Seitwärts (VY_MAX = 0).
# Rückwärts nur über das BackUp-Recovery (VX_BACKUP), go2_cmd_vel.py lässt bis -0,15 m/s durch.
VX_MAX, VX_MIN, VY_MAX, WZ_MAX = 0.35, 0.0, 0.0, 0.40
VX_BACKUP = -0.15

with open(SRC) as f:
    P = yaml.safe_load(f)


def walk(node):
    """Frame- und Odom-Namen überall ersetzen."""
    if isinstance(node, dict):
        for k, v in list(node.items()):
            if isinstance(v, str):
                if v == "base_footprint":
                    node[k] = "base_link"
                if k.endswith("odom_topic") and v in ("/odom", "odom"):
                    node[k] = "/utlidar/robot_odom"
            else:
                walk(v)
    elif isinstance(node, list):
        for x in node:
            walk(x)


walk(P)


def rp(name):
    return P[name]["ros__parameters"]


# ---------- AMCL ----------
a = rp("amcl")
a.update({
    "laser_min_range": 0.30,
    "laser_max_range": 15.0,
    "scan_topic": "/scan",
    # Öfter aktualisieren, vor allem beim Drehen (0,08 rad ≈ 4,5°)
    "update_min_d": 0.08,
    "update_min_a": 0.08,
    "transform_tolerance": 1.0,
    # Omni-Modell: Der Vierbeiner rutscht beim Drehen auch seitlich -> Partikel verarmen nicht
    "robot_model_type": "nav2_amcl::OmniMotionModel",
    # Mehr Rauschen: der Dreh-Odometrie des Go2 weniger trauen
    "alpha1": 0.4, "alpha2": 0.4, "alpha3": 0.25, "alpha4": 0.4, "alpha5": 0.25,
    "max_beams": 120,
    "min_particles": 500,
    "max_particles": 2500,
    # Startpose (0,0,0) sofort setzen, damit Nav2 beim Start nicht auf "map" wartet.
    # Die echte Position setzt man danach wie gewohnt in Foxglove.
    "set_initial_pose": True,
    "initial_pose": {"x": 0.0, "y": 0.0, "z": 0.0, "yaw": 0.0},
})

# ---------- BT-Navigator: dem Pi mehr Zeit für Antworten geben ----------
btn = rp("bt_navigator")
btn["default_server_timeout"] = 500      # ms (Standard 20 ms ist auf dem Pi zu knapp)
btn["wait_for_service_timeout"] = 2000   # ms

# ---------- Controller: DWB statt MPPI (leichter für den Pi 5), holonom ----------
c = rp("controller_server")
c["controller_frequency"] = 10.0
c["controller_plugins"] = ["FollowPath"]
c["FollowPath"] = {
    # Reiner DWB, KEIN RotationShimController mehr: Auf der Stelle drehen erzeugt beim
    # Vierbeiner den meisten Odometrie-Schlupf und verschmiert den Scan. DWB fährt
    # stattdessen weiche Bögen (vorwärts laufen und dabei drehen).
    "plugin": "dwb_core::DWBLocalPlanner",
    "debug_trajectory_details": False,
    "min_vel_x": VX_MIN, "max_vel_x": VX_MAX,
    "min_vel_y": -VY_MAX, "max_vel_y": VY_MAX,
    "max_vel_theta": WZ_MAX,
    "min_speed_xy": 0.0, "max_speed_xy": VX_MAX, "min_speed_theta": 0.18,
    "acc_lim_x": 0.6, "acc_lim_y": 0.0, "acc_lim_theta": 0.8,
    "decel_lim_x": -0.8, "decel_lim_y": 0.0, "decel_lim_theta": -1.0,
    "vx_samples": 15, "vy_samples": 1, "vtheta_samples": 20,
    "sim_time": 1.7,
    "linear_granularity": 0.05, "angular_granularity": 0.025,
    "transform_tolerance": 0.5,
    "xy_goal_tolerance": 0.25,
    "trans_stopped_velocity": 0.05,
    "short_circuit_trajectory_evaluation": True,
    "stateful": True,
    "critics": ["RotateToGoal", "Oscillation", "BaseObstacle", "GoalAlign", "PathAlign", "PathDist", "GoalDist"],
    "BaseObstacle.scale": 0.02,
    "PathAlign.scale": 24.0, "PathAlign.forward_point_distance": 0.20,
    "GoalAlign.scale": 16.0, "GoalAlign.forward_point_distance": 0.20,
    "PathDist.scale": 32.0, "GoalDist.scale": 24.0,
    "RotateToGoal.scale": 16.0, "RotateToGoal.slowing_factor": 5.0, "RotateToGoal.lookahead_time": -1.0,
}
# Fortschritts-Prüfung lockern: langsames, vorsichtiges Drehen soll nicht als
# "Failed to make progress" abgebrochen werden.
for k in ("progress_checker",):
    if k in c and isinstance(c[k], dict):
        c[k]["required_movement_radius"] = 0.15
        c[k]["movement_time_allowance"] = 20.0
for k in ("general_goal_checker", "goal_checker"):
    if k in c and isinstance(c[k], dict):
        c[k]["xy_goal_tolerance"] = 0.25
        c[k]["yaw_goal_tolerance"] = 0.35


# ---------- Costmaps ----------
def costmap(cm):
    cm.pop("robot_radius", None)
    cm["footprint"] = FOOTPRINT
    cm["robot_base_frame"] = "base_link"
    if "inflation_layer" in cm:
        # Kleiner Radius + steiler Abfall: der Hund "klebt" nicht mehr an Wänden
        cm["inflation_layer"]["inflation_radius"] = 0.28
        cm["inflation_layer"]["cost_scaling_factor"] = 4.0
    for layer in ("obstacle_layer", "voxel_layer"):
        if layer in cm and "scan" in cm[layer]:
            s = cm[layer]["scan"]
            s["topic"] = "/scan"
            s["clearing"] = True
            s["marking"] = True
            s["raytrace_min_range"] = 0.20
            s["raytrace_max_range"] = 12.0
            s["obstacle_min_range"] = 0.25
            s["obstacle_max_range"] = 8.0


costmap(P["local_costmap"]["local_costmap"]["ros__parameters"])
costmap(P["global_costmap"]["global_costmap"]["ros__parameters"])
P["global_costmap"]["global_costmap"]["ros__parameters"]["track_unknown_space"] = True
P["planner_server"]["ros__parameters"]["GridBased"]["allow_unknown"] = True

# ---------- Behaviors, Glättung ----------
b = rp("behavior_server")
# Recovery-Drehungen langsam, damit der Scan nicht verschmiert
b["max_rotational_vel"] = 0.35
b["min_rotational_vel"] = 0.20
b["rotational_acc_lim"] = 0.8

v = rp("velocity_smoother")
v["max_velocity"] = [VX_MAX, VY_MAX, WZ_MAX]
# Rückwärts bis -0,15 m/s durchlassen, sonst verschluckt der Smoother das BackUp-Recovery
v["min_velocity"] = [VX_BACKUP, -VY_MAX, -WZ_MAX]

# Hinweis: Das BackUp-Behavior muss bleiben – der Standard-Verhaltensbaum des bt_navigator
# erwartet es. Es fährt jetzt wirklich langsam rückwärts (go2_cmd_vel.py erlaubt bis -0,15 m/s)
# und befreit den Hund, wenn die Nase in der Wand-Inflation steckt.
v["max_accel"] = [0.6, 0.5, 0.8]
v["max_decel"] = [-0.8, -0.5, -1.0]
v["feedback"] = "OPEN_LOOP"

# ---------- Collision-Monitor ----------
# Die "Approach"-Zone hat den Hund ständig auf 0 gebremst (Scan-Punkte direkt am Rumpf).
# Hindernisse vermeidet Nav2 weiterhin über die Costmaps; der Not-Halt läuft über Fernbedienung/Watchdog.
cm = rp("collision_monitor")
cm["transform_tolerance"] = 0.5
cm["source_timeout"] = 2.0
if "FootprintApproach" in cm:
    cm["FootprintApproach"]["enabled"] = False

with open(DST, "w") as f:
    f.write("# Automatisch erzeugt von make_nav2_params.py aus\n# " + SRC + "\n")
    yaml.safe_dump(P, f, sort_keys=False, default_flow_style=False, allow_unicode=True)
print("geschrieben:", DST)
