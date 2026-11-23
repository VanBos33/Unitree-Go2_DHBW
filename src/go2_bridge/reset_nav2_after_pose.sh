#!/usr/bin/env bash
# Nach dem Setzen der "2D Pose Estimate" in Foxglove ausfuehren:
#   1. Local + Global Costmap komplett leeren (Geisterwaende von der falschen Startpose weg)
#   2. Pruefen, ob der bt_navigator aktiv ist, sonst Nav2 starten bzw. fortsetzen
#
# Aufruf:  ~/unitree_ws/src/go2_bridge/reset_nav2_after_pose.sh

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
if [ -f "$HERE/go2_env.sh" ]; then
  # shellcheck disable=SC1091
  source "$HERE/go2_env.sh"
elif [ -f /opt/ros/jazzy/setup.bash ]; then
  # shellcheck disable=SC1091
  source /opt/ros/jazzy/setup.bash
fi

echo "== Costmaps leeren =="
for cm in local global; do
  srv="/${cm}_costmap/clear_entirely_${cm}_costmap"
  if ros2 service call "$srv" nav2_msgs/srv/ClearEntireCostmap "{}" >/dev/null 2>&1; then
    echo "  $srv  OK"
  else
    echo "  $srv  FEHLER (laeuft Nav2?)"
  fi
done

echo "== bt_navigator pruefen =="
state="$(ros2 lifecycle get /bt_navigator 2>/dev/null | awk '{print $1}')"
echo "  Zustand: ${state:-unbekannt}"
MGR=/lifecycle_manager_navigation/manage_nodes
case "$state" in
  active)
    echo "  bt_navigator ist aktiv - nichts zu tun." ;;
  inactive)
    echo "  inaktiv -> RESUME"
    ros2 service call "$MGR" nav2_msgs/srv/ManageLifecycleNodes "{command: 2}" ;;
  unconfigured|"")
    echo "  nicht konfiguriert -> STARTUP"
    ros2 service call "$MGR" nav2_msgs/srv/ManageLifecycleNodes "{command: 0}" ;;
  *)
    echo "  unerwarteter Zustand '$state' -> RESET + STARTUP"
    ros2 service call "$MGR" nav2_msgs/srv/ManageLifecycleNodes "{command: 3}"
    ros2 service call "$MGR" nav2_msgs/srv/ManageLifecycleNodes "{command: 0}" ;;
esac
echo "Fertig. Jetzt das Ziel (Nav2 Goal) setzen."
