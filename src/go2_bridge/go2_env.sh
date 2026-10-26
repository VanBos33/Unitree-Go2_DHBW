# Umgebung fuer ROS 2 <-> Go2 EDU (in JEDEM neuen Terminal ausfuehren: source ~/go2_env.sh)
source /opt/ros/jazzy/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
# eigenes CycloneDDS aus ~/unitree_ws ausblenden (sonst Absturz "buffer overflow" mit eth0)
export LD_LIBRARY_PATH=$(echo "$LD_LIBRARY_PATH" | tr ':' '\n' | grep -v unitree_ws | paste -sd:)
export CYCLONEDDS_URI='<CycloneDDS><Domain><General><Interfaces><NetworkInterface name="eth0" priority="default" multicast="default"/></Interfaces></General></Domain></CycloneDDS>'
echo "Go2-Umgebung geladen (RMW=$RMW_IMPLEMENTATION, Interface eth0)"
