# Source this on the LAPTOP before running teleop or checking robot topics.
#   source configs/tmr_laptop_env.sh
# Assumes you are already inside the pixi ROS 2 env (pixi shell) with the workspace
# sourced, AND that multicast works on the laptop<->robot Ethernet link (test with
# `ros2 multicast send` / `ros2 multicast receive` - see README).
#
# Native FastDDS, default discovery. No discovery server, no robot-side config.

export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
# Domain 0 is the teleop domain: nothing in the robot's shell rc files sets
# ROS_DOMAIN_ID, so the stack launches on 0 too. Laptop and robot MUST match -
# a mismatch shows up as "ros2 topic list" seeing only local topics.
# Override per-shell with:  ROS_DOMAIN_ID=<n> source configs/tmr_laptop_env.sh
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
unset ROS_DISCOVERY_SERVER

# Pin DDS to the Ethernet NIC facing the robot (the laptop is multi-homed: WiFi + Ethernet).
# Edit the address inside the XML if the laptop IP changes: ip -4 addr show | grep 172.16.16
_here="$(cd "$(dirname "${BASH_SOURCE[0]:-$0}")" && pwd)"
export FASTRTPS_DEFAULT_PROFILES_FILE="$_here/fastdds_laptop_discovery.xml"

# The ros2 CLI daemon caches DDS settings; restart it so it picks up the above.
ros2 daemon stop >/dev/null 2>&1
ros2 daemon start >/dev/null 2>&1

echo "TMR DDS env set (native FastDDS, default discovery, Ethernet-pinned):"
echo "  NOTE: launch the robot stack with the SAME ROS_DOMAIN_ID."
echo "  RMW=$RMW_IMPLEMENTATION  DOMAIN=$ROS_DOMAIN_ID"
echo "  profile=$FASTRTPS_DEFAULT_PROFILES_FILE"
echo "Verify multicast first:  (robot) ros2 multicast receive   (laptop) ros2 multicast send"
echo "Then:  ros2 topic list   (should include the robot's /olive/... topics)"
