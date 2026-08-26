#!/usr/bin/env bash
# Report whether the mobile base is in a state where it CAN move. Read-only: this
# commands nothing and moves nothing. Run it on the companion after each bringup stage
# to find which stage breaks the base.
#
#   ./base_health.sh
#
# Everything here uses --no-daemon on purpose. The ros2 CLI daemon caches the DDS graph
# and will happily report "Subscription count: 0" on a topic that is in fact connected -
# that false reading cost a whole debugging session on 2026-08-22.

set -uo pipefail

export ROS_DOMAIN_ID="${TMR_ROS_DOMAIN_ID:-0}"
if ! command -v ros2 >/dev/null 2>&1; then
  set +u
  source /opt/ros/humble/setup.bash
  source "${TMR_WS:-$HOME/tams_ws}/install/local_setup.bash"
  set -u
fi

echo "ROS_DOMAIN_ID=$ROS_DOMAIN_ID"

echo
echo "--- running stacks ---"
pgrep -af 'tmrv0_2.launch|spine.launch|franka_fr3_arm_controllers.launch|robotiq' \
  | sed 's/--ros-args.*//' | head -8 || echo "  none"

echo
echo "--- controllers (want: swerve_drive_controller active) ---"
timeout 20 ros2 control list_controllers 2>&1 | head -8

echo
echo "--- base command interfaces (want: claimed) ---"
timeout 20 ros2 control list_hardware_interfaces 2>&1 \
  | grep -E '(vx|vy|wz)/cartesian_velocity' | head -6

echo
echo "--- cmd_vel endpoints (want: >=1 subscriber, the controller) ---"
timeout 25 ros2 topic info -v --no-daemon /swerve_drive_controller/cmd_vel 2>&1 \
  | grep -E 'count|Node name|Topic type'

echo
echo "--- odom (want: ~50 Hz; proves the controller is spinning) ---"
timeout 8 ros2 topic hz --no-daemon /swerve_drive_controller/odom 2>&1 | head -2

echo
echo "If all of the above look right and the base still will not move, the command is"
echo "being discarded or the fault is below ros2_control. Command it directly with:"
echo "  python3 base_nudge.py     # stamps each message; ros2 topic pub does NOT"
echo "and watch what the controller accepted:"
echo "  ros2 topic echo /swerve_drive_controller/cmd_vel_out"
