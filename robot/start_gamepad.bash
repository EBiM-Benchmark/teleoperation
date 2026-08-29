#!/usr/bin/env bash
# Start ONLY the companion-side joystick reader for coexistence with GELLO + pedals.
#
# This deliberately does not use the old `gamepad_demo` alias: that alias launches a
# second ros2_control stack and conflicts with the running swerve base.  joy_node sends
# raw state to laptop-side base_bridge, which gives any pressed pedal priority.
set -Eeuo pipefail

device="${TMR_GAMEPAD_DEVICE:-/dev/input/js0}"
case "${1:-}" in
  --device)
    [ $# -eq 2 ] || { echo "Usage: $0 [--device /dev/input/jsN]" >&2; exit 2; }
    device="$2"
    ;;
  "") ;;
  *) echo "Usage: $0 [--device /dev/input/jsN]" >&2; exit 2 ;;
esac

if [ ! -e "$device" ]; then
  echo "ERROR: gamepad device not found: $device" >&2
  echo "Connect the controller, then check: ls -l /dev/input/js*" >&2
  exit 1
fi
if [ ! -r "$device" ]; then
  echo "ERROR: gamepad is not readable: $device" >&2
  echo "Add $(id -un) to the input group, then log out and back in." >&2
  exit 1
fi

export ROS_DOMAIN_ID="${TMR_ROS_DOMAIN_ID:-0}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
dds_profile="${TMR_DDS_PROFILE:-$HOME/fastdds_udp_only.xml}"
if [ -f "$dds_profile" ]; then
  export FASTRTPS_DEFAULT_PROFILES_FILE="$dds_profile"
fi

set +u
source /opt/ros/humble/setup.bash
source "${TMR_WS:-$HOME/tams_ws}/install/local_setup.bash"
set -u

echo "Gamepad coexistence reader: $device -> /teleop/gamepad/joy"
echo "Hold RB to drive. Hold RB+LB for turbo. Any pressed pedal has priority."
echo "Ctrl+C stops only the gamepad; GELLO, pedals, and recording keep running."
exec ros2 run joy joy_node --ros-args \
  -r __node:=coexist_gamepad_joy \
  -r joy:=/teleop/gamepad/joy \
  -p dev:="$device" \
  -p deadzone:=0.3 \
  -p autorepeat_rate:=20.0
