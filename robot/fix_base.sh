#!/usr/bin/env bash
# Make the mobile base actually commandable after a bringup. Run it AFTER start_robot.bash.
#
#   ./fix_base.sh            # report, and repair if needed
#   ./fix_base.sh --check    # report only, change nothing
#
# WHY THIS IS NEEDED
# start_robot.bash brings the base up correctly - the log says "Successful 'activate' of
# hardware 'TmrHardware'". Six to ten seconds later its own spawners run controller SWITCH
# operations, and a switch stalls the 1 kHz RT loop long enough that libfranka misses its
# deadline. ros2_control then deactivates the component. Observed 2026-08-24:
#
#   t+201s  Successful 'activate' of hardware 'TmrHardware'
#   t+207s  Loading controller 'swerve_drive_controller'
#   t+211s  Loading controller 'joint_state_broadcaster'
#   t+213s  spawner-4 died, exit code 1          <- hardware is unconfigured from here
#
# The trap is that NOTHING looks broken afterwards: swerve_drive_controller still reports
# 'active' and /swerve_drive_controller/odom still publishes a healthy 50 Hz. Only
# list_hardware_components tells the truth. Always check THAT, never list_controllers.
#
# ORDER MATTERS: hardware first, then cycle the controller. Activating the controller
# alone does nothing - it is already 'active', so on_activate() never re-runs and the
# interfaces are never re-bound.
#
# Left alone the hardware holds active indefinitely (verified 60 s untouched), so this is
# a one-shot repair, not a babysitter.
#
# Deliberately does NOT touch:
#   * the arms - re-activating joint_impedance_controller while GELLO publishes would
#     command the arm toward the leader's pose. Use recover.py --side <side>, hands off.
#   * joint_state_broadcaster - activating it is what knocked the hardware out in the
#     first place. It only feeds real wheel values into /dynamic_joint_states; the base
#     drives fine without it.

set -uo pipefail

check_only=false
[ "${1:-}" = "--check" ] && check_only=true

if ! command -v ros2 >/dev/null 2>&1; then
  set +u
  source /opt/ros/humble/setup.bash
  source "${TMR_WS:-$HOME/tams_ws}/install/local_setup.bash"
  set -u
fi
export ROS_DOMAIN_ID="${TMR_ROS_DOMAIN_ID:-0}"

CM=/controller_manager
hw_state() {
  timeout 25 ros2 control list_hardware_components 2>/dev/null \
    | grep -A4 'name:.*TmrHardware' | grep -oE 'active|unconfigured|inactive|finalized' | head -1
}
claimed() {
  timeout 25 ros2 control list_hardware_interfaces 2>/dev/null \
    | grep -cE 'vx/cartesian_velocity.*\[claimed\]'
}

# Distinguish "the base is faulted" from "the base is not running at all" - both used to
# print `unknown`, which reads as a fault and sends you repairing a stack that is down.
if ! timeout 20 ros2 service list 2>/dev/null | grep -q '^/controller_manager/list_hardware_components$'; then
  echo "The base controller_manager is not reachable." >&2
  echo "  Either the base is not running (start it with ~/start_robot.bash), or its" >&2
  echo "  ros2_control_node died. Check with: pgrep -af tmrv0_2.launch" >&2
  exit 2
fi

state="$(hw_state)"
echo "TmrHardware : ${state:-unreadable (controller_manager did not answer in time)}"
echo "vx claimed  : $(claimed)  (want 1)"

if [ "$state" = "active" ] && [ "$(claimed)" = "1" ]; then
  echo "Base is commandable. Nothing to do."
  exit 0
fi

if $check_only; then
  echo "NOT commandable. Re-run without --check to repair."
  exit 1
fi

# 1. hardware first
if [ "$state" != "active" ]; then
  echo "-> activating TmrHardware"
  timeout 30 ros2 service call "$CM/set_hardware_component_state" \
    controller_manager_msgs/srv/SetHardwareComponentState \
    "{name: TmrHardware, target_state: {id: 3, label: active}}" >/dev/null 2>&1
  sleep 2
  [ "$(hw_state)" = "active" ] || {
    echo "FAILED: hardware stuck at '$(hw_state)'." >&2
    echo "  The base may be faulted robot-side. Check Desk (https://172.16.16.10/)," >&2
    echo "  then restart the base launch." >&2
    exit 1; }
fi

# 2. then re-bind the controller
echo "-> cycling swerve_drive_controller so it re-claims"
SW="$CM/switch_controller"; T=controller_manager_msgs/srv/SwitchController
timeout 30 ros2 service call "$SW" "$T" \
  "{deactivate_controllers: ['swerve_drive_controller'], strictness: 1}" >/dev/null 2>&1
sleep 2
timeout 30 ros2 service call "$SW" "$T" \
  "{activate_controllers: ['swerve_drive_controller'], strictness: 1}" >/dev/null 2>&1
sleep 3

echo
echo "TmrHardware : $(hw_state)"
echo "vx claimed  : $(claimed)  (want 1)"
if [ "$(hw_state)" = "active" ] && [ "$(claimed)" = "1" ]; then
  echo "Base is commandable. Test with:  python3 ~/base_nudge.py"
  exit 0
fi
echo "Still not commandable - restart the base launch and run this again." >&2
exit 1
