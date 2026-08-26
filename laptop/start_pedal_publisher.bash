#!/usr/bin/env bash
# Publish ONLY the foot-switch state on /pedal/state, for a split-host teleop setup.
#
#   ./start_pedal_publisher.bash [--restart]
#
# The foot switches are plugged into THIS laptop, but every bridge that consumes them
# (base_bridge, spine_bridge, mode_manager, labs_pedal_bridge, the state publishers) runs
# on the GELLO host inside its Humble container. Those bridges only SUBSCRIBE to
# /pedal/state, so it does not matter which machine publishes it - this script supplies
# that one topic and nothing else.
#
# Start this BEFORE ./start_teleop.bash --no-pedals on the GELLO host, so the bridges see
# a live /pedal/state from their first tick.
#
# Deliberately NOT start_pedal.bash: that one also starts the bridges, which would then
# compete with the GELLO host's copies for the same topics.

set -Eeuo pipefail

workspace_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$workspace_dir"

restart=false
while (( $# > 0 )); do
  case "$1" in
    --restart) restart=true; shift ;;
    *) echo "Usage: $0 [--restart]" >&2; exit 2 ;;
  esac
done

source configs/teleop_common.sh
teleop_enter_ros_env "$(basename "${BASH_SOURCE[0]}")"
teleop_source_workspace

# The publisher GRABS both switches exclusively (evdev grab), so a survivor makes a new
# run silently see no pedals.
teleop_guard_running '/install(_pixi)?/pedal_state_publisher/' \
  "Pedal publisher" "$restart" 'start_pedal_publisher\.bash|start_pedal\.bash'

teleop_check_pedal_devices

cat <<MSG

Publishing /pedal/state on ROS_DOMAIN_ID=$ROS_DOMAIN_ID. Nothing else runs here.

  Next, on the GELLO host:  ./start_teleop.bash --no-pedals --record --task-id <uuid>
  Check it arrives there:   ros2 topic echo /pedal/state

Press Ctrl+C to stop.
MSG

exec ros2 run pedal_state_publisher pedal_state_publisher
