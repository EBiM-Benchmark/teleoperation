#!/usr/bin/env bash
# Record a raw rosbag of everything a LeRobot dataset needs.
#
# A sibling of start_teleop.bash: runs in the Humble container, foreground, Ctrl+C to stop.
#
# WHY A RAW BAG FIRST
# LABS owns the real recording pipeline, but this proves the topics exist and are gapless
# BEFORE committing to it, and an MCAP bag is convertible either way. The topic list below
# is exactly LABS' manifest (labs_integration/tmr_station/config_data_recorder.yml) plus the
# two base lidars, so what this captures is what LABS would capture.
#
# THE RULE THIS SCRIPT EXISTS TO ENFORCE
# LABS' TemporalSynchronizer takes the LATEST first-message time and the EARLIEST last-message
# time across all configured topics, and fails the episode if either is more than 1 s from
# the episode bounds. One silent publisher therefore fails the WHOLE conversion, not just its
# own column. `--check` refuses to record when a required topic has no publisher, so you find
# out in a second rather than after a ten-minute episode.
#
# ROS DOMAIN
# Everything must be on ONE domain - a single `ros2 bag record` sees exactly one. Domain 0 is
# the target: teleop, LABS docker-compose and (since 2026-08-23) start_zed.bash all use it.
# If a camera or the ZED is missing from --check, suspect a stale ROS_DOMAIN_ID=100 first.
#
# USAGE
#   ./record_bag.bash --check              # what is publishing? changes nothing
#   ./record_bag.bash                      # record everything, Ctrl+C to stop
#   ./record_bag.bash --no-video           # state/action/lidar only (low bandwidth)
#   ./record_bag.bash --out DIR            # default ~/teleop_bags/<timestamp>
#   ./record_bag.bash --name my_episode    # bag name suffix
set -Eeuo pipefail

CONTAINER="${TMR_CONTAINER:-gello-humble}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"

repo_host="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
case "$repo_host" in
  "$HOME"/*) repo_ctr="/workspace/${repo_host#"$HOME"/}" ;;
  *) echo "ERROR: expected the repo under \$HOME ($HOME), got $repo_host" >&2; exit 1 ;;
esac

# ---------------------------------------------------------------- topic manifest
# Observation - robot state. These 20 dims are the arms; see modality.json for the layout.
STATE_TOPICS=(
  /left/franka_robot_state_broadcaster/measured_joint_states
  /left/franka_robot_state_broadcaster/external_joint_torques
  /left/franka_robot_state_broadcaster/external_wrench_in_stiffness_frame
  /left/gripper/gripper_joint_states
  /right/franka_robot_state_broadcaster/measured_joint_states
  /right/franka_robot_state_broadcaster/external_joint_torques
  /right/franka_robot_state_broadcaster/external_wrench_in_stiffness_frame
  /right/gripper/gripper_joint_states
  /mobile_base/pose
  /mobile_base/twist
  /swerve_drive_controller/cmd_vel_out
  /spine/joint_states
)
# Actions - the follower topic of each teleop device.
ACTION_TOPICS=(
  /left/gello/joint_states
  /right/gello/joint_states
  /left/gripper/gripper_client/target_gripper_width_percent
  /right/gripper/gripper_client/target_gripper_width_percent
  /swerve_drive_controller/cmd_vel
  /spine/target_height
)
# Video. UNCOMPRESSED sensor_msgs/Image only - LABS rejects CompressedImage outright.
VIDEO_TOPICS=(
  /wrist_camera_left/camera/color/image_raw
  /wrist_camera_right/camera/color/image_raw
  /head_camera/zed_node/rgb/image_rect_color
)
# Not in the LABS manifest; recorded for our own use. Names inferred from the namespaces in
# franka_mobile_sensors' default_sensor_suite.yaml - verify with --check.
LIDAR_TOPICS=(
  /lidar_front/scan
  /lidar_rear/scan
)
# Archived: useful context, dropped at dataset-build time.
EXTRA_TOPICS=(
  /tf
  /tf_static
  /pedal/state
  /teleop/pedal_mode
  /swerve_drive_controller/odometry
)

mode=record; want_video=true; out=""; name=""
while [ $# -gt 0 ]; do
  case "$1" in
    --check)    mode=check; shift ;;
    --no-video) want_video=false; shift ;;
    --out)      out="$2"; shift 2 ;;
    --name)     name="$2"; shift 2 ;;
    -h|--help)  sed -n '2,30p' "$0"; exit 0 ;;
    *) echo "unknown argument: $1" >&2; exit 2 ;;
  esac
done

topics=("${STATE_TOPICS[@]}" "${ACTION_TOPICS[@]}" "${LIDAR_TOPICS[@]}" "${EXTRA_TOPICS[@]}")
$want_video && topics+=("${VIDEO_TOPICS[@]}")

# Required = anything whose absence breaks the LeRobot conversion. The extras are not.
required=("${STATE_TOPICS[@]}" "${ACTION_TOPICS[@]}")
$want_video && required+=("${VIDEO_TOPICS[@]}")

prelude="source /opt/ros/humble/setup.bash \
  && source /opt/ros/humble/franka/setup.bash \
  && cd '$repo_ctr' && source install/setup.bash \
  && export ROS_DOMAIN_ID=$ROS_DOMAIN_ID RMW_IMPLEMENTATION=rmw_fastrtps_cpp PYTHONUNBUFFERED=1"

dex()  { docker exec -u "$(id -u):20" -e HOME=/tmp "$CONTAINER" bash -lc "$1"; }

[ "$(docker inspect -f '{{.State.Running}}' "$CONTAINER" 2>/dev/null)" = true ] || {
  echo "ERROR: container '$CONTAINER' is not running. Start it with ./start_teleop.bash" >&2
  exit 1
}

# ------------------------------------------------------------------- check
# One `ros2 topic list` for the whole manifest: a topic list per topic would be a DDS
# discovery burst per call, and those bursts abort running FCI control loops.
echo "Discovering topics on domain $ROS_DOMAIN_ID (Fast DDS is slow here, ~15-25 s)..."
live="$(dex "$prelude && timeout 60 ros2 topic list --no-daemon 2>/dev/null" || true)"
if [ -z "$live" ]; then
  echo "ERROR: no topics discovered at all. Is anything running?" >&2
  exit 1
fi

missing=(); present=0
printf '\n%-64s %s\n' "TOPIC" "STATUS"
for t in "${topics[@]}"; do
  if grep -qxF "$t" <<<"$live"; then
    printf '%-64s %s\n' "$t" "ok"; present=$((present+1))
  else
    printf '%-64s %s\n' "$t" "MISSING"
    for r in "${required[@]}"; do [ "$r" = "$t" ] && missing+=("$t"); done
  fi
done
printf '\n%d/%d topics present\n' "$present" "${#topics[@]}"

if [ "${#missing[@]}" -gt 0 ]; then
  echo
  echo "${#missing[@]} REQUIRED topic(s) missing:" >&2
  printf '  %s\n' "${missing[@]}" >&2
  echo >&2
  echo "Recording now would produce an episode that cannot be converted: LABS fails the" >&2
  echo "WHOLE episode if any configured topic is more than 1 s short of the bounds." >&2
  echo "Start the missing publisher, or use --no-video if only cameras are missing." >&2
  [ "$mode" = check ] || exit 1
fi
[ "$mode" = check ] && exit 0

# ------------------------------------------------------------------ record
stamp="$(date +%Y%m%d-%H%M%S)"
[ -n "$name" ] && stamp="${stamp}_${name}"
out="${out:-$HOME/teleop_bags/$stamp}"
case "$out" in
  "$HOME"/*) out_ctr="/workspace/${out#"$HOME"/}" ;;
  *) echo "ERROR: --out must be under \$HOME so the container can see it" >&2; exit 1 ;;
esac
mkdir -p "$(dirname "$out")"

echo
echo "=============================================================="
echo " Recording -> $out"
echo " ${#topics[@]} topics, video=$want_video"
echo " Ctrl+C to stop."
echo "=============================================================="
echo

# MCAP because that is what LABS' lerobot_mcap_reader consumes; sqlite3 bags would need a
# re-encode before conversion.
storage="-s mcap"
if ! dex "$prelude && ros2 bag record --help 2>&1 | grep -q mcap" >/dev/null 2>&1; then
  echo "NOTE: rosbag2_storage_mcap not found; falling back to the sqlite3 default." >&2
  echo "      Install with: sudo apt install ros-humble-rosbag2-storage-mcap" >&2
  storage=""
fi

cleanup() { trap - INT TERM; echo; echo "stopping..."; }
trap cleanup INT TERM
docker exec -it -u "$(id -u):20" -e HOME=/tmp "$CONTAINER" bash -lc \
  "$prelude && exec ros2 bag record $storage -o '$out_ctr' ${topics[*]}" || true

echo
if [ -d "$out" ]; then
  echo "Bag: $out  ($(du -sh "$out" 2>/dev/null | cut -f1))"
  echo "Inspect with:"
  echo "  docker exec -u $(id -u):20 -e HOME=/tmp $CONTAINER bash -lc \\"
  echo "    'source /opt/ros/humble/setup.bash && ros2 bag info $out_ctr'"
else
  echo "No bag was written." >&2
fi
