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
  # NOT gripper_joint_states: robotiq_controllers.yaml nests `publish_topic` inside the
  # controller_manager ros__parameters block, where the broadcaster never reads it, so the
  # name never takes effect. Verified on hardware 2026-08-23 - the real topic is
  # joint_states. LABS' config_data_recorder.yml still says gripper_joint_states and would
  # therefore silently drop both gripper state columns.
  /left/gripper/joint_states
  /right/franka_robot_state_broadcaster/measured_joint_states
  /right/franka_robot_state_broadcaster/external_joint_torques
  /right/franka_robot_state_broadcaster/external_wrench_in_stiffness_frame
  /right/gripper/joint_states
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

# DEFAULT TRANSPORTS, deliberately - everything must share one transport world.
#
# The wrist cameras used to run with useBuiltinTransports=false and UDP whitelisted to the
# wired 172.16.16.140. A default-transport participant then could not discover them AT ALL:
# their topics were simply absent, with no error. Pinning the recorder to match found the
# cameras but lost the teleop nodes instead, and adding SHM did not bridge the two - SHM
# cannot cross a container boundary, and the cameras run in their own container with their
# own /dev/shm. The fix was on the camera side: drop its custom profile so it uses the same
# default transports as the robot stack and the teleop nodes.
#
# Opt into a profile with TMR_DDS_PROFILE=<path>, but check what it does to discovery first.
dds_host="${TMR_DDS_PROFILE-}"
dds_env=""
if [ -n "$dds_host" ]; then
  [ -f "$dds_host" ] || { echo "ERROR: DDS profile not found: $dds_host" >&2; exit 1; }
  case "$dds_host" in
    "$HOME"/*) dds_ctr="/workspace/${dds_host#"$HOME"/}" ;;
    *) echo "ERROR: DDS profile must live under \$HOME to be visible in the container" >&2; exit 1 ;;
  esac
  dds_env="&& export FASTRTPS_DEFAULT_PROFILES_FILE='$dds_ctr' FASTDDS_DEFAULT_PROFILES_FILE='$dds_ctr'"
fi

prelude="source /opt/ros/humble/setup.bash \
  && source /opt/ros/humble/franka/setup.bash \
  && cd '$repo_ctr' && source install/setup.bash \
  && export ROS_DOMAIN_ID=$ROS_DOMAIN_ID RMW_IMPLEMENTATION=rmw_fastrtps_cpp PYTHONUNBUFFERED=1 $dds_env"

dex()  { docker exec -u "$(id -u):20" -e HOME=/tmp "$CONTAINER" bash -lc "$1"; }

[ "$(docker inspect -f '{{.State.Running}}' "$CONTAINER" 2>/dev/null)" = true ] || {
  echo "ERROR: container '$CONTAINER' is not running. Start it with ./start_teleop.bash" >&2
  exit 1
}

# ------------------------------------------------------------------- check
# One `ros2 topic list` for the whole manifest: a topic list per topic would be a DDS
# discovery burst per call, and those bursts abort running FCI control loops.
echo "Discovering topics on domain $ROS_DOMAIN_ID (spinning ${TMR_SPIN_TIME:-25}s; Fast DDS is slow here)..."
# `ros2 topic list -v`, not plain list: a bare list includes topics that only have a
# SUBSCRIBER, so a robot-side controller listening for /left/gello/joint_states made the
# topic look healthy while nothing published it. Only the "Published topics:" section counts.
# --spin-time is essential, not a nicety. `ros2 topic list` spins ~1 s by default and
# then reports, but Fast DDS discovery on this machine takes 15-25 s, so the default
# reports a half-discovered graph: nodes that were definitely publishing showed up as
# MISSING while a latched topic happened to arrive in time.
spin="${TMR_SPIN_TIME:-25}"
raw="$(dex "$prelude && timeout $((spin + 45)) ros2 topic list -v --spin-time $spin --no-daemon 2>/dev/null" || true)"
live="$(awk '/^Published topics:/{p=1;next} /^Subscribed topics:/{p=0} p && /^ \* /{print $2}' <<<"$raw")"
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
