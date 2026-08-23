#!/usr/bin/env bash
# Start the LAPTOP half of TMR teleoperation: GELLO leaders + foot pedals.
# The robot half is ~/start_robot.bash ON THE ROBOT. Start the robot first.
#
# WHY THIS RUNS IN A CONTAINER
# ----------------------------
# This laptop runs ROS 2 Kilted; the robot runs Humble. ROS 2 does not support
# cross-distro communication: from the Kilted host, `ros2 node list` comes back EMPTY and
# every controller_manager service call times out, even though topics partially discover
# (differing type hashes / service ABI). The same calls succeed from the Humble image, so
# the laptop-side nodes run there, on the host network.
#
# Do NOT "fix" this by switching the laptop to CycloneDDS because it connects faster.
# The robot speaks Fast DDS; mixing vendors breaks services and actions, which is exactly
# what the spine bridge needs. Fast DDS taking 15-25 s to create a node on this machine is
# a slow start, not a failure - be patient rather than changing RMW.
#
# DEVICE PERMISSIONS
#   * GELLO leaders are /dev/ttyACM* (root:dialout 0660). A privileged container gets a
#     FRESH /dev, so the host ACL granting your user access does NOT carry over. That is
#     why the exec user is <uid>:20 - primary group dialout - and not <uid>:<gid>.
#   * The foot switches need configs/99-pcsensor-footswitch.rules (MODE 0666) installed.
#     The shipped rule pins ID_PATH to one machine's USB ports; pedal_state_publisher
#     autodetects by vendor:product, so only the 0666 mode actually matters.
#
# CLOCK SKEW IS THE #1 CAUSE OF "IT STARTED BUT NOTHING MOVES"
#   SwerveDriveController silently discards cmd_vel older than 0.5 s (no error, anywhere),
#   and JointImpedanceController rejects stale GELLO samples. This script warns; fix with
#   ./configs/sync_robot_clock.sh.
#
# USAGE
#   ./start_teleop.bash                       # restart both stacks, motion only
#   ./start_teleop.bash --record --task-id <uuid>
#   ./start_teleop.bash --pedal-fg            # pedal stack in the FOREGROUND, so
#                                             # keyboard_state_publisher gets a TTY and
#                                             # 'm' / w,a,s,d,q,e work
#   ./start_teleop.bash -d                    # detach and return to the prompt
#   ./start_teleop.bash stop | status | logs [gello|pedal]
#
# By default it stays in the FOREGROUND following both logs; Ctrl+C stops both stacks.
set -Eeuo pipefail

IMAGE="${TMR_IMAGE:-teleoperation_devcontainer-gello-ros2:latest}"
CONTAINER="${TMR_CONTAINER:-gello-humble}"
GELLO_CFG="${TMR_GELLO_CFG:-franka_gello_duo.yaml}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"

repo_host="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# Mirror .devcontainer/docker-compose.yml, which mounts the repo's GRANDPARENT at
# /workspace. Deriving the container path keeps this working if the repo is renamed.
case "$repo_host" in
  "$HOME"/*) repo_ctr="/workspace/${repo_host#"$HOME"/}" ;;
  *) echo "ERROR: expected the repo under \$HOME ($HOME), got $repo_host" >&2; exit 1 ;;
esac

cmd=start; record=false; task_id=""; pedal_fg=false; log_which=""; detach=false
while [ $# -gt 0 ]; do
  case "$1" in
    start|stop|status) cmd="$1"; shift ;;
    logs) cmd=logs; shift; case "${1:-}" in gello|pedal) log_which="$1"; shift ;; esac ;;
    --record)   record=true; shift ;;
    --task-id)  task_id="$2"; record=true; shift 2 ;;
    --pedal-fg) pedal_fg=true; shift ;;
    -d|--detach) detach=true; shift ;;
    -h|--help)  sed -n '2,45p' "$0"; exit 0 ;;
    *) echo "unknown argument: $1" >&2; exit 2 ;;
  esac
done

# Keep every byte of DDS on WiFi so the robot's wired 172.16.16.x link carries ONLY its
# three 1 kHz Franka FCI streams. Sharing that link aborted the base and BOTH arms with
# ["communication_constraints_violation"] within 10 ms of each other on 2026-08-23, after
# which ros2_control demotes the hardware to `unconfigured` while the controllers still
# report `active` - GELLO and pedal commands then go nowhere, silently.
#
# The robot must run its matching ~/fastdds_wifi.xml (start_robot.bash exports it) or DDS
# still reaches the wired NIC from that side and the problem remains.
#
# Escape hatch: TMR_DDS_PROFILE="" for default all-interface discovery, or a path to
# another profile. It must live under $HOME to be visible inside the container.
# DEFAULT IS EMPTY: all-interface discovery, matching the robot side (fixed in 5a78c47).
#
# This used to default to configs/fastdds_laptop_wifi.xml, which pins DDS to the WiFi
# address ONLY. That silently partitioned the laptop: teleop on 192.168.50.117 could
# not see the wrist cameras, which pin to the wired 172.16.16.140 - so no single
# `ros2 bag record` could ever capture both. Reserving the wired link for FCI is still
# the right idea, but it needs every participant on a matching profile, which is not
# the case today. Opt in with TMR_DDS_PROFILE=<path> once that is sorted.
dds_host="${TMR_DDS_PROFILE-}"
dds_env=""
if [ -n "$dds_host" ]; then
  [ -f "$dds_host" ] || { echo "ERROR: DDS profile not found: $dds_host" >&2; exit 1; }
  case "$dds_host" in
    "$HOME"/*) dds_ctr="/workspace/${dds_host#"$HOME"/}" ;;
    *) echo "ERROR: DDS profile must be under \$HOME to be visible in the container" >&2; exit 1 ;;
  esac
  dds_env="&& export FASTRTPS_DEFAULT_PROFILES_FILE='$dds_ctr'"
fi

# Env prelude shared by every exec. local_setup vs setup matters less here than on the
# robot, but /opt/ros/humble/franka carries libfranka + franka_msgs and must come first.
prelude="source /opt/ros/humble/setup.bash \
  && source /opt/ros/humble/franka/setup.bash \
  && cd '$repo_ctr' && source install/setup.bash \
  && export ROS_DOMAIN_ID=$ROS_DOMAIN_ID RMW_IMPLEMENTATION=rmw_fastrtps_cpp PYTHONUNBUFFERED=1 $dds_env"

dex()  { docker exec -u "$(id -u):20" -e HOME=/tmp "$CONTAINER" bash -lc "$1"; }
dexd() { docker exec -d -u "$(id -u):20" -e HOME=/tmp "$CONTAINER" bash -lc "$1"; }

container_running() { [ "$(docker inspect -f '{{.State.Running}}' "$CONTAINER" 2>/dev/null)" = true ]; }

stop_stacks() {
  container_running || return 0
  # SIGINT to the launch leaders first, so they shut their own children down in order.
  docker exec -u "$(id -u):20" "$CONTAINER" bash -lc '
    for p in $(pgrep -f "ros2 launch" 2>/dev/null); do kill -INT $p 2>/dev/null || true; done
    for _ in $(seq 1 20); do pgrep -f "ros2 launch" >/dev/null 2>&1 || break; sleep 0.5; done
    # Orphaned nodes keep the serial ports and evdev grabs, so the next start fails with
    # "Could not open port" / "no candidate device found". Escalate rather than leave them.
    pkill -KILL -f "gello_publisher|pedal_state_publisher|base_bridge|spine_bridge|mode_manager|labs_pedal_bridge|mobile_base_state_bridge|spine_state_publisher|keyboard_state_publisher" 2>/dev/null || true
    exit 0' >/dev/null 2>&1 || true
}

case "$cmd" in
  stop)
    stop_stacks; echo "Laptop teleop stacks stopped (container '$CONTAINER' left running)."; exit 0 ;;
  status)
    container_running || { echo "container '$CONTAINER': NOT running"; exit 1; }
    echo "container '$CONTAINER': running"
    # grep -v "bash -lc" drops this command's own wrapper shell, which contains the
    # pattern string and would otherwise always match.
    dex 'pgrep -af "gello_publisher|pedal_state_publisher|base_bridge|spine_bridge|mode_manager|labs_pedal_bridge" \
         | grep -v "bash -lc" | sed "s/ --ros-args.*//;s|.*/||" | sed "s/^/  /" \
         | grep . || echo "  (no teleop nodes running)"'
    exit 0 ;;
  logs)
    case "$log_which" in
      gello) dex 'tail -n 40 -f /tmp/gello.log' ;;
      pedal) dex 'tail -n 40 -f /tmp/pedal.log' ;;
      *)     dex 'tail -n 20 /tmp/gello.log /tmp/pedal.log | cat -v | cut -c1-160' ;;
    esac
    exit 0 ;;
esac

# ------------------------------------------------------------------ container
if ! container_running; then
  echo "Starting container '$CONTAINER'..."
  docker rm -f "$CONTAINER" >/dev/null 2>&1 || true
  # privileged + host network: privileged for /dev (GELLO serial, evdev foot switches),
  # host network because DDS discovery to the robot must not be NATed.
  # X11 passthrough so rviz2 / rqt_image_view can display on the host. Harmless when there
  # is no display (the vars are simply empty). GL works because this container is
  # privileged with /dev mounted, so it reaches /dev/dri; without that, rviz2 connects to
  # the X server and then fails at "libGL error: glx: failed to create dri3 screen".
  x11_args=()
  if [ -d /tmp/.X11-unix ] && [ -n "${DISPLAY:-}" ]; then
    x11_args=(-e "DISPLAY=$DISPLAY" -e QT_X11_NO_MITSHM=1 -v /tmp/.X11-unix:/tmp/.X11-unix)
    [ -f "$HOME/.Xauthority" ] && x11_args+=(-v "$HOME/.Xauthority:/tmp/.Xauthority:ro" -e XAUTHORITY=/tmp/.Xauthority)
  fi
  docker run -d --name "$CONTAINER" --privileged --network host --init \
    "${x11_args[@]}" \
    -v "$HOME:/workspace" \
    -v /dev/serial/by-id:/dev/serial/by-id \
    -e ROS_DOMAIN_ID="$ROS_DOMAIN_ID" -e RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
    "$IMAGE" sleep infinity >/dev/null
fi

# These are pip-installed into the running container, so they do NOT survive `docker rm`.
# rosbag2's mcap storage plugin, needed by record_bag.bash: LABS' lerobot_mcap_reader
# consumes MCAP, and the sqlite3 default would need a re-encode before conversion. Like the
# pip deps below, an apt install into the running container does NOT survive `docker rm`.
if ! dex 'ls /opt/ros/humble/share | grep -q rosbag2_storage_mcap' >/dev/null 2>&1; then
  echo "Installing rosbag2 mcap storage into the container..."
  docker exec -u 0 "$CONTAINER" bash -lc \
    "apt-get update -qq && apt-get install -y -qq ros-humble-rosbag2-storage-mcap" >/dev/null 2>&1 || true
fi

if ! dex 'python3 -c "import evdev, dynamixel_sdk, requests"' >/dev/null 2>&1; then
  echo "Installing missing Python deps into the container..."
  docker exec -u 0 "$CONTAINER" bash -lc \
    "pip3 install -q -r '$repo_ctr/requirements.txt' requests" >/dev/null 2>&1 || true
fi

if ! dex "[ -f '$repo_ctr/install/setup.bash' ]" 2>/dev/null; then
  echo "Workspace not built for Humble; building..."
  # Build as your uid, NOT root: root builds through the bind mount are what leave
  # build/ install/ log/ owned by root and break the next host-side colcon build.
  dex "source /opt/ros/humble/setup.bash && source /opt/ros/humble/franka/setup.bash \
       && cd '$repo_ctr' \
       && colcon build --symlink-install --packages-skip franka_fr3_arm_controllers franka_gripper_manager" \
    | tail -3
fi

# -------------------------------------------------------------------- clock
if [ -x "$repo_host/configs/sync_robot_clock.sh" ] \
   && ssh -o BatchMode=yes -o ConnectTimeout=5 "${TMR_HOST:-companion}" true 2>/dev/null; then
  skew_line="$("$repo_host/configs/sync_robot_clock.sh" --check 2>/dev/null | grep robot || true)"
  [ -n "$skew_line" ] && echo "Clock:$skew_line"
  case "$skew_line" in
    *NEEDS\ CORRECTION*)
      # auto-sync: skew > 0.5 s makes the base silently discard cmd_vel and the arms reject
      # GELLO samples, with nothing logged. Correcting it is not optional, so do not make
      # the operator remember it. Passwordless via /usr/local/sbin/tmr-set-clock on the
      # robot; without that sudoers rule this falls back to prompting, so it is bounded.
      echo "  correcting..."
      if timeout 90 "$repo_host/configs/sync_robot_clock.sh" </dev/null 2>&1 | grep -E "robot is|Clock synced"; then
        :
      else
        echo "  -> automatic sync failed. Run ./configs/sync_robot_clock.sh by hand," >&2
        echo "     or the base will ignore cmd_vel silently." >&2
      fi
      ;;
  esac
else
  echo "Clock: skipped (no passwordless ssh to ${TMR_HOST:-companion})."
  echo "  Set it up once with:  ssh-copy-id ${TMR_HOST:-companion}"
  echo "  Until then check by hand: ./configs/sync_robot_clock.sh --check"
fi

# ------------------------------------------------------------------- restart
stop_stacks

if [ -n "$dds_host" ]; then echo "DDS: $dds_host (WiFi only; robot wired link reserved for FCI)"; \
else echo "DDS: default discovery (all interfaces)"; fi
echo "Starting GELLO leaders ($GELLO_CFG)..."
dexd "$prelude && exec ros2 launch franka_gello_state_publisher main.launch.py \
      config_file:=$GELLO_CFG > /tmp/gello.log 2>&1"

pedal_args="record:=$($record && echo true || echo false)"
[ -n "$task_id" ] && pedal_args="$pedal_args task_id:=$task_id"

if $pedal_fg; then
  echo "Waiting for GELLO to come up (Fast DDS start is slow here)..."
  sleep 20
  dex "grep -aiE 'error|died' /tmp/gello.log | tail -5" || true
  echo
  echo "Starting pedal stack in the FOREGROUND. This terminal must keep focus for 'm'"
  echo "and w/a/s/d/q/e to reach keyboard_state_publisher. Ctrl+C stops the pedal stack"
  echo "(GELLO keeps running; use './start_teleop.bash stop' for both)."
  echo
  exec docker exec -it -u "$(id -u):20" -e HOME=/tmp "$CONTAINER" bash -lc \
    "$prelude && exec ros2 launch tmr_pedal_teleop mobile_teleop.launch.py $pedal_args"
fi

echo "Starting pedal stack ($pedal_args)..."
dexd "$prelude && exec ros2 launch tmr_pedal_teleop mobile_teleop.launch.py $pedal_args > /tmp/pedal.log 2>&1"

# Poll instead of sleeping a fixed 25 s: Fast DDS node creation is slow here, but how slow
# varies, and waiting the worst case every time wastes most of a minute per restart.
# Returns as soon as both stacks report ready; caps so a genuine failure still surfaces.
ready_wait() {
  local deadline=$(( SECONDS + ${TMR_READY_TIMEOUT:-30} ))
  while (( SECONDS < deadline )); do
    if dex 'grep -aq "Pedal publisher started" /tmp/pedal.log && grep -aq "gripper=" /tmp/gello.log' 2>/dev/null; then
      return 0
    fi
    sleep 1
  done
  echo "  (still not ready after ${TMR_READY_TIMEOUT:-30}s - showing the logs anyway)" >&2
  return 1
}
echo "Waiting for both stacks..."
ready_wait || true

echo
echo "=== GELLO ==="
dex 'grep -aiE "error|died" /tmp/gello.log | tail -5 || true; tail -2 /tmp/gello.log | cut -c1-100'
echo "=== PEDAL ==="
dex 'grep -aE "Pedal publisher started|Foot switch [12]:|base_bridge ->|PEDAL MODE|spine_bridge ready" /tmp/pedal.log | tail -8'
echo
echo "keyboard_state_publisher dies without a TTY - expected when not using --pedal-fg."
echo
echo "Arms stay INACTIVE by design. With both arms at the home pose and hands OFF the"
echo "GELLOs, activate impedance control from inside the container:"
echo "  ros2 control set_controller_state joint_impedance_controller active -c /left/controller_manager"
echo "  ros2 control set_controller_state joint_impedance_controller active -c /right/controller_manager"

if $detach; then
  echo
  echo "Detached. Logs: ./start_teleop.bash logs [gello|pedal]   Stop: ./start_teleop.bash stop"
  exit 0
fi

# Foreground by default. Detached start left no visible log and no way to Ctrl+C, so the
# only way to stop was a second, separate invocation. Follow both logs here and make Ctrl+C
# a clean shutdown of BOTH stacks.
echo
echo "=============================================================="
echo " Following both logs. Ctrl+C stops BOTH stacks."
echo " (-d starts detached; --pedal-fg gives the pedal stack a real"
echo "  TTY so 'm' and w/a/s/d/q/e reach keyboard_state_publisher.)"
echo "=============================================================="
echo
cleanup_fg() {
  trap - INT TERM
  echo
  echo "Stopping teleop stacks..."
  # The follower runs INSIDE the container and would outlive this script otherwise.
  docker exec "$CONTAINER" bash -lc 'pkill -f "tail -n 20 -F /tmp/gello.log" || true' >/dev/null 2>&1 || true
  stop_stacks
  echo "stopped."
  exit 0
}
trap cleanup_fg INT TERM
dex 'tail -n 20 -F /tmp/gello.log /tmp/pedal.log' || true
cleanup_fg
