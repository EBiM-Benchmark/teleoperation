#!/usr/bin/env bash
# Start ALL robot-side stacks for TMR teleoperation, in the required order:
#   base -> spine -> arms -> grippers
#
# Run this ON THE ROBOT after a reboot. The laptop half is ./start_teleop.bash.
#
# Deliberately does NOT move anything: the arm impedance controllers spawn inactive,
# and sending the arms to the home pose stays a manual, supervised step.

set -Eeuo pipefail

ws_dir="${TMR_WS:-$HOME/tams_ws}"
spine_ip="${TMR_SPINE_IP:-172.16.16.10}"
# Laptop and robot MUST share a domain. Nothing in the robot's rc files sets one, so 0 is
# what every ROS 2 process here lands on unless something overrides it - and 0 is what the
# rest of the system already assumes: labs_integration/tmr_station/docker-compose.yml,
# fastdds_labs.xml, configs/fastdds_laptop_discovery.xml, .devcontainer/docker-compose.yml
# and the Olive sensors are all on 0. Keep this in step with configs/tmr_laptop_env.sh.
export ROS_DOMAIN_ID="${TMR_ROS_DOMAIN_ID:-0}"

# Keep every byte of DDS on WiFi so the wired 172.16.16.x link carries ONLY the three
# 1 kHz Franka FCI streams. Sharing it is what aborted the base and BOTH arms with
# ["communication_constraints_violation"] on 2026-08-23, within 10 ms of each other.
# See ~/fastdds_wifi.xml for the full reasoning.
#
# Escape hatch: TMR_DDS_PROFILE="" falls back to default all-interface discovery,
# TMR_DDS_PROFILE=/path/to.xml uses another profile.
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
# DEFAULT IS EMPTY: default all-interface discovery, which is what actually works.
#
# ~/fastdds_wifi.xml is OPT-IN and currently BROKEN: its initialPeersList destroys the
# robot's own LOCAL discovery, because unicast initial peers probe only a small range of
# participant indices per address and this robot runs well over a dozen participants. The
# symptom is spawners unable to contact their own controller_manager:
#   [spawner-3] Could not contact service /left/gripper/controller_manager/list_controllers
#
# Reserving the wired link for FCI is still the right idea - see docs/RUNBOOK.md S9 - but
# do not enable this until the profile is fixed and tested on throwaway nodes:
#   TMR_DDS_PROFILE=$HOME/fastdds_wifi.xml ~/start_robot.bash --restart
export FASTRTPS_DEFAULT_PROFILES_FILE="${TMR_DDS_PROFILE-}"
if [ -z "$FASTRTPS_DEFAULT_PROFILES_FILE" ]; then
  unset FASTRTPS_DEFAULT_PROFILES_FILE
  echo "DDS: default discovery (all interfaces)."
elif [ ! -f "$FASTRTPS_DEFAULT_PROFILES_FILE" ]; then
  echo "ERROR: DDS profile not found: $FASTRTPS_DEFAULT_PROFILES_FILE" >&2
  exit 1
else
  echo "DDS: $FASTRTPS_DEFAULT_PROFILES_FILE (WiFi only; wired link reserved for FCI)"
fi
# The ros2 CLI daemon caches DDS settings, so a stale one would keep using the old
# transports and report a graph that does not match what the nodes actually see.
ros2 daemon stop >/dev/null 2>&1 || true

skip_arms=false
home_pose=true
sensors=true
activate=true
home_file="${TMR_HOME_POSE:-$HOME/teleop_home_pose.yaml}"
restart=false
for arg in "$@"; do
  case "$arg" in
    --skip-arms) skip_arms=true ;;
    # Stop any already-running robot stacks instead of refusing to start.
    --restart)   restart=true ;;
    --no-home)   home_pose=false ;;
    --no-sensors) sensors=false ;;
    --no-activate) activate=false ;;
    *) echo "Usage: $0 [--skip-arms] [--restart] [--no-home] [--no-activate] [--no-sensors]" >&2; exit 2 ;;
  esac
done

# ---------------------------------------------------------------- environment
# NOTE: local_setup.bash, never setup.bash. setup.bash chains to the workspace's
# recorded parent (~/ros2_ws, a ~4-month-stale franka stack) and re-injects it as an
# overlay; colcon prepends only-if-absent, so once it is in front it stays in front
# and franka_bringup resolves to the wrong workspace.
set +u
source /opt/ros/humble/setup.bash
source "$ws_dir/install/local_setup.bash"
set -u

if ! ros2 pkg prefix franka_bringup 2>/dev/null | grep -q "$ws_dir"; then
  echo "franka_bringup does not resolve to $ws_dir - the stale ~/ros2_ws overlay is in front." >&2
  echo "Open a genuinely new terminal (exec bash inherits AMENT_PREFIX_PATH) and retry." >&2
  exit 1
fi

# rviz2 is part of the base launch and aborts (SIGABRT) with no X display, e.g. over
# plain ssh. Headless keeps it from dying noisily; it does not affect teleop either way.
if [[ -z "${DISPLAY:-}" ]]; then
  export QT_QPA_PLATFORM=offscreen
fi

# ------------------------------------------------------------- already running?
# A leftover bringup is not harmless: its controller_manager answers the new launch's
# spawners, which then see "Controller already loaded" and fail to configure. That is
# exactly how joint_state_broadcaster died on 2026-08-20.
stale_pattern='ros2_control_node|tmrv0_2.launch|spine.launch|franka_fr3_arm_controllers.launch|robotiq_gripper_controller_client.launch'

# The stale_pattern above matches LAUNCH processes and ros2_control_node, but the NODES
# those launches spawn have their own executable names and match none of it. Kill a launch
# (or lose its terminal) and those nodes are reparented to init, where they survive every
# later --restart. Observed 2026-08-23: six orphaned robotiq_gripper_client and one
# franka_spine_server, the oldest 8 h old, plus five `timeout ... ros2 service call` pairs
# leaked by wait_for_controller below.
#
# They are not just idle CPU. A second spine_action_server serves the SAME action name,
# and every orphaned gripper client subscribes to the same command topics and contends for
# the gripper serial ports. The load they add is also what starves the base's 1 kHz FCI
# loop into a communication_constraints_violation reflex.
#
# Deliberately narrow: PPID 1 (genuinely orphaned) AND owned by this user AND matching a
# node this script starts. A process whose launch parent is still alive is never touched.
orphan_pattern='tams_ws/install/(franka_gripper_manager|franka_spine_server)|robotiq_gripper_client|spine_action_server|/(robot_state_publisher|joint_state_publisher|rviz2)|ros2 service call .*controller_manager'

list_orphans() {
  # `|| true` throughout: grep exits 1 on no-match and `set -o pipefail` would abort.
  ps -eo pid,ppid,user:24,args --no-headers 2>/dev/null \
    | awk -v me="$(id -un)" '$2==1 && $3==me' \
    | grep -E "$orphan_pattern" \
    | awk '{print $1}' || true
}

sweep_orphans() {
  local pids sig p
  for sig in INT TERM KILL; do
    pids="$(list_orphans)"
    [ -z "$pids" ] && return 0
    echo "  orphaned nodes, sending SIG$sig: $(echo $pids | tr '\n' ' ')"
    for p in $pids; do kill "-$sig" "$p" 2>/dev/null || true; done
    # Reaping is not instant; the first sweep of 2026-08-23 looked like a failure purely
    # because it re-checked after 2 s.
    for _ in $(seq 1 16); do
      [ -z "$(list_orphans)" ] && break
      sleep 0.5
    done
  done
  pids="$(list_orphans)"
  [ -n "$pids" ] && echo "  WARNING: orphans survived SIGKILL: $(echo $pids | tr '\n' ' ')" >&2
  return 0
}
existing="$(pgrep -af "$stale_pattern" || true)"
if [[ -n "$existing" ]]; then
  if ! $restart; then
    echo "Robot stacks are already running; refusing to start a second set:" >&2
    echo "$existing" >&2
    echo >&2
    echo "Stop them first (Ctrl+C in their terminal), or re-run with --restart to have" >&2
    echo "this script stop them for you." >&2
    exit 1
  fi

  echo "Stopping the running robot stacks (--restart)..."
  echo "$existing" >&2
  # SIGINT is what ros2 launch expects: it shuts its own children down in order.
  # Signal launch processes first so they can clean up their ros2_control_node.
  for pid in $(pgrep -f 'tmrv0_2.launch|spine.launch|franka_fr3_arm_controllers.launch|robotiq_gripper_controller_client.launch' || true); do
    kill -INT "$pid" 2>/dev/null || true
  done
  for _ in {1..60}; do
    pgrep -f "$stale_pattern" >/dev/null 2>&1 || break
    sleep 0.5
  done
  # Anything still holding on after 30 s is an orphan (typically a ros2_control_node
  # whose launch parent already exited). It owns the FCI/base connection, so the new
  # bringup cannot start until it is gone.
  if pgrep -f "$stale_pattern" >/dev/null 2>&1; then
    echo "  some processes ignored SIGINT; escalating to SIGTERM." >&2
    pkill -TERM -f "$stale_pattern" 2>/dev/null || true
    for _ in {1..30}; do
      pgrep -f "$stale_pattern" >/dev/null 2>&1 || break
      sleep 0.5
    done
  fi
  if pgrep -f "$stale_pattern" >/dev/null 2>&1; then
    echo "  some orphaned processes ignored SIGTERM; escalating to SIGKILL." >&2
    pkill -KILL -f "$stale_pattern" 2>/dev/null || true
    for _ in {1..20}; do
      pgrep -f "$stale_pattern" >/dev/null 2>&1 || break
      sleep 0.25
    done
  fi
  if pgrep -f "$stale_pattern" >/dev/null 2>&1; then
    echo "Could not stop the existing robot stacks even with SIGKILL:" >&2
    pgrep -af "$stale_pattern" >&2
    exit 1
  fi
  # Reparented nodes the stale_pattern cannot see.
  sweep_orphans
  echo "  stopped."
  # The hardware needs a moment to drop the previous FCI/base session.
  sleep 3
fi

# -------------------------------------------------------------------- lifecycle
launch_pids=()
launch_names=()
# Sensors go here instead of launch_pids. They are stopped on exit like everything else, but
# their liveness is NOT monitored: on 2026-08-23 a ZED that could not find its camera exited,
# wait_any_launch saw the stage die, and the EXIT trap tore down the ENTIRE robot - base,
# arms, grippers and spine - because one camera was unplugged. A sensor must never do that.
aux_pids=()
aux_names=()

stop_all() {
  trap - EXIT INT TERM
  # Sensors first: they are pure publishers, and stopping them early quiets the graph while
  # the control stacks shut down.
  for pid in "${aux_pids[@]:-}"; do
    [ -n "$pid" ] && kill -TERM -- "-$pid" 2>/dev/null || true
  done
  for pid in "${launch_pids[@]}"; do
    kill -TERM -- "-$pid" 2>/dev/null || true
  done
  # The launch leader can exit before ros2_control children; test the process group, not only its PID.
  for _ in {1..60}; do
    local alive=false
    for pid in "${launch_pids[@]}"; do
      kill -0 -- "-$pid" 2>/dev/null && alive=true
    done
    $alive || break
    sleep 0.25
  done
  for pid in "${launch_pids[@]}"; do
    kill -0 -- "-$pid" 2>/dev/null && kill -KILL -- "-$pid" 2>/dev/null || true
    wait "$pid" 2>/dev/null || true
  done
}
trap stop_all EXIT INT TERM

# Wait until any launch stage exits, and say WHICH one.
#
# `wait -n "${launch_pids[@]}"` fails with "no such job" once bash has reaped a pid it no
# longer tracks. Under `set -e` plus the EXIT trap that stops every stack, that failure
# tore the whole robot down right after a successful bringup on 2026-08-23, reporting only
# a bare pid. A stage really had died - the message just made it look like a bash bug.
wait_any_launch() {
  local i pid rc live
  while :; do
    live=()
    for i in "${!launch_pids[@]}"; do
      pid="${launch_pids[$i]}"
      if kill -0 "$pid" 2>/dev/null; then
        live+=("$pid")
      else
        echo "The '\''${launch_names[$i]}'\'' stage (pid $pid) is gone." >&2
        return 1
      fi
    done
    (( ${#live[@]} )) || { echo "No launch stages are running." >&2; return 1; }
    set +e
    wait -n "${live[@]}"
    rc=$?
    set -e
    # 127 = "no such job": the pid vanished between the check above and the wait. Loop and
    # re-check rather than reporting it as a stage failure.
    (( rc == 127 )) && { sleep 1; continue; }
    return "$rc"
  done
}

# wait_for <kind> <pattern> <timeout_s> <description>
wait_for() {
  local kind="$1" pattern="$2" timeout="$3" what="$4"
  local deadline=$(( SECONDS + timeout ))
  while (( SECONDS < deadline )); do
    if ros2 "$kind" list 2>/dev/null | grep -q -- "$pattern"; then
      echo "  ok: $what"
      return 0
    fi
    sleep 1
  done
  # Non-fatal by design: under `set -e` a non-zero return here would fire the EXIT trap
  # and tear down a robot that is actually running fine, just slower than expected.
  echo "  WARNING: timed out waiting for $what ($kind matching '$pattern')." >&2
  echo "           Continuing anyway - verify by hand before relying on it." >&2
  return 0
}

# Like start_stage, but the process is not monitored for liveness - see aux_pids.
start_aux_stage() {
  local name="$1"; shift
  echo "Starting $name..."
  setsid "$@" &
  aux_pids+=($!)
  aux_names+=("$name")
  sleep 2
  if ! kill -0 "${aux_pids[-1]}" 2>/dev/null; then
    echo "  WARNING: $name exited immediately; continuing without it." >&2
    return 0
  fi
}

start_stage() {
  local name="$1"; shift
  echo "Starting $name..."
  setsid "$@" &
  launch_pids+=($!)
  launch_names+=("$name")
  sleep 2
  if ! kill -0 "${launch_pids[-1]}" 2>/dev/null; then
    echo "$name exited immediately." >&2
    return 1
  fi
}

# ------------------------------------------------------- stages, as functions
# The base is started LAST when the arms are involved. Why: on 2026-08-23 the base came up
# healthy, ran fine for 14 s, and then died 2.4 s after the arm stacks activated their own
# two 1 kHz FCI loops:
#
#   t=875.9  swerve_drive_controller configured and activated
#   t=887.4  left+right FrankaHardwareInterface activate
#   t=890.1  BASE: libfranka Move command aborted by reflex
#            ["communication_constraints_violation"]
#
# ros2_control then demotes TmrHardware to `unconfigured`, its vx/vy/wz cartesian_velocity
# interfaces go [unavailable], and swerve_drive_controller never claims them - while still
# reporting `active`. The pedals publish correct cmd_vel and the base ignores every message
# with NOTHING logged on the laptop. Verify with ./base_health.sh: want [claimed].
#
# The reflex fires inside the arm bringup burst, not during steady-state teleop, so bring
# the base up after that burst has passed rather than making it survive it.
#
# Do NOT instead lower the base controller_manager update_rate (1000 Hz, in
# franka_ros2/franka_bringup/config/controllers.yaml): the base IS the 1 kHz consumer here,
# so slowing it makes the deadline misses worse, not better.
start_base() {
  start_stage "mobile base" \
    ros2 launch franka_bringup tmrv0_2.launch.py controller_name:=swerve_drive_controller
  wait_for topic '/swerve_drive_controller/odom' 30 "base odometry"

  # The base is useless if this one is not active - it owns the cartesian_velocity
  # command interfaces the pedal bridge ultimately drives.
  if ! ros2 control list_controllers 2>/dev/null | grep -q 'swerve_drive_controller.*active'; then
    echo "  WARNING: swerve_drive_controller is not active; the base will not move." >&2
  fi
  if ! ros2 control list_controllers 2>/dev/null | grep -q 'joint_state_broadcaster.*active'; then
    # Cosmetic only (it feeds /dynamic_joint_states and real wheel values), but it is the
    # controller that loses the race against a stale manager, so recover it in place.
    echo "  joint_state_broadcaster inactive; spawning it."
    ros2 run controller_manager spawner joint_state_broadcaster -c /controller_manager || true
  fi
}

# Drive both arms to the agreed teleop home pose (configs/teleop_home_pose.yaml on the
# laptop, copied to $home_file here). Skip with --no-home.
#
# THIS MOVES THE ARMS. The rest of this script deliberately moves nothing, so the countdown
# below is the chance to Ctrl+C if the workspace is not clear.
#
# Why it belongs here, BEFORE start_base: homing puts each arm's FCI into Move mode, and
# that traffic is exactly what has been tripping the base's own 1 kHz loop into a
# communication_constraints_violation. Homing while the base is still down costs nothing.
#
# joint_impedance_controller holds the command interfaces, so it must be inactive first; it
# spawns inactive, so this is normally a no-op and matters only on a re-run.
#
# One arm at a time, per the README: confirm the left/right mapping before two 7-DOF arms
# share a workspace.
home_arms() {
  # Delegates to ~/home_arms.py: ONE DDS participant for both arms.
  #
  # This used to shell out to `ros2 action send_goal` once per arm. Each call creates a new
  # participant, and that discovery burst repeatedly destroyed the other arm's goal
  # response ("Failed to send goal response ... client will not receive response") on
  # 2026-08-23. Same failure as two `ros2 control` calls killing the first arm; same fix.
  #
  # It also made Ctrl+C useless: a hung `timeout 120` call swallowed the interrupt and the
  # bash retry loop just moved to the next attempt. One python process is interruptible.
  local script="${TMR_HOME_SCRIPT:-$HOME/home_arms.py}"
  if [ ! -f "$script" ]; then
    echo "  WARNING: $script not found; skipping homing." >&2
    return 0
  fi
  if [ ! -f "$home_file" ]; then
    echo "  WARNING: home pose file not found ($home_file); skipping homing." >&2
    return 0
  fi

  echo
  echo "=============================================================="
  echo " ABOUT TO MOVE BOTH ARMS to the teleop home pose."
  echo " Clear the workspace; hands off the arms and the GELLOs."
  echo " Ctrl+C now to skip (--no-home disables this permanently)."
  echo "=============================================================="
  for i in 3 2 1; do printf "\r  starting in %ss... " "$i"; sleep 1; done
  echo

  # Bounded so a wedged action server can never freeze the bringup: worst case is roughly
  # discovery (15 s) + two arms * (accept 10 s + motion 30 s).
  timeout 150 python3 "$script" --file "$home_file" || {
    echo "  WARNING: homing did not complete cleanly; verify both arm poses by hand" >&2
    echo "           before activating impedance control." >&2
  }
  echo
}

# ZED head camera + both SICK nanoScan2 lidars.
#
# Deliberately FIRST, before any FCI loop is running. Bringing a driver up is a 15-25 s DDS
# discovery burst on this network, and such a burst aborts control loops that are already
# running - that is what has been tripping communication_constraints_violation all along.
# With the sensors up front, their discovery is finished before the arms or base exist.
#
# Bandwidth is fine despite the ZED being uncompressed: head_camera_zed_params.yaml sets
# pub_downscale_factor 2.0 and pub_frame_rate 15 with depth off, so it is ~640x360x15,
# roughly 10 MB/s - not the ~80 MB/s an untuned HD720@30 stream would be.
#
# start_cameras:=false because default_sensor_suite.yaml declares four D455s and only three
# are plugged in; the camera launch would fail on the missing one. The two WRIST cameras are
# not here at all - they run on the laptop, in a Docker container.
start_sensors() {
  if [ -x "$HOME/start_zed.bash" ]; then
    start_aux_stage "ZED head camera" "$HOME/start_zed.bash"
    wait_for topic '/head_camera/zed_node/rgb/image_rect_color' 40 "ZED rgb"
  else
    echo "  WARNING: ~/start_zed.bash not found; skipping the head camera." >&2
  fi

  start_aux_stage "base lidars" \
    ros2 launch franka_mobile_sensors franka_mobile_sensors.launch.py \
      start_cameras:=false start_lidars:=true start_rviz:=false
  # Topic names come from the namespaces in default_sensor_suite.yaml. wait_for is
  # non-fatal, so a wrong guess warns rather than tearing the robot down.
  wait_for topic '/lidar_front/scan' 40 "front lidar"
  wait_for topic '/lidar_rear/scan' 40 "rear lidar"
}

start_spine() {
  start_stage "spine" \
    ros2 launch franka_spine_server spine.launch.py spine_ip:="$spine_ip"
  wait_for service '/franka_spine_node/get_state' 30 "spine services"
}

if $skip_arms; then
  # Unchanged ordering, deliberately: with no arm stacks there is no bringup storm for the
  # base to survive, and base-then-spine is the path verified working on 2026-08-23.
  start_base
  start_spine
  echo
  echo "Base and spine are up (--skip-arms). Pedal teleoperation is ready."
  echo "Press Ctrl+C to stop everything."
  wait_any_launch
  exit $?
fi

# ---------------------------------------------------------------- 1. sensors
if $sensors; then
  start_sensors
fi

# ------------------------------------------------------------------ 2. spine
start_spine

# Controller state via the list_controllers SERVICE, not `ros2 control`. The CLI
# (ros2controlcli) is not guaranteed to be installed, and its output format varies
# between releases; the service is available wherever a controller_manager runs.
#
# A healthy controller_manager answers list_controllers in well under a second, so a
# short per-call timeout costs nothing and keeps a wedged manager from burning the whole
# budget one 20 s call at a time. Override with TMR_CM_CALL_TIMEOUT if a slow companion
# ever needs more.
cm_call_timeout="${TMR_CM_CALL_TIMEOUT:-3}"

controller_state() {
  local ns="$1" name="$2"
  timeout -k 2 "$cm_call_timeout" ros2 service call "/$ns/controller_manager/list_controllers" \
      controller_manager_msgs/srv/ListControllers 2>/dev/null \
    | grep -o "name='$name', state='[a-z]*'" | grep -o "state='[a-z]*'" | cut -d"'" -f2
}

# Waits until the controller is loaded AND settled (active/inactive) - not merely until
# the controller_manager's services exist, which happens almost immediately.
# NEVER fatal: a detection failure must not tear down a healthy robot via the EXIT trap.
wait_for_controller() {
  local ns="$1" name="$2" timeout="$3" state=""
  local deadline=$(( SECONDS + timeout ))
  while (( SECONDS < deadline )); do
    state="$(controller_state "$ns" "$name" || true)"
    case "$state" in
      active|inactive) echo "  ok: $ns/$name ($state)"; return 0 ;;
      unconfigured)
        # A spawner whose load_controller call timed out retries, hits "already loaded"
        # and dies FATAL, leaving the controller loaded but unconfigured. Configuring is
        # a state transition only: it claims no command interfaces and moves nothing.
        echo "  $ns/$name is unconfigured (spawner race); configuring it."
        timeout -k 2 "$cm_call_timeout" ros2 service call "/$ns/controller_manager/configure_controller" \
          controller_manager_msgs/srv/ConfigureController "{name: '$name'}" >/dev/null 2>&1 || true
        ;;
    esac
    sleep 3
  done
  echo "  WARNING: $ns/$name did not settle within ${timeout}s (last state: ${state:-unknown})." >&2
  echo "           Continuing anyway - check with: ros2 control list_controllers -c /$ns/controller_manager" >&2
  return 0
}

# ------------------------------------------------------------------- 3. arms
start_stage "both arms" \
  ros2 launch franka_fr3_arm_controllers franka_fr3_arm_controllers.launch.py \
    robot_config_file:=tmr_duo_config.yaml

# joint_impedance_controller is the one teleop activates. It spawns --inactive by
# design, so "inactive" here is success, not a failure.
#
# The budget only has to cover the spawners: on a healthy robot they load and configure
# within a few seconds, so 30 s is generous. It used to be 90 s per arm, which is 3
# minutes of dead wait whenever the manager is unresponsive - and the outcome in that
# case is a warning either way, so the extra minutes buy nothing. Raise with
# TMR_CM_SETTLE_TIMEOUT if a spawner ever legitimately needs longer.
cm_settle_timeout="${TMR_CM_SETTLE_TIMEOUT:-15}"
wait_for_controller left  joint_impedance_controller "$cm_settle_timeout"
wait_for_controller right joint_impedance_controller "$cm_settle_timeout"

# franka_robot_state_broadcaster asks for 'fr3/robot_state' while this robot exports
# 'left_fr3v2/robot_state' / 'right_fr3v2/robot_state' - its arm_id is not being set
# from tmr_duo_config.yaml's arm_id (fr3v2), and its param file is missing the /left
# and /right namespaces. It therefore stays inactive and its
# /<side>/franka_robot_state_broadcaster/* topics never publish. Teleoperation does not
# use them: joint_impedance_controller reads ros2_control state interfaces directly.
# Reported, not repaired - the fix belongs in the robot's launch config, not here.
for side in left right; do
  if [[ "$(controller_state "$side" franka_robot_state_broadcaster)" != "active" ]]; then
    echo "  note: $side/franka_robot_state_broadcaster not active (arm_id mismatch); harmless for teleop."
  fi
done

# Let the arm managers go quiet before the gripper launch adds two more
# controller_managers and six spawners.
sleep 3

# --------------------------------------------------------------- 4. grippers
start_stage "both grippers" \
  ros2 launch franka_gripper_manager robotiq_gripper_controller_client.launch.py \
    config_file:=tmr_duo_config_robotiq.yaml
wait_for topic '/left/gripper/gripper_client/target_gripper_width_percent' 30 "left gripper client"
wait_for topic '/right/gripper/gripper_client/target_gripper_width_percent' 30 "right gripper client"

# ------------------------------------------------------------- 5. home the arms
if $home_pose; then
  home_arms
fi

# --------------------------------------------------------------- 6. base (LAST)
# Deliberately after the arms and grippers; see start_base above.
start_base

# ----------------------------------------------------------- 7. activate the arms
# Runs LAST, after the base, because that is the order proven to work by hand. Activation
# puts each arm's FCI into Move mode, and doing it before the base is up has repeatedly
# aborted whichever loop was already running.
#
# ~/activate_arms.py uses ONE DDS participant for both arms and discovers both services
# before switching either. Two separate `ros2 control` calls kill the first arm - the
# second call's discovery burst aborts it about 3 s in. Never replace this with two calls.
#
# THIS MAKES THE ARMS LIVE: they begin following the GELLOs immediately.
if $activate; then
  activate_script="${TMR_ACTIVATE_SCRIPT:-$HOME/activate_arms.py}"
  if [ ! -f "$activate_script" ]; then
    echo "  WARNING: $activate_script not found; activate the arms by hand." >&2
  else
    echo
    echo "=============================================================="
    echo " ABOUT TO ACTIVATE BOTH ARMS - they will follow the GELLOs."
    echo " Hands OFF the GELLOs: the GELLO/arm pose delta is captured at"
    echo " this instant, and movement now becomes an approach target."
    echo " Ctrl+C to skip (--no-activate disables this permanently)."
    echo "=============================================================="
    for i in 3 2 1; do printf "\r  activating in %ss... " "$i"; sleep 1; done
    echo
    # Bounded: discovery 15 s + two switch calls at 10 s, plus slack.
    timeout 60 python3 "$activate_script" || {
      echo "  WARNING: activation did not complete; run it by hand:" >&2
      echo "           python3 $activate_script" >&2
    }
  fi
fi

cat <<'MSG'

Robot side is up: base, spine, arms (INACTIVE), grippers.

Nothing will move yet. On the laptop run ./start_teleop.bash, then, one arm at a time:

  1. Send the arm to the recorded home pose (configs/teleop_home_pose.yaml, RUNBOOK.md S5).
     The GELLO->arm map is a DELTA from the pose captured at activation, so activating
     away from home offsets the whole correspondence.
  2. Hands OFF the GELLOs, then:
       ros2 control set_controller_state joint_impedance_controller active -c /left/controller_manager
       ros2 control set_controller_state joint_impedance_controller active -c /right/controller_manager

Press Ctrl+C to stop every robot stack. Stop THIS first, then the laptop.
MSG

set +e
wait_any_launch
status=$?
set -e
(( status == 0 )) && status=1
echo "A robot launch process exited (status $status); stopping the rest." >&2
exit "$status"
