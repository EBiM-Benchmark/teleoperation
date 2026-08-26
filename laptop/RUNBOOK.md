# Manual runbook — GELLO + pedals driving the whole TMR

The short, no-script version. Every command here is also in `README.md`, which explains
*why*; this page is just the order to type them in.

Coverage: **pedals → base + spine**, **GELLO → both arms + both grippers**.

Something not moving? **§2** moves each actuator on its own, with no input device in
the loop — run it before debugging the teleop chain.

---

## 0. Order matters

Bring up **robot first, laptop second**. Shut down **laptop first, robot second** — the
base watchdog then halts the base itself instead of it holding a last command.

---

## 0.5 The fast path — two scripts, two terminals

This is the normal way in. The rest of this page is the manual expansion of these two
commands, for when one of them misbehaves.

**Terminal 1 — on the robot.** Leave it open; Ctrl+C here stops everything it started.

```bash
ssh tmr-user@companion
./start_robot.bash          # base -> spine -> arms (INACTIVE) -> grippers
```

Wait for `Robot side is up: base, spine, arms (INACTIVE), grippers.` Takes ~1.5 min. A
healthy run has **no WARNING lines**:

```
  ok: base odometry
  ok: spine services
  ok: left/joint_impedance_controller (inactive)
  ok: right/joint_impedance_controller (inactive)
  ok: left gripper client
  ok: right gripper client
```

`inactive` for `joint_impedance_controller` is **success** — it spawns `--inactive` by design
so activation stays an explicit gate. The two `note: ... franka_robot_state_broadcaster not
active` lines are expected and harmless for teleop (arm_id mismatch, see README).

If instead you get `WARNING: left/joint_impedance_controller did not settle`, the arm
controller_manager is not answering — see §9.

**Terminal 2 — on the laptop.** Run it in a *real* terminal: without a TTY the
`keyboard_state_publisher` dies and the `m` mode toggle is unavailable.

```bash
cd /home/1sliu/teleoperation
./start_teleop.bash         # pedals -> GELLO
```

Add `--task-id <UUID>` (or set `TMR_LABS_TASK_ID`) if you want LABS recording; without it the
recording pedal refuses to start. `--restart` stops a previous session instead of refusing.

**Pedals, spine and grippers are live at this point. The arms are not.** Continue to §5 to
home the arms and activate them.

### Flags worth knowing

| Flag | Effect |
|---|---|
| `./start_robot.bash --restart` | Stop a running/orphaned stack first. Needed after a `Ctrl+\` or any hard kill, which orphans `ros2_control_node`s that still hold the FCI. |
| `./start_robot.bash --skip-arms` | Base + spine only, ~20 s. Pedal driving with no arms *and no grippers*. |
| `./start_teleop.bash --skip-device-checks` | Skips **all** preflight (devices, robot reachability, clock). Diagnostics only. |

### Domain

Both scripts default to **`ROS_DOMAIN_ID=0`**, matching LABS, the Jetson and the Olive
sensors. Nothing in the robot's rc files sets a domain, so a plain SSH shell is already on 0 —
**do not export one**. An old terminal still exporting 100 will see none of the stack and
every command will fail with no useful error. Override both sides together with
`TMR_ROS_DOMAIN_ID` if you must.

### Give Ctrl+C time

`start_robot.bash`'s trap walks each stage down in order and takes 10–15 s. Reaching for
`Ctrl+\` kills the launch leader before it can reap its children, leaving orphaned
`ros2_control_node`s holding the FCI — after which the next start needs `--restart`.

---

## 1. Robot side — four launches, four terminals

A new terminal on the robot sources `~/tams_ws` automatically. Over SSH the login shell is
**zsh**, where sourcing a ROS setup silently fails — run remote commands as `bash -ic "..."`.

```bash
# 1. Mobile base
ros2 launch franka_bringup tmrv0_2.launch.py controller_name:=swerve_drive_controller

# 2. Spine (torso)
ros2 launch franka_spine_server spine.launch.py spine_ip:=172.16.16.10

# 3. Both arms
ros2 launch franka_fr3_arm_controllers franka_fr3_arm_controllers.launch.py \
  robot_config_file:=tmr_duo_config.yaml

# 4. Both Robotiq grippers
ros2 launch franka_gripper_manager robotiq_gripper_controller_client.launch.py \
  config_file:=tmr_duo_config_robotiq.yaml
```

The arm controllers spawn **inactive** on purpose. Nothing moves yet.

---

## 2. Move each thing a little

Before wiring up any input device, prove each actuator moves on its own. Four small
motions, one per subsystem — arm, gripper, spine, base. Each one isolates a single link of
the chain, so when teleop later does nothing you know whether to suspect the pedal, the
bridge, the network, or the hardware.

Run these **on the robot**, one SSH terminal each:

```bash
ssh tmr-user@companion
bash -i                      # the SSH login shell is zsh, where sourcing ROS silently fails
```

That `bash -i` matters: without it every package looks missing. The domain needs no export —
`.bashrc` sets none, so the shell lands on 0, which is what `start_robot.bash` launches on and
what LABS, the Jetson and the Olive sensors all use. If the stack *was* started with a
`TMR_ROS_DOMAIN_ID` override, match it here or the shell sees **none** of the running stack:
`ros2 topic list` comes back nearly empty and every command below fails with no useful error.
Scripting it from your own machine instead: `ssh tmr-user@companion 'bash -ic "..."'`.

Running the base test *on the robot* is not incidental — the base ages every command against
its own clock, so a local publish takes clock skew out of the picture entirely. **Base moves
from the robot but not from the laptop ⇒ the fault is skew or DDS, not the base.**

Before starting:

* `./start_robot.bash` has finished, and `ros2 control list_controllers` shows
  `swerve_drive_controller` **active**.
* **No laptop teleop running.** `mobile_teleop.launch.py` streams a zero `TwistStamped` at
  20 Hz and will fight a manual base publish; `franka_gello_state_publisher` streams arm
  *and* gripper commands at 30 Hz and will override manual goals.
* Space is clear: the base test drives ~8 cm, the spine test lifts 2 cm.

### 2.1 Arm — nudge one joint, one arm at a time

`joint_impedance_controller` holds the effort command interfaces, so stand it down first:

```bash
ros2 control set_controller_state joint_impedance_controller inactive -c /left/controller_manager
```

`/left/franka/joint_states` is **not** ordered joint1..7, so build the goal by name. This
snippet only *prints* the command — the motion stays a deliberate paste:

```bash
python3 - <<'PY'
import rclpy
from sensor_msgs.msg import JointState
NS, JOINT, DELTA = "left", 7, 0.15          # joint7 = wrist roll; 0.15 rad ~= 8.6 deg
rclpy.init(); n = rclpy.create_node("ptp_nudge"); box = []
n.create_subscription(JointState, f"/{NS}/franka/joint_states", lambda m: box.append(m), 10)
while rclpy.ok() and not box:
    rclpy.spin_once(n, timeout_sec=1.0)
q = dict(zip(box[-1].name, box[-1].position))
g = [q[f"{NS}_fr3v2_joint{i}"] for i in range(1, 8)]
g[JOINT - 1] += DELTA
print(f"ros2 action send_goal /{NS}/action_server/ptp_motion franka_msgs/action/PTPMotion \\")
print('  "{goal_joint_configuration: [' + ", ".join(f"{v:.6f}" for v in g) + '],')
print('    maximum_joint_velocities: [0.15,0.15,0.15,0.15,0.15,0.15,0.15], goal_tolerance: 0.01}"')
PY
```

Paste what it prints. `status: 2` is TARGET_REACHED. Re-run with `DELTA = -0.15` to put the
joint back, and `NS = "right"` for the other arm — **one arm at a time**.

Same action server and same 0.15 rad/s cap as the home-pose step in §5, just a small
relative target instead of an absolute pose. Leave the controller `inactive` afterwards;
that is its correct resting state.

If the arm aborts with `communication_constraints_violation`, that is a robot-side reflex,
not a controller fault:

```bash
ros2 action send_goal /left/action_server/error_recovery franka_msgs/action/ErrorRecovery {}
```

### 2.2 Gripper — quarter close, then open

```bash
ros2 action send_goal /left/gripper/robotiq_gripper_controller/gripper_cmd \
  control_msgs/action/GripperCommand "{command: {position: 0.20, max_effort: 1.0}}"

ros2 action send_goal /left/gripper/robotiq_gripper_controller/gripper_cmd \
  control_msgs/action/GripperCommand "{command: {position: 0.0, max_effort: 1.0}}"
```

`reached_goal: true` in the result is the pass condition. `right/gripper` for the other one.

`position` is radians of `robotiq_85_left_knuckle_joint`: **0.0 = open, 0.8 = closed**.
Note that is the *inverse* of the GELLO percent topic, where 1.0 = open —
`robotiq_gripper_client` sends `1 - percent`. Reading the two conventions as the same is an
easy way to convince yourself a working gripper is backwards.

### 2.3 Spine — up 2 cm

```bash
ros2 service call /franka_spine_node/get_state    franka_spine_msgs/srv/GetSpineState
ros2 service call /franka_spine_node/get_position franka_spine_msgs/srv/GetPosition
ros2 service call /franka_spine_node/switch_on    franka_spine_msgs/srv/SwitchOn
```

Want `SwitchedOn`. **`get_position` returns millimetres**, despite the interface documenting
metres — a known `franka_spine_server` bug. `move_absolute.position` really is metres. So a
reading of `344.0` means the spine is at 0.344 m, and 2 cm up is `0.364`:

```bash
ros2 action send_goal /franka_spine_node/move_absolute franka_spine_msgs/action/MoveAbsolute \
  "{position: 0.364, velocity: 0.05, acceleration: 0.05, deceleration: 0.05}"
```

> ⚠ **The spine cannot be aborted.** `Halt` is a 404 on the device, and cancelling the goal
> abandons it without stopping the hardware. Whatever target you send *will* be travelled
> to, at 0.05 m/s. This is why the step here is 2 cm and not "somewhere near the top".

`velocity` must be a multiple of 0.001 — the server converts to an integer mm/s. Soft limits
are read from the device (≈ 0–0.77 m, genuine metres here):

```bash
ros2 service call /franka_spine_node/get_parameters_spine franka_spine_msgs/srv/GetParameters
```

The `_spine` suffix is not a typo; the plain name is taken by the standard ROS 2 parameter
service. A refused move returns `success: false` rather than raising — on HTTP 424 "invalid
state or busy", cycle `switch_off` then `switch_on` (README, *Spine refuses every move*).

### 2.4 Base — forward ~8 cm

```bash
ros2 topic pub -r 20 -t 40 /swerve_drive_controller/cmd_vel geometry_msgs/msg/TwistStamped \
  "{header: auto, twist: {linear: {x: 0.05}}}"
```

`header: auto` is **mandatory** — `ros2 topic pub` re-stamps on every publication, and an
unset stamp is maximally stale, which the controller discards in silence. `-r 20 -t 40` is
2 s of commands, comfortably inside the 0.5 s `cmd_vel_timeout`; when they stop the watchdog
halts the base itself. Ctrl+C does the same.

`--times` also makes `pub` wait for a subscriber before it starts. That is a free check: a
repeating `Waiting for at least 1 matching subscription(s)...` means nothing is listening on
that topic — the base stack is down, on another domain, or not reaching you — and no command
was ever sent.

Limits are 0.1 m/s on x and y, 0.1 rad/s on yaw, with 0.1 acceleration on all three —
anything larger is silently clamped. That ramp means 0.05 m/s is only reached after 0.5 s,
so the 2 s burst covers roughly 8 cm. Measured on 2026-08-22: `0.0862 m`, with both drive
joints turning at ~1.1 rad/s.

> **This is why "the pedals do not move the base" is usually wrong.** At these speeds a
> half-second tap is ~2 cm. Before suspecting the software, hold a pedal for a full five
> seconds, or run the burst above and read `odom` rather than trusting your eyes.

Strafe and rotate, same shape: `twist: {linear: {y: 0.05}}` and `twist: {angular: {z: 0.05}}`
(positive z is CCW).

In a second terminal, watch what the controller actually accepted:

```bash
ros2 topic echo /swerve_drive_controller/cmd_vel_out
```

Non-zero `cmd_vel` with all-zero `cmd_vel_out` is the stale-command signature — go to the
clock check in §3.

---

## 3. Laptop side — environment

On the **TAMS laptop** ROS 2 lives in a container, so enter it first:

```bash
docker exec -it teleop_inputs bash
source /opt/ros/humble/setup.bash
cd /workspace
```

On a **Pixi laptop** instead:

```bash
source <(cd ~/my_ros_ws && pixi shell-hook -s bash)
cd ~/Documents/code/teleoperation
```

Then, on either:

```bash
source install/setup.bash
source configs/tmr_laptop_env.sh     # domain 0 + DDS pinned to the Ethernet NIC
```

All three lines are required: ROS gives you `ros2`, `install/setup.bash` gives the packages,
the env script gives the DDS config.

### Check the clock before anything else

The base ages every command against its **own** clock and silently discards anything older
than 0.5 s — no error is logged anywhere. Verify by reading a robot-published topic:

```bash
python3 - <<'PY'
import time, rclpy
from nav_msgs.msg import Odometry
rclpy.init(); n = rclpy.create_node("skew"); s = []
n.create_subscription(Odometry, "/swerve_drive_controller/odom",
    lambda m: s.append(m.header.stamp.sec + m.header.stamp.nanosec*1e-9 - time.time()), 10)
t = time.time()
while time.time() - t < 6 and len(s) < 5:
    rclpy.spin_once(n, timeout_sec=0.5)
print(f"skew: {sum(s)/len(s):+.3f} s" if s else "NO ODOM — robot stack down?")
PY
```

Want well under 0.5 s. If it is not: `sudo systemctl restart systemd-timesyncd`, then
restart the teleop nodes (a step change in the clock upsets running ROS timers).

---

## 4. Laptop side — pedals, then GELLO

Two more terminals, each with the section-3 environment sourced.

```bash
# Terminal A — pedals (base + spine)
ros2 launch tmr_pedal_teleop mobile_teleop.launch.py

# Terminal B — GELLO (arms + grippers)
ros2 launch franka_gello_state_publisher main.launch.py config_file:=franka_gello_duo.yaml
```

Pedals first: if the pedal publisher cannot open both switches the whole launch shuts down,
and you want to know that before the GELLOs are live.

Expected:

```
Foot switch 1: /dev/input/event5 ... [autodetected]
Foot switch 2: /dev/input/event9 ... [autodetected]
Pedal publisher started. Switch1=True Switch2=True.
[left.gello_publisher]:  Publishing GELLO joint states.
[right.gello_publisher]: Publishing GELLO joint states.
```

**Only one pedal publisher can run at a time** — it grabs both switches exclusively. A stray
second instance silently sees no pedals.

The base and spine are live now. The arms are not.

---

## 5. Arms — home pose, then activate

**Both arms must be at the home pose before you activate teleop.** The controller maps
GELLO→arm as a *delta* from the poses captured at activation, so activating away from home
leaves the whole correspondence offset — it still "works", it just does not match.

One arm at a time. Deactivate the impedance controller first; it holds the command interfaces.

```bash
ros2 control set_controller_state joint_impedance_controller inactive -c /left/controller_manager

ros2 action send_goal /left/action_server/ptp_motion franka_msgs/action/PTPMotion \
  "{goal_joint_configuration: [-1.072185, -0.082723, 1.024787, -2.746873, 1.176839, 1.976895, 0.168207],
    maximum_joint_velocities: [0.15,0.15,0.15,0.15,0.15,0.15,0.15], goal_tolerance: 0.01}"

ros2 action send_goal /right/action_server/ptp_motion franka_msgs/action/PTPMotion \
  "{goal_joint_configuration: [0.809728, -0.335639, -0.800341, -2.792884, -1.182067, 1.760234, -0.095578],
    maximum_joint_velocities: [0.15,0.15,0.15,0.15,0.15,0.15,0.15], goal_tolerance: 0.01}"
```

`status: 2` means TARGET_REACHED. (Values are `configs/teleop_home_pose.yaml`.)

Then **hands off the GELLOs** and activate. Any movement between activation and the first
update becomes an approach target, traversed slowly but traversed:

```bash
ros2 control set_controller_state joint_impedance_controller active -c /left/controller_manager
ros2 control set_controller_state joint_impedance_controller active -c /right/controller_manager
```

The arms are live. The grippers follow the GELLO triggers with no extra step.

---

## 6. Controls

**Pedals** — FS1 is one switch, FS2 the other:

| Pedal | Motion | | Pedal | Motion |
|---|---|---|---|---|
| FS1.a | forward (x+) | | FS2.a | backward (x−) |
| FS1.b | strafe left (y+) | | FS2.b | strafe right (y−) |
| FS1.c | rotate CW | | FS2.c | rotate CCW |

Spine, hold to jog: **FS1.a + FS2.c = up**, **FS1.c + FS2.a = down**. While a spine combo is
held the base stays still. Release stops the spine within one 0.02 m step.

**GELLO** — move it, the arm follows; squeeze the trigger, the gripper closes.

> The operator's hands are **crossed** with respect to the namespaces, and that is intended:
> the GELLO in your **right** hand drives the `left` arm and the `left` gripper. Do not
> "fix" it. What must hold is that a GELLO's trigger closes the gripper bolted to the arm
> that same GELLO moves.

---

## 7. Is it working?

```bash
ros2 topic info /pedal/state                 # want 1 publisher, 3 subscribers
                                             #   base_bridge, spine_bridge, labs_pedal_bridge
ros2 topic info /left/gello/joint_states     # want 1 publisher
```

Hold FS1.a and watch both of these — `cmd_vel` non-zero but `cmd_vel_out` all zero means
the controller is discarding commands as stale, i.e. go back to the clock check:

```bash
ros2 topic echo /swerve_drive_controller/cmd_vel
ros2 topic echo /swerve_drive_controller/cmd_vel_out
```

> Do **not** judge the laptop-local nodes with `ros2 node list` or `ros2 topic hz` — the DDS
> whitelist makes a perfectly healthy stack look dead. Use `ros2 topic info` and `pgrep`.

---

## 8. Shutdown

Ctrl+C the **laptop** terminals first (GELLO, then pedals), then the robot launches.

Give each one 10–15 s. `start_robot.bash`'s trap walks the stages down in order; killing it
harder than SIGINT orphans `ros2_control_node`s that keep holding the FCI, and the next start
then needs `--restart`.

---

## 9. `ros2 control` hangs on one arm but `ros2 param` answers

Signature — the asymmetry is the whole diagnosis:

```bash
ros2 control list_controllers -c /left/controller_manager
#   Failed getting a result from calling .../list_controllers in 10.0. (Attempt 1 of 3.)
ros2 param get /left/joint_impedance_controller k_alpha
#   answers instantly
```

Every arm controller spawner dies, `joint_impedance_controller` never reaches `inactive`, and
`start_robot.bash` prints `WARNING: left/joint_impedance_controller did not settle`. The base
`/controller_manager` is unaffected.

**This is a blocked lifecycle callback, not DDS and not load.** `controller_manager` holds one
`services_lock_` across every one of its service callbacks and calls `on_configure`
synchronously while holding it. Anything that blocks inside a controller's `on_configure`
therefore wedges `list_controllers`, `configure_controller`, `switch_controller` and
`load_controller` for the life of the process. Parameter services sit on the node's default
callback group and never take that lock — hence the split.

Diagnose with `ros2 node list`, not with DDS settings. Do not chase clock skew, domains,
shared memory or CPU headroom: a wedged manager reproduces identically on an isolated domain,
with SHM disabled, and with the arms launched alone on an idle machine.

Historical instance (fixed 2026-08-22): `JointImpedanceController::on_configure` built an
`AsyncParametersClient` for `"robot_state_publisher"` — a *relative* name resolving to
`<ns>/robot_state_publisher` — and called `wait_for_service()` with no timeout. Setting
`use_visualization: "false"` in `tmr_duo_config.yaml` stopped that node from launching, so the
wait never returned. The fetched value was never read. Fix was to delete the block.

If a controller must read something from another node, take it as a parameter. Never block on
another node from inside a lifecycle callback.
