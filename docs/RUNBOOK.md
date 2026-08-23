# TMR teleoperation runbook

Day-to-day operating procedure for the Franka Mobile FR3 Duo (TMR) with GELLO leaders and
foot pedals. For first-time laptop setup see [`LABS_INTEGRATION.md`](LABS_INTEGRATION.md);
for per-subsystem reference see the [main README](../README.md).

Verified working end to end on **2026-08-23**: base driving from the pedals, both arms
following their GELLOs, both Robotiq grippers, arms auto-homed at bringup.

> ⚠️ **Known open issue: this is not yet reliable across restarts.** The same
> `communication_constraints_violation` reflex has recurred after a working session. The
> procedure below removes the *known* triggers (startup order, two-command arm activation,
> ad-hoc graph queries, orphan buildup), but the underlying cause — three 1 kHz FCI streams
> sharing one NIC with all DDS traffic — is **not fixed**. See [S9](#s9-robot-side-changes-2026-08-23).
> Expect to re-run the bringup, and check [S2](#s2-the-silent-failure-you-must-be-able-to-recognise)
> before concluding the robot is healthy.

---

## S1. The startup order is not optional

```bash
# 1. LAPTOP - GELLO leaders + foot pedals
cd ~/teleoperation && ./start_teleop.bash

# 2. ROBOT - spine, arms, grippers, home the arms, then the base LAST
~/start_robot.bash --restart

# 3. ROBOT - activate BOTH arms from ONE process
python3 ~/activate_arms.py
```

Each step is ordered for a reason, and every reason is the same one: **a new DDS
participant is a 15-25 s discovery burst on this network, and such a burst aborts any
Franka FCI control loop that is already running.**

| step | why here |
|---|---|
| laptop first | its 8 nodes finish joining the graph while the robot's FCI loops are still down |
| base last | the base survived every arm/gripper bringup only once it started after them |
| `activate_arms.py` | one participant for both arms; see [S3](#s3-never-activate-arms-with-two-separate-commands) |

### Shutdown: robot first, then laptop

```bash
# 1. ROBOT - Ctrl+C in the start_robot.bash terminal
#    Its EXIT trap stops every stage in order (base, grippers, arms, spine).

# 2. LAPTOP - Ctrl+C in the start_teleop.bash terminal
#    (or, if it was started with -d:)
./start_teleop.bash stop
```

Stop the **robot first**. The arms and base then stop being commanded while the laptop is
still publishing, which is harmless - GELLO and pedal messages simply go nowhere. Doing it
the other way leaves the robot's controllers running with no command stream.

Before shutting the robot down, it is worth deactivating the arms so they are not left in
Move mode:

```bash
python3 ~/activate_arms.py --deactivate
```

---

## S2. The silent failure you must be able to recognise

When an FCI loop is aborted, `ros2_control` demotes its hardware to `unconfigured` **but
the controller on top keeps reporting `active`**. Nothing is logged on the laptop. The
pedals publish correct `cmd_vel` and GELLO publishes correct joint states, and the robot
ignores all of it.

The only reliable tell is on the robot:

```bash
bash ~/base_health.sh     # want: vx/vy/wz/cartesian_velocity  [available] [claimed]
```

`[unavailable] [unclaimed]` means the base is dead no matter how healthy everything else
looks. For the arms, look for this in the robot's terminal:

```
libfranka: Move command aborted: motion aborted by reflex! ["communication_constraints_violation"]
```

Recovery is a bringup. A demoted hardware component does not come back on its own.

---

## S3. Never activate arms with two separate commands

```bash
python3 ~/activate_arms.py               # correct
python3 ~/activate_arms.py --deactivate  # both off, e.g. before re-homing
```

Do **not** do this:

```bash
# WRONG - the second command kills the first arm
ros2 control set_controller_state joint_impedance_controller active -c /left/controller_manager
ros2 control set_controller_state joint_impedance_controller active -c /right/controller_manager
```

Each CLI call creates a fresh DDS participant. Measured on 2026-08-23:

```
457671.90  LEFT impedance activated
457674.66  LEFT reflex, communication_constraints_violation   <- 2.8 s later
457675.19  RIGHT impedance activated
```

The left arm died from the discovery burst of the command activating the right one.
`activate_arms.py` waits for **both** services to be discovered before switching
**either**, so all discovery happens while both arms are still idle.

Keep hands **off** the GELLOs during activation: the controller captures the GELLO and arm
poses in the same instant, and movement in between becomes an approach target.

---

## S4. Do not query the ROS graph during teleop

`ros2 topic list`, `ros2 control list_controllers`, `ros2 node list` - especially with
`--no-daemon` - each create a participant, and therefore each is a discovery burst that can
abort a running FCI loop. Several aborts investigated on 2026-08-23 were caused by the
diagnostics themselves.

If you must inspect a live session, prefer things that put **nothing** on the ROS graph:

```bash
ssh companion 'tmux capture-pane -p -t 0 -S -200'   # the robot's own console
ssh companion 'ps -eo pid,ppid,etime,args | grep ros2'
./start_teleop.bash logs pedal                      # reads a log file in the container
```

---

## S5. Arm home pose

`start_robot.bash` homes both arms automatically, after the grippers and before the base.
It reads `~/teleop_home_pose.yaml` on the robot (copied from
[`configs/teleop_home_pose.yaml`](../configs/teleop_home_pose.yaml)), deactivates
`joint_impedance_controller` first because it holds the command interfaces, and sends
`PTPMotion` at 0.15 rad/s one arm at a time.

* It prints a warning and a **5 s countdown** before moving. Ctrl+C to skip.
* `--no-home` disables it permanently.
* Homing the arms matters because the GELLO to arm mapping is a **delta** captured at
  activation. Activate away from home and teleop still "works", it just does not match.

---

## S6. Laptop side: `start_teleop.bash`

```bash
./start_teleop.bash              # foreground, live logs, Ctrl+C stops both stacks
./start_teleop.bash -d           # detached, return to the prompt
./start_teleop.bash --pedal-fg   # real TTY for the pedal stack: 'm' and w/a/s/d/q/e work
./start_teleop.bash stop | status | logs [gello|pedal]
```

Plain `./start_teleop.bash` **is** the restart - it stops what is running first, escalating
to SIGKILL, because a leftover node keeps the GELLO serial ports and the evdev grabs and
makes the next start fail.

It runs the nodes in the **Humble container**, not on the host. This is required, not a
preference: the laptop runs ROS 2 **Kilted** and the robot runs **Humble**, and ROS 2 does
not support cross-distro communication. From the Kilted host, `ros2 node list` returns
empty and every `controller_manager` service call times out, while topics partially
discover - which looks like a network fault and is not one.

The script also handles, so you do not have to:

* starting the container (`--privileged`, host network, `/dev` and `/dev/serial/by-id`)
* running as `uid:20` (**dialout**) - a privileged container gets a *fresh* `/dev`, so the
  host ACL granting your user the GELLO serial ports does not carry over
* re-installing `evdev` / `dynamixel_sdk` / `requests`, which do not survive `docker rm`
* rebuilding for Humble **as your uid** - a root build through the bind mount leaves
  `build/ install/ log/` root-owned and breaks the next build
* a clock-skew check (skipped silently without passwordless ssh)

---

## S7. Clock skew

`SwerveDriveController` discards `cmd_vel` older than **0.5 s** and logs nothing, and
`JointImpedanceController` rejects stale GELLO samples. Symptom: everything looks healthy,
nothing moves.

```bash
./configs/sync_robot_clock.sh --check   # measure only
./configs/sync_robot_clock.sh           # measure and correct
```

Restart the teleop nodes afterwards - a step change in the clock upsets running ROS timers.
Set up `ssh-copy-id companion` once and `start_teleop.bash` will check this automatically.

---

## S8. Troubleshooting

| symptom | cause | fix |
|---|---|---|
| Pedals publish, base does not move, nothing logged | `TmrHardware` demoted to `unconfigured` | `base_health.sh`; re-run bringup |
| One arm stops following GELLO right after activation | the other arm's activation command | use `activate_arms.py` ([S3](#s3-never-activate-arms-with-two-separate-commands)) |
| Base dies ~2 s after the arms come up | base started before the arms | base must start **last**; `start_robot.bash` does this |
| Nothing moves, no errors anywhere | clock skew > 0.5 s | [S7](#s7-clock-skew) |
| `ros2 node list` empty, services time out, topics visible | Kilted host talking to a Humble robot | use the container ([S6](#s6-laptop-side-start_teleopbash)) |
| `Could not open port /dev/serial/by-id/...` | container user lacks **dialout** | `start_teleop.bash` uses `uid:20`; check the host ACL too |
| `no candidate device found` for the foot switches | udev rule missing | install `configs/99-pcsensor-footswitch.rules`; only `MODE="0666"` matters, the switches are autodetected by vendor:product |
| Load climbing across restarts, reflexes more frequent | orphaned nodes from earlier runs | `start_robot.bash --restart` now sweeps them ([S9](#s9-robot-side-changes-2026-08-23)) |
| `wait: <pid>: no such job` and everything shuts down | a launch stage died | fixed: it now names the stage |
| `franka_robot_state_broadcaster` fails to activate | known, wrong `arm_id`; teleop does not use it | ignore |

---

## S9. Robot-side changes (2026-08-23)

These run **on the robot**, and are now versioned in [`../robot/`](../robot/). Deploy and
compare them with `./robot/deploy.bash` (`--check` to diff, `--pull` to capture edits made
on the robot). Every in-place edit also left a timestamped backup on the robot
(`~/start_robot.bash.before-*`).

| file | change |
|---|---|
| `~/start_robot.bash` | base starts **last**; homes both arms; orphan sweep on `--restart`; `timeout -k` so `wait_for_controller` stops leaking `ros2 service call` pairs; `wait_any_launch` names a dead stage instead of failing with `no such job` |
| `~/activate_arms.py` | **new** - activates both arms from one DDS participant |
| `~/teleop_home_pose.yaml` | **new** - copy of `configs/teleop_home_pose.yaml` |
| `~/fastdds_wifi.xml` | **written but NOT in use** - see below |

The orphan leak is worth understanding: `start_robot.bash`'s `stale_pattern` matched only
launch processes and `ros2_control_node`, never the node executables those launches spawn
(`robotiq_gripper_client`, `spine_action_server`, `robot_state_publisher`). Each
`--restart` therefore leaked them to init. Found on 2026-08-23: six orphaned gripper
clients and a spine server, the oldest 8 h old, all still bound to the same topics and
serial ports.

### Not done: moving DDS onto WiFi

The underlying fragility is that three 1 kHz FCI streams share one USB-Ethernet NIC
(`enx*`, r8152, gigabit on USB 3) with all DDS traffic. Separating them - DDS on WiFi
(`192.168.50.x`, both machines are on "Instinct Office"), wired reserved for FCI - remains
the principled fix.

An attempt on 2026-08-23 **failed and was reverted**: adding `initialPeersList` broke the
robot's own local discovery, because unicast initial peers only probe a small range of
participant indices per address and the robot runs well over a dozen participants. The
profile files remain for a future attempt. Revert either side with `TMR_DDS_PROFILE=""` -
no file edits needed.

Do **not** "fix" this by lowering the base `controller_manager` `update_rate` (1000 Hz, in
`franka_ros2/franka_bringup/config/controllers.yaml`). The base **is** the 1 kHz consumer;
slowing it makes deadline misses worse.
