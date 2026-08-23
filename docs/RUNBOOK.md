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
> sharing one NIC with all DDS traffic — is **not fixed**. See [S10](#s10-robot-side-changes-2026-08-23).
> Expect to re-run the bringup, and check [S2](#s2-the-silent-failure-you-must-be-able-to-recognise)
> before concluding the robot is healthy.

---

## S1. The startup order is not optional

```bash
# 1. LAPTOP - GELLO leaders + foot pedals. Also checks and CORRECTS clock skew.
cd ~/teleoperation && ./start_teleop.bash

# 2. ROBOT - spine, arms, grippers, home the arms, base LAST, then activate both arms
~/start_robot.bash --restart
```

That is the whole procedure. Clock correction, arm homing and arm activation used to be
separate steps people forgot; they now run inside those two commands. Opt out per stage with
`--no-home`, `--no-activate`, or `--skip-arms` (base + spine only).

Each step is ordered for a reason, and every reason is the same one: **a new DDS
participant is a 15-25 s discovery burst on this network, and such a burst aborts any
Franka FCI control loop that is already running.**

| step | why here |
|---|---|
| laptop first | its 8 nodes finish joining the graph while the robot's FCI loops are still down |
| base last | the base survived every arm/gripper bringup only once it started after them |
| `activate_arms.py` | one participant for both arms, run automatically at the end of step 2; see [S3](#s3-never-activate-arms-with-two-separate-commands) |

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

A demoted hardware component does not come back on its own — but it does **not** require a
full bringup either. See [S11](#s11-fast-recovery-after-a-reflex).

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

`start_robot.bash` homes both arms automatically, after the grippers and before the base,
by calling [`robot/home_arms.py`](../robot/home_arms.py). It reads
`~/teleop_home_pose.yaml` (deployed from
[`configs/teleop_home_pose.yaml`](../configs/teleop_home_pose.yaml)), deactivates
`joint_impedance_controller` first because it holds the command interfaces, and sends
`PTPMotion` at 0.15 rad/s.

* **One DDS participant for both arms**, same as `activate_arms.py`. Shelling out to
  `ros2 action send_goal` twice made the second call's discovery burst destroy the first
  arm's goal response (`Failed to send goal response ... client will not receive response`).
* **Skips an arm already at home** (every joint within 0.05 rad). Not just faster - each
  PTP goal is another FCI Move cycle, and those are what trip the reflex. `--force`
  overrides.
* Run it by hand any time: `python3 ~/home_arms.py [--side left] [--force]`

* It prints a warning and a **3 s countdown** before moving. Ctrl+C to skip - and it is
  now genuinely interruptible, where the old bash retry loop swallowed the interrupt
  and looked frozen.
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
| An arm stops following its GELLO after hitting a limit | reflex; hardware demoted to `unconfigured` | `python3 ~/recover_arms.py` ([S11](#s11-fast-recovery-after-a-reflex)) — no restart needed |
| One arm stops following GELLO right after activation | the other arm's activation command | use `activate_arms.py` ([S3](#s3-never-activate-arms-with-two-separate-commands)) |
| Spine: `424 Client Error: Failed Dependency` on `motion-mm:start` | robot not powered on from Desk after a reboot | open `https://172.16.16.10/` and press **power on**; see [S9](#s9-known-bugs-and-gotchas) |
| A spawner cannot reach its **own** local `controller_manager` | a Fast DDS profile with `initialPeersList` is loaded | `TMR_DDS_PROFILE` must be empty; see [S9](#s9-known-bugs-and-gotchas) |
| `Unverified HTTPS request ... 172.16.16.10` | Desk's self-signed cert, from the spine server | ignore - it is a warning, not an error |
| `https://172.16.16.10/` will not open | control units powered off; ARP fails | check robot power, E-stop, and the switch link lights |
| Base dies ~2 s after the arms come up | base started before the arms | base must start **last**; `start_robot.bash` does this |
| Nothing moves, no errors anywhere | clock skew > 0.5 s | [S7](#s7-clock-skew) |
| `ros2 node list` empty, services time out, topics visible | Kilted host talking to a Humble robot | use the container ([S6](#s6-laptop-side-start_teleopbash)) |
| `Could not open port /dev/serial/by-id/...` | container user lacks **dialout** | `start_teleop.bash` uses `uid:20`; check the host ACL too |
| `no candidate device found` for the foot switches | udev rule missing | install `configs/99-pcsensor-footswitch.rules`; only `MODE="0666"` matters, the switches are autodetected by vendor:product |
| Load climbing across restarts, reflexes more frequent | orphaned nodes from earlier runs | `start_robot.bash --restart` now sweeps them ([S10](#s10-robot-side-changes-2026-08-23)) |
| `wait: <pid>: no such job` and everything shuts down | a launch stage died | fixed: it now names the stage |
| `franka_robot_state_broadcaster` fails to activate | known, wrong `arm_id`; teleop does not use it | ignore |

---

## S9. Known bugs and gotchas

### The WiFi DDS profile breaks local discovery — keep it OFF

`~/fastdds_wifi.xml` is **opt-in and currently broken.** Its `initialPeersList` destroys the
robot's own **local** node discovery: unicast initial peers probe only a small range of
participant indices per address, and this robot runs well over a dozen participants. The
symptom is a spawner failing to reach a controller_manager on the *same machine*:

```
[spawner-3] Could not contact service /left/gripper/controller_manager/list_controllers
```

This bit twice. `start_robot.bash` shipped briefly with that profile as its **default**, so
every bringup reintroduced the fault unless someone remembered to pass `TMR_DDS_PROFILE=""`
by hand — and the docs meanwhile described the profile as "not in use", which was wrong.

Fixed in `5a78c47`: the default is now empty (plain all-interface discovery, which works).
Confirm at bringup — the script prints one of these near the top:

```
DDS: default discovery (all interfaces).                      <- correct
DDS: /home/tmr-user/fastdds_wifi.xml (WiFi only; ...)         <- WRONG, expect breakage
```

Do not enable it until the profile is fixed **and tested on throwaway nodes** rather than on
the live bringup path.

### After a robot reboot

The DDS-profile bug above shows up on the **first bringup after a reboot**, which makes it
easy to blame the reboot. It is not the reboot — the profile was loaded on every bringup.

What *does* survive a reboot, verified 2026-08-23 (18 min uptime):

| | |
|---|---|
| `/usr/local/sbin/tmr-set-clock` + `/etc/sudoers.d/tmr-clock` | survive |
| laptop→robot ssh key auth | survives |
| clock accuracy | **survives** — measured 0.011 s straight after reboot |

The clock surviving is not luck: `tmr-set-clock` runs `hwclock -w`, so a correction is
written to the hardware clock and restored at boot. Without that the companion would come up
skewed every time, since it has no reachable NTP source.

Still worth running `./configs/sync_robot_clock.sh --check` after a reboot — it is instant
and passwordless now.

**What does NOT survive: the robot's power-on state.** After a reboot or power cycle you
must press **power on** in TMR Desk (`https://172.16.16.10/`, self-signed cert, accept the
browser warning) before anything will move. Skip it and the spine fails with:

```
[franka_spine_node]: Failed to start motion: 424 Client Error: Failed Dependency
                     for url: https://172.16.16.10/spine/api/motion-mm:start
```

Pressing power on recovers the spine automatically — no restart of the ROS stacks needed.

> ⚠ **Do not diagnose this from `/spine/api/state`.** It reports `"SwitchedOn"` **both**
> when motion works and when it fails with 424, so it looks healthy either way. The limits
> endpoint is equally unhelpful — the target is well inside `0–770 mm`. The `424` itself is
> the signal, and Desk is the fix. Verified 2026-08-23: state `"SwitchedOn"`, position
> `353 mm`, identical before and after the power-on that fixed it.

### Editing the script does not fix a running stack

`FASTRTPS_DEFAULT_PROFILES_FILE` is read into each node's environment at launch. Changing
the script changes nothing for processes already running — a restart is required.

### Ctrl+\ is not a better Ctrl+C

`Ctrl+\` (SIGQUIT) produces `Quit (core dumped)` on the `wait` and skips the orderly
shutdown. Use `Ctrl+C`, which lets the EXIT trap stop the stages in order.

---

## S10. Robot-side changes (2026-08-23)

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

An attempt on 2026-08-23 **failed**: adding `initialPeersList` broke the robot's own local
discovery, because unicast initial peers only probe a small range of participant indices per
address and the robot runs well over a dozen participants. The symptom is a spawner unable
to reach its **own** controller_manager:

```
[spawner-3] Could not contact service /left/gripper/controller_manager/list_controllers
```

`start_robot.bash` shipped briefly with that profile as its **default**, which reintroduced
the fault on every bringup unless `TMR_DDS_PROFILE=""` was passed by hand. The default is
now empty - plain all-interface discovery, which works. The profile is opt-in and should
stay off until it is fixed and tested on throwaway nodes:

```bash
TMR_DDS_PROFILE=$HOME/fastdds_wifi.xml ~/start_robot.bash --restart   # do not, yet
```

Do **not** "fix" this by lowering the base `controller_manager` `update_rate` (1000 Hz, in
`franka_ros2/franka_bringup/config/controllers.yaml`). The base **is** the 1 kHz consumer;
slowing it makes deadline misses worse.

---

## S11. Fast recovery after a reflex

When an arm hits a speed or torque limit, or its FCI loop misses deadlines, libfranka aborts
the motion and `ros2_control` demotes `<side>_FrankaHardwareInterface` to `unconfigured`
while `joint_impedance_controller` still reports `active`. The arm stops following its GELLO.

Restarting `start_robot.bash` fixes it and costs minutes. This is the same repair in seconds:

```bash
python3 ~/recover_arms.py                 # both arms
python3 ~/recover_arms.py --side left     # just the one that died
python3 ~/recover_arms.py --no-activate   # recover, but leave the controller inactive
```

Three steps per arm, all through **one** DDS participant:

| | |
|---|---|
| 1 | `/<side>/action_server/error_recovery` → libfranka `automaticErrorRecovery()`, clears the reflex |
| 2 | `set_hardware_component_state` → `<side>_FrankaHardwareInterface` back to `active` (via `inactive` if a direct jump is refused) |
| 3 | `switch_controller` → `joint_impedance_controller` active again |

It reports each hardware state as it goes, and skips an arm that is already `active`.

### When this is not enough

`automaticErrorRecovery()` cannot clear an error that requires **manual intervention** — a
joint limit violation locks the brakes. If Desk shows the joints locked:

1. unlock the joints in TMR Desk (`https://172.16.16.10/`)
2. then run `recover_arms.py`

That ordering is the shortcut worth knowing: **unlocking in Desk does not by itself require
restarting the bash script.** Only if `recover_arms.py` still reports the hardware as
something other than `active` do you need a full bringup.

### Why it is one script and not three commands

Running the three steps as separate `ros2` invocations would create three DDS participants,
and each participant's 15–25 s discovery burst can abort whichever arm is still running —
turning a one-arm problem into a two-arm one. Same reason
[`activate_arms.py`](#s3-never-activate-arms-with-two-separate-commands) exists.
