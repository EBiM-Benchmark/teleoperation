# Introduction  
## About
This repository implements the teleoperation framework for the [**Franka Mobile FR3 Duo**](https://franka.de/mobile-fr3-duo) used in the [**EBiM competition**](https://ebim-benchmark.github.io/competition.html).
For **manipulation tasks**, it provides the configuration and implementation for teleoperation using the [**Franka GELLO Duo**](https://franka.de/de/gello).
For **mobile tasks**, it supports teleoperation using either a **keyboard** or a **USB foot pedal**.
Unless there are specific application requirements, **keyboard-based teleoperation is recommended** for mobile navigation, as it provides a more standardized and reliable control interface.

Episode recording is provided by [**LABS**](https://github.com/frankarobotics/labs), driven
from the same foot pedals — press **`m`** to switch them between moving the robot and
controlling data collection.

> 🚀 **Setting up a new laptop? Start with
> [`docs/LABS_INTEGRATION.md`](docs/LABS_INTEGRATION.md).** It covers the whole path:
> devices and udev rules, clock sync, Fast DDS networking, installing LABS, the ZED on the
> Jetson, bring-up order, and a first-run checklist. The sections below are the per-subsystem
> reference it links into.

> 📼 **Recording datasets? See [`docs/DATA_COLLECTION.md`](docs/DATA_COLLECTION.md)** for the
> record-and-verify loop and how to play a bag back in rviz2 / rqt_image_view / rqt_plot.

> ▶️ **Running it day to day? Use [`docs/RUNBOOK.md`](docs/RUNBOOK.md).** The startup order
> is load-bearing — laptop first, then the robot, then `activate_arms.py` — and getting it
> wrong produces a *silent* failure where every controller reports `active` while the robot
> ignores every command. The runbook explains the order, how to recognise that failure, and
> what not to do while teleop is running.

## Package Overview
* **franka_gello_state_publisher** - Reads the states of the Franka GELLO devices and publishes the joint states of the left arm, right arm, and grippers.
* **franka_gello_state_subscriber** - Subscribes to the published GELLO states for testing or for subsequent integration with robot control.
* **keyboard_state_publisher** - Reads keyboard inputs (`W`, `A`, `S`, `D`, `Q`, `E`) and publishes the corresponding keyboard states.
* **keyboard_state_subscriber** - Subscribes to the keyboard state topic and prints the corresponding key events
* **pedal_state_publisher** - Reads the **two** PCsensor foot switches via `evdev` and publishes the set of pressed pedals (`1A`..`2C`) on `/pedal/state`.
* **pedal_state_subscriber** - Debug helper that prints pedal actions (superseded by `tmr_pedal_teleop`).
* **tmr_pedal_teleop** - Bridges `/pedal/state` to the TMR mobile base (`swerve_drive_controller/cmd_vel`, `TwistStamped`) and the Franka spine (`franka_spine_msgs/action/MoveAbsolute`, hold-to-jog). Also owns the DRIVE/RECORD pedal mode (`mode_manager`), drives LABS data collection (`labs_pedal_bridge`) and publishes base/spine state for recording (`mobile_base_state_bridge`, `spine_state_publisher`).
* **franka_spine_msgs** - Vendored spine action/service definitions (copied from the robot's `tams_ws`) so the spine bridge can build on the teleop host.
* **franka_gripper_manager** - Controls the grippers; use `robotiq_gripper_client` for the TMR's Robotiq grippers.
* **franka_fr3_arm_controllers** - Contains the control packages for the Franka FR3 robotic arms.

# Deployment Guide

## Prepare the Pixi ROS 2 Environment

Enter the workspace:

```bash
cd ~/my_ros_ws
pixi shell
```

Verify that the environment is correctly configured:

```bash
echo $CONDA_PREFIX
which ros2
python --version
echo $ROS_DISTRO
```

The output should be similar to:

```bash
/home/demo/my_ros_ws/.pixi/envs/default
/home/demo/my_ros_ws/.pixi/envs/default/bin/ros2
Python 3.9.x
humble
```

If the `pixi shell` environment is not available, you will need to install and configure **Pixi** and **ROS 2 Humble** on the new machine before proceeding.

## Install Python Dependencies

```bash
cd ~/my_ros_ws
pixi shell

pip install dynamixel-sdk tyro evdev
```

If `colcon` is not available, install it using:

```bash
pixi add colcon-core colcon-common-extensions
```

Alternatively, you can install it with `pip`:

```bash
pip install colcon-core colcon-common-extensions
```

## Configure Device Permissions

The GELLO devices communicate through serial ports and require the **dialout** group permission:

```bash
sudo usermod -aG dialout $USER
```

The USB foot pedal is accessed via `/dev/input/eventX` and requires the **input** group permission:

```bash
sudo usermod -aG input $USER
```

After executing the above commands, **log out and log back in (or reboot)** for the changes to take effect.

Verify that the permissions have been applied:

```bash
groups
```

The output should include:

```bash
dialout input
```

## Build the ROS 2 Workspace

```bash
cd ~/Documents/code/teleoperation
colcon build --symlink-install
source install/setup.bash
```

If the build completes successfully, the ROS 2 workspace is ready to use.

# GELLO

## Verify the GELLO USB Devices

After connecting the GELLO devices, check the available serial devices:

```bash
ls /dev/serial/by-id/
```

On the current test machine, the two GELLO devices are identified as:

```text
usb-ROBOTIS_OpenRB-150_38F23AFA5157375037202020FF11170D-if00
usb-ROBOTIS_OpenRB-150_BDEDB3875157375037202020FF102618-if00
```

> **Note:** The USB device IDs may be different on a new computer. Be sure to verify the device IDs before updating the configuration.

> ⚠ **Verify which device is which side** - the two GELLOs are indistinguishable from
> their IDs alone, and having them swapped is not obvious: the arm follows nothing, or the
> calibration tool reports that the arm moved while "its" GELLO stayed still. To check,
> move **one** GELLO by hand and see which serial device changes:
>
> ```bash
> python3 src/franka_gello_state_publisher/scripts/gello_calibration_check.py \
>   --side left --monitor
> ```
>
> The `GELLO ... deg moved` row must respond to the **left** GELLO. If it does not, the
> `com_port` values in `franka_gello_duo.yaml` are swapped. (They were, until 2026-08-05.)

## GELLO Duo Configuration File

The configuration file is located at:

```text
~/Documents/code/teleoperation/src/franka_gello_state_publisher/config/franka_gello_duo.yaml
```

Example configuration:

```yaml
LEFT:
namespace: "left"
  com_port: "usb-ROBOTIS_OpenRB-150_38F23AFA5157375037202020FF11170D-if00"
  num_arm_joints: 7
  joint_signs: [1, 1, -1, 1, 1, 1, 1]
  gripper: true
  assembly_offsets: [4.712, 3.142, 4.712, 3.142, 4.712, 3.142, 4.712]
  gripper_range_rad: [2.521, 3.253]
  dynamixel_torque_enable: [0,0,0,0,0,0,0,0]
  dynamixel_goal_position: [0.0,0.0,0.0,-1.571,0.0,1.571,0.0,3.509]
  dynamixel_kp_p: [30,60,0,30,0,0,0,50]
  dynamixel_kp_i: [0,0,0,0,0,0,0,0]
  dynamixel_kp_d: [250,100,80,60,30,10,5,0]

RIGHT:
  namespace: "right"
  com_port: "usb-ROBOTIS_OpenRB-150_BDEDB3875157375037202020FF102618-if00"
  num_arm_joints: 7
  joint_signs: [1, 1, -1, 1, 1, 1, 1]
  gripper: true
  assembly_offsets: [1.571, 3.142, 1.571, 3.142, 1.571, 3.142, 0.000]
  gripper_range_rad: [2.570, 3.299]
  dynamixel_torque_enable: [0,0,0,0,0,0,0,0]
  dynamixel_goal_position: [0.0,0.0,0.0,-1.571,0.0,1.571,0.0,3.509]
  dynamixel_kp_p: [30,60,0,30,0,0,0,50]
  dynamixel_kp_i: [0,0,0,0,0,0,0,0]
  dynamixel_kp_d: [250,100,80,60,30,10,5,0]
```

> **Note:**
> For `com_port`, only specify the USB device ID. Do **not** include `/dev/serial/by-id/`, as the launch file will automatically prepend this path.

## Calibrating a GELLO (joint_signs and assembly_offsets)

`assembly_offsets` and `joint_signs` describe how each Dynamixel is physically mounted.
They are **per build** - they are not portable between GELLOs, and a wrong value is not
cosmetic: an inverted sign makes the impedance controller drive the arm toward a mirrored
pose, which can command past a joint limit and trip a `power_limit_violation` reflex.

> ⚠ **`get_offsets.py` cannot detect a wrong sign.** It computes `assembly_offsets` from
> the `--joint-signs` you give it and trusts them. Wrong signs produce offsets that look
> perfectly plausible.

### Check an existing calibration

With the arm bringup running (so joint states are published) and the GELLO publisher
**stopped** (it holds the serial port exclusively), pose the GELLO to match the arm and
run:

```bash
python3 src/franka_gello_state_publisher/scripts/gello_calibration_check.py --side left
```

It prints the GELLO's reported joint values next to the arm's real ones. Every diff should
be near zero. This is read-only and never commands the arm - run it with
`joint_impedance_controller` inactive or unspawned so a bad calibration cannot move
anything.

### Derive a calibration

```bash
python3 src/franka_gello_state_publisher/scripts/gello_calibration_check.py --side left --solve
```

This captures **two or more** arm poses. One pose is not enough: for either sign there is
an offset that fits it, so a single pose cannot determine the signs (verified in
simulation - choosing the sign whose offset lands nearest a multiple of 90 degrees is
wrong about half the time). With two poses the sign follows from the ratio of the deltas,
and `|dRaw/dArm|` must come out near 1.0, which doubles as a check that you matched the
poses properly.

For each pose: put the arm somewhere, match the GELLO to it by eye, press Enter. Between
poses move joints by roughly 35-150 degrees - too little and the direction is lost in
posing error, too much and the raw encoder delta wraps. The tool keeps asking for poses
until every joint is resolved, then prints a block to paste into
`franka_gello_duo.yaml`.

Afterwards re-run without `--solve` and confirm it reports `PASS` before starting the
publisher.

### The underlying mapping

```
normalized = mod((raw - assembly_offset) * joint_sign - MID, 2*pi) - pi + MID
```

which inverts to `assembly_offset = raw - joint_sign * (target + pi)`. Note the `+ pi`:
normalized values are wrapped into `[MID-pi, MID+pi)`, so the naive
`offset = raw - target*sign` is wrong by half a turn. `MID` is the midpoint of each
joint's range, so joints 4 and 6 (whose ranges are not centred on zero) have windows that
do not span `[-pi, pi)`.

## Start the GELLO Publisher

Terminal 1:

```bash id="6dj50s"
cd ~/my_ros_ws
pixi shell
cd ~/Documents/code/teleoperation
source install/setup.bash
ros2 launch franka_gello_state_publisher main.launch.py \
  config_file:=franka_gello_duo.yaml
```

Expected output:

```bash id="d3jsi0"
[left.gello_publisher]: Publishing GELLO joint states.
[right.gello_publisher]: Publishing GELLO joint states.
```

## Start the GELLO Subscriber

Terminal 2:

```bash id="1wy5b7"
cd ~/my_ros_ws
pixi shell
cd ~/Documents/code/teleoperation
source install/setup.bash
ros2 run franka_gello_state_subscriber franka_gello_state_subscriber
```

After moving the GELLO devices, you should see the states of the left arm, right arm, and grippers changing in the terminal output.

# Keyboard

## Start the Keyboard Publisher

Terminal 1:

```bash
cd ~/my_ros_ws
pixi shell
cd ~/Documents/code/teleoperation
source install/setup.bash
ros2 run keyboard_state_publisher keyboard_state_publisher
```

## Start the Keyboard Subscriber

Terminal 2:

```bash
cd ~/my_ros_ws
pixi shell
cd ~/Documents/code/teleoperation
source install/setup.bash
ros2 run keyboard_state_subscriber keyboard_state_subscriber
```

After pressing **W**, **A**, **S**, **D**, **Q**, or **E**, the subscriber will print the corresponding key press or action.

# Mobile teleoperation (two foot switches -> base + spine)

The TMR mobile base is an **omnidirectional swerve drive** and the torso is a **vertical
spine**. The two PCsensor foot switches (6 pedals total) drive them:

| Pedal | Motion | | Pedal | Motion |
|---|---|---|---|---|
| FS1.a | x+ (forward) | | FS2.a | x- (backward) |
| FS1.b | y+ (strafe left) | | FS2.b | y- (strafe right) |
| FS1.c | rotate CW | | FS2.c | rotate CCW |

Spine (hold to jog): **FS1.a + FS2.c = up**, **FS1.c + FS2.a = down**. While a spine combo
is held the base is held still.

`FS1` is the switch on USB port `2.1` (`/dev/f_pedal_l`), `FS2` is on `2.2`
(`/dev/f_pedal_r`). The mapping and speeds live in
`src/tmr_pedal_teleop/config/pedal_map.yaml` — edit it to retune.

### How the spine jog works (and why it is not a simple hold-and-halt)

The spine offers **absolute moves only**, and this device has **no usable pause**:

* `franka_spine_server`'s `Halt` posts to `/spine/api/motion:halt`, which **does not exist**
  on the device (HTTP 404). Its swagger declares only `motion:quick-stop`.
* `motion:quick-stop` is a **DS402 emergency stop**, not a pause — afterwards the device is
  `SwitchedOff` and needs an explicit switch-on. It must not be wired to a pedal release.
* Cancelling the action does **not** stop the hardware, and the server rejects new goals
  while a motion is in progress (`"another motion is in progress"`).

So `spine_bridge` jogs **incrementally**: while the combo is held it chains short absolute
moves (`jog_step`, default 0.02 m), starting each only after the previous finishes.
Releasing simply stops issuing moves, so the spine always comes to rest by itself.
**Worst-case overshoot after release is one `jog_step`.**

Two things that will bite anyone modifying this:

1. A **refused** motion (HTTP 424) returns a normal action result with `success=False` — it
   does *not* raise. Treating that as success marches the commanded target away from the
   spine's real position while nothing moves.
2. Retrying at tick rate keeps the device busy and floods 424s; back off (`retry_delay`).

## Distinguish the two foot switches (udev rule)

Both switches are identical (`3553:b001`, empty USB serial), so they can only be told
apart by **USB port**. Install the shipped rule to get stable device nodes:

```bash
sudo cp configs/99-pcsensor-footswitch.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules
sudo udevadm trigger --subsystem-match=input --action=add
ls -l /dev/f_pedal_l /dev/f_pedal_r   # verify
```

> **The symlinks land in `/dev`, not `/dev/input`** — `SYMLINK+=` is relative to `/dev`.

Whichever switch is on USB path `...usb-0:2.1:1.0` becomes `f_pedal_l` (switch **1**);
`2.2` becomes `f_pedal_r` (switch **2**). Each switch exposes three input devices on the
same path; `ENV{ID_INPUT_KEYBOARD}=="1"` selects the keyboard one that emits the a/b/c
keycodes. Find your paths with:

```bash
udevadm info -q property -n /dev/input/eventXX | grep -E 'ID_PATH|ID_INPUT'
```

`MODE="0666"` in the rule avoids needing the `input` group. If the symlinks are absent,
`pedal_state_publisher` falls back to the `by-path` nodes.

> `pedal_state_publisher` **grabs** both devices exclusively (so keystrokes don't leak into
> the focused window). A second instance cannot open them — kill any stray one before
> launching, or it will silently see no pedals.

## Networking (teleop host -> robot)

The bridges run on the teleop laptop and publish over DDS to the robot's already-running
controllers. Source the shipped env on the laptop:

```bash
source configs/tmr_laptop_env.sh   # native FastDDS, ROS_DOMAIN_ID=0, pinned to the Ethernet NIC
```

Both machines must use the **same `ROS_DOMAIN_ID`** (0). Nothing in the robot's shell rc
sets a domain, so its stack launches on 0 too.

`configs/fastdds_laptop_discovery.xml` pins DDS to the Ethernet NIC because the laptop is
multi-homed (WiFi + Ethernet). **Update the `<address>` in that file if the laptop's
Ethernet IP changes** (`ip -4 addr show | grep 172.16.16`).

Confirm discovery from the laptop:

```bash
ros2 topic list | grep -E "swerve|spine"
ros2 action list | grep spine
```

### ⚠ Clock sync is mandatory

`SwerveDriveController` ages each command against **its own** clock:

```cpp
const auto age_of_last_command = (time - msg->header.stamp).seconds();
if (age_of_last_command < cmd_vel_timeout_) { /* 0.5 s, else the command stays 0 */ }
```

`base_bridge` stamps with the **laptop** clock. If the two machines differ by more than
0.5 s, **every command is silently discarded and the base never moves** — with no error
logged anywhere. Check before blaming anything else:

```bash
# laptop
timedatectl | grep synchronized       # want: yes
# skew, measured directly
L1=$(date +%s.%N); R=$(ssh tmr-user@companion date +%s.%N); L2=$(date +%s.%N)
python3 -c "print(f'skew: {$R-($L1+$L2)/2:+.3f} s')"
```

Fix with `sudo systemctl restart systemd-timesyncd`, then **restart the teleop nodes** — a
step change in the clock upsets running ROS timers.

## Confirmed robot interfaces

Read from the robot's own sources / device swagger — **not guesses**:

| What | Name |
|---|---|
| Base command | `/swerve_drive_controller/cmd_vel` (`geometry_msgs/TwistStamped`) |
| Base limits | **0.1 m/s** (x, y), **0.1 rad/s** (yaw), accel 0.1 — anything larger is silently clamped |
| Base watchdog | `cmd_vel_timeout` 0.5 s → must publish faster than 2 Hz |
| Spine action | `/franka_spine_node/move_absolute` |
| Spine services | `/franka_spine_node/{switch_on,switch_off,get_position,get_state,fault_reset,get_parameters_spine}` |
| Spine / base IP | `172.16.16.10` |
| Arm IPs | **LEFT `172.16.16.12`, RIGHT `172.16.16.11`** |

Note `get_parameters_spine` — the plain name is taken by the standard ROS 2 parameter
service. The spine node registers everything **privately** (`~/...`) with an empty default
namespace.

## Run

### On the robot

`~/tams_ws` is the workspace to use (`franka_mobile`, `franka_spine_server` and the arm
controllers exist only there). A new terminal sources it automatically via `.bashrc`.

```bash
ros2 launch franka_bringup tmrv0_2.launch.py controller_name:=swerve_drive_controller
ros2 launch franka_spine_server spine.launch.py spine_ip:=172.16.16.10
```

### On the laptop

```bash
source <(cd ~/my_ros_ws && pixi shell-hook -s bash)   # ROS 2 Humble env
cd ~/Documents/code/teleoperation && source install/setup.bash
source configs/tmr_laptop_env.sh
ros2 launch tmr_pedal_teleop mobile_teleop.launch.py
```

The launch file starts `pedal_state_publisher`, `base_bridge`, and `spine_bridge` together.
All three lines are required — the pixi hook provides `ros2`, `install/setup.bash` provides
the packages, and the env script provides the DDS config.

**Shut down input-side first** (laptop nodes, then robot). The base watchdog then halts it,
rather than the base holding a last command.

> **Build note:** the message package `franka_spine_msgs` needs CMake to use the Pixi
> environment's Python. If message generation fails to find NumPy, build with:
> `colcon build --symlink-install --cmake-args -DPython3_EXECUTABLE=$(which python3)`.

# LABS data collection (pedals switch roles with `m`)

> Full setup walkthrough for a fresh machine:
> **[`docs/LABS_INTEGRATION.md`](docs/LABS_INTEGRATION.md)**. This section is the reference
> for how the mode switch and the pedal map behave.

Episode recording is done by [LABS](https://github.com/frankarobotics/labs), running as
containers on the teleop host. **LABS records; this workspace drives all the hardware.**
Its `franka-robot`, `controller-coordinator`, `franka-gello` and `robotiq-gripper`
services are commented out in `deployments/tmr_station/docker-compose.yml`, and the ZED
runs on the Jetson Orin.

## The mode switch

All six pedals are already spent on base motion plus two spine combos, so they cannot also
carry recording controls. Instead **`m` toggles what the whole set means**:

| Mode | Pedals do | Base and spine |
|---|---|---|
| `DRIVE` (default) | move the base and jog the spine | live |
| `RECORD` | drive LABS data collection | held still |

`m` is read by `keyboard_state_publisher` (its terminal must have focus). This cannot
clash with the pedals: `pedal_state_publisher` `grab()`s both foot switches exclusively, so
their a/b/c keystrokes never reach the terminal. `mode_manager` latches the current mode on
`/teleop/pedal_mode` with TRANSIENT_LOCAL durability, so a bridge that restarts picks it up
immediately instead of guessing.

**Switching to RECORD stops the base within one control cycle.** `base_bridge` keeps
publishing a *zero* `TwistStamped` at 20 Hz rather than stopping — the swerve controller's
0.5 s `cmd_vel_timeout` has to keep being fed, and a zero command is what actually holds
the base. `spine_bridge` stops issuing jog steps but lets the step in flight finish, since
this device has no usable halt.

## Pedal map in RECORD mode

`labs_pedal_bridge` polls `GET /api/v1/system/info` and calls the LABS REST API directly,
so nothing depends on which window has focus. `recording_state` wins over `workflow_state`,
matching the LABS UI's own button logic.

| LABS state | FS1.a | FS1.b | FS1.c | FS2.a | FS2.b | FS2.c |
|---|---|---|---|---|---|---|
| IDLE | — | start teleop | — | — | — | — |
| READY | — | stop teleop | sync robots | — | — | — |
| SYNCING | — | stop teleop | — | — | — | — |
| FOLLOWING | **start recording** | stop teleop | — | — | — | — |
| RECORDING | **stop recording** | — | — | — | — | — |
| REVIEWING | — | — | — | save successful | save failed | discard |

Retune it in `src/tmr_pedal_teleop/config/pedal_map.yaml` (`map_<STATE>_<TOKEN>`), no code
change needed.

## Run

```bash
ros2 launch tmr_pedal_teleop mobile_teleop.launch.py \
  labs_url:=http://localhost:3001 \
  task_id:=<uuid from deployments/tmr_station/config_tasks.yml>
```

`task_id` is **required** to start a recording. LABS keeps its selected task in a browser
cookie, so there is no server-side "current task" to read — an episode started by pedal
uses the id given here, which can differ from what the browser shows. Keep them in sync.

Add `record:=false` to bring up motion only, with no LABS bridge or state publishers.

## What gets recorded, and the one rule that breaks episodes

Beyond the arms and cameras, three nodes here exist purely so base and spine motion reach
the dataset:

| Node | Publishes | Why |
|---|---|---|
| `mobile_base_state_bridge` | `/mobile_base/pose` (`PoseStamped`), `/mobile_base/twist` (`TwistStamped`) | LABS understands neither `nav_msgs/Odometry` nor `/tf` at dataset-build time |
| `spine_state_publisher` | `/spine/joint_states` (`JointState`) at 50 Hz | the spine has no state topic at all, only services |
| `spine_bridge` | `/spine/target_height` (`Float32`) at 20 Hz | gives the spine an *action* dimension |

> ⚠ **Every recorded topic must publish continuously for the whole episode.** LABS'
> `TemporalSynchronizer` takes the latest first-message time and the earliest last-message
> time across all configured topics and raises `SynchronizationError` if either is more
> than 1 s from the episode bounds — so one event-driven publisher fails the *entire*
> conversion, not just its own column. That is why each of these publishes at a fixed rate
> even when idle, republishing its last known value rather than skipping a tick.

Confirm the base odometry topic before the first recording — it is a parameter because it
has not been verified on hardware:

```bash
ros2 topic list | grep -i swerve
ros2 topic info /swerve_drive_controller/odometry
```

## DDS

LABS ships configured for CycloneDDS on `ROS_DOMAIN_ID` 100. The TMR and the Jetson are
native Fast DDS on domain 0, and all participants in a ROS 2 graph must share one RMW, so
the TMR station config moves LABS to Fast DDS/domain 0 instead. Its `fastdds_labs.xml`
mirrors `configs/fastdds_laptop_discovery.xml` here — each host needs its own copy with its
own IP in `interfaceWhiteList`.

## Installing the LABS side

We have read-only access to `frankarobotics/labs`, so the changes ship in this repo under
[`labs_integration/`](labs_integration/) and are applied on top of a clean checkout:

```bash
git clone git@github.com:frankarobotics/labs.git ~/Documents/code/labs
cd ~/Documents/code/labs && git checkout 6a62eb5
./bootstrap.sh && bash && task install:dependencies && git submodule update --init

~/Documents/code/teleoperation/labs_integration/apply_to_labs.sh ~/Documents/code/labs
```

See [`labs_integration/README.md`](labs_integration/README.md) for what the patch changes
and how to regenerate it.

# Arm teleoperation (GELLO -> FR3 duo)

The arms are **not** part of the mobile bringup. `franka_bringup`'s `tmrv0_2.urdf.xacro` is
explicitly the *"standalone, no arms"* base variant — its `TmrHardware` ros2_control system
holds only drive joints and cartesian-velocity IO. Each arm runs its **own**
controller_manager against its own FCI, so the two stacks do not conflict.

`franka_fr3_arm_controllers` and `franka_gripper_manager` must be **deployed to the robot**
(they bind to the FR3 hardware interface and cannot bridge from the laptop like the base
and spine do). They live in `~/tams_ws/src/` and are built there.

```bash
# robot
ros2 launch franka_fr3_arm_controllers franka_fr3_arm_controllers.launch.py \
  robot_config_file:=tmr_duo_config.yaml
# laptop
ros2 launch franka_gello_state_publisher main.launch.py config_file:=franka_gello_duo.yaml
```

`config/tmr_duo_config.yaml` is the TMR-specific config:

* `arm_id: fr3v2` (not `fr3`) — joints are `left_fr3v2_joint1..7` / `right_fr3v2_joint1..7`,
  built as `${arm_prefix}${robot_type}_jointN`; `fr3v2.urdf.xacro` inserts the `_` itself,
  so `arm_prefix` is `left`/`right` with **no** trailing underscore.
* `namespace` must equal `arm_prefix`: the controller derives its interface prefix from its
  **node namespace**, and subscribes to the relative topic `gello/joint_states`, which
  resolves to `/left/gello/joint_states`.
* `load_gripper: "false"` and `joint_sources: ["joint_states"]` — the TMR uses **Robotiq**
  grippers, not the Franka Hand, so pulling `franka_gripper/joint_states` would hang.
* **LEFT = .12, RIGHT = .11** — the *reverse* of the upstream `example_fr3_duo_config.yaml`
  convention. Do not "fix" it to match the template.

> ⚠ Bring up **one arm at a time** the first time, and confirm the left/right mapping
> before two 7-DOF arms share a workspace.
>
> You do **not** need to pre-match each GELLO to its arm. `on_activate` captures the arm
> pose and the GELLO pose in the same instant, so the initial delta is zero and the mapped
> target starts at the arm's *current* pose — the controller logs exactly that. (An earlier
> version of this warning said the arm "drives toward whatever pose the GELLO is currently
> in"; that described the old absolute mapping and is no longer true.) What still matters is
> keeping your hands **off** the GELLOs during activation: any movement between activation
> and the first update becomes an approach target, traversed at
> `motion_generator_speed_factor` (0.05, deliberately slow).
>
> The controller also spawns `--inactive` by design, so activation is an explicit gate.
> Activate **both arms from one process**, on the robot:
>
> ```bash
> python3 ~/activate_arms.py
> ```
>
> ⚠ Do **not** run `ros2 control set_controller_state` twice, once per arm. Each CLI call
> creates a new DDS participant, and participant discovery here is a 15–25 s exchange on a
> link already carrying three 1 kHz FCI streams. That burst aborts the arm that is *already*
> active. Measured 2026-08-23: LEFT activated at t=457671.9, LEFT reflex at 457674.7,
> RIGHT activated at 457675.2 — the left arm was killed by the command that started the
> right one. `activate_arms.py` waits for both services before switching either.
> See [`docs/RUNBOOK.md`](docs/RUNBOOK.md) S3.

## Start both arms from the home pose

> ℹ️ **This is now automatic.** `~/start_robot.bash` homes both arms after the grippers and
> before the base, reading `~/teleop_home_pose.yaml`. It warns and counts down 5 s first;
> `--no-home` disables it. The manual procedure below still applies if you skip it or need
> to re-home mid-session.

**Both arms must be at the agreed home pose before you activate teleop.** The impedance
controller maps GELLO→arm as a *delta* from the poses captured at `on_activate`, so the arm
follows from wherever it happens to be standing. Activate it away from home and the whole
correspondence is offset — teleop still "works", it just does not match.

`franka_msgs/action/PTPMotion` is the vendor point-to-point mover and drives them there.
Deactivate `joint_impedance_controller` first — it holds the command interfaces. One arm at
a time:

```bash
ros2 control set_controller_state joint_impedance_controller inactive -c /left/controller_manager

# 7 values per side from configs/teleop_home_pose.yaml -> <SIDE>.arm_joint_positions
ros2 action send_goal /left/action_server/ptp_motion franka_msgs/action/PTPMotion \
  "{goal_joint_configuration: [-1.072185, -0.082723, 1.024787, -2.746873, 1.176839, 1.976895, 0.168207],
    maximum_joint_velocities: [0.15,0.15,0.15,0.15,0.15,0.15,0.15], goal_tolerance: 0.01}"

ros2 action send_goal /right/action_server/ptp_motion franka_msgs/action/PTPMotion \
  "{goal_joint_configuration: [0.809728, -0.335639, -0.800341, -2.792884, -1.182067, 1.760234, -0.095578],
    maximum_joint_velocities: [0.15,0.15,0.15,0.15,0.15,0.15,0.15], goal_tolerance: 0.01}"
```

`status: 2` is `TARGET_REACHED`. At 0.15 rad/s both arms reached home to within 0.0003 rad
from roughly 2 rad away. `/<side>/action_server/error_recovery` is also available.

> ⚠ **`/<side>/franka/joint_states` is NOT ordered `joint1..7`.** Observed orders were
> `[1,2,3,4,5,7,6]` on the left and `[3,4,7,6,2,1,5]` on the right — `joint_state_publisher`
> aggregates and does not sort. Always zip `msg.name` with `msg.position`; reading the array
> positionally silently returns another joint's value. It first showed up as joint 4
> apparently sitting outside its own limit. The controller itself is unaffected — it uses
> ros2_control state interfaces, not this topic.

## Grippers (Robotiq 2F-85)

Working as of 2026-08-16. The grippers run **on the robot**, one `controller_manager` per
gripper, independent of the arm stacks:

```bash
# robot
ros2 launch franka_gripper_manager robotiq_gripper_controller_client.launch.py \
  config_file:=tmr_duo_config_robotiq.yaml
# laptop (same command as arm teleop - the GELLO publisher drives both)
ros2 launch franka_gello_state_publisher main.launch.py config_file:=franka_gello_duo.yaml
```

Use `config/tmr_duo_config_robotiq.yaml`, **not** `example_fr3_duo_config_robotiq.yaml` —
the upstream example lists FTDI serials (`DA8BS24U` / `DA8BU5F3`) that do not exist on this
robot. This robot has `DAAQM5UJ` (left) and `DAAQM4W3` (right).

The GELLO publishes a `std_msgs/Float32` percent on
`<ns>/gripper/gripper_client/target_gripper_width_percent`; `robotiq_gripper_client`
converts it to a `GripperCommand` action goal. **1.0 = open, 0.0 = closed.** Unlike the base
and the arms, this message has **no header**, so nothing ages it — the gripper path is
immune to the clock skew described above and works even on an unsynced robot clock.

### ⚠ Bind each gripper to its ARM, never to the operator's hand

**The operator's hands are crossed with respect to the namespaces, and that is intended.**
Verified end to end on 2026-08-16 with both arms live and both grippers driven:

| namespace | GELLO device | operator's hand | arm | gripper FTDI |
| --- | --- | --- | --- | --- |
| `left` | `BDEDB387` | **right** hand | `left_fr3v2` (.12) | `DAAQM4W3` |
| `right` | `38F23AFA` | **left** hand | `right_fr3v2` (.11) | `DAAQM5UJ` |

So the GELLO in your right hand drives the *left* arm and the *left* arm's gripper. Do not
"fix" that — it is the arrangement the operator wants, and it was confirmed deliberately
after a session spent flip-flopping over it.

**The invariant that actually matters is coherence, not handedness:** whichever arm a GELLO
moves, its trigger must close the gripper *bolted to that same arm*. Because the gripper and
the arm share a namespace, that holds automatically as long as each gripper's `com_port` is
bound to the namespace of the arm it is mounted on. Test it that way too — "does this GELLO
move an arm and close the gripper on that same arm?" needs no left/right words and cannot be
answered wrongly from the operator's frame.

The gripper binding deliberately does **not** follow the udev symlink names: the robot's rule
calls `DAAQM4W3` `r_gripper_r`, but that gripper is bolted to the arm the `left` namespace
drives. The rule does usefully set `MODE:="0666"`, so no `dialout` membership is needed.
Never use raw `/dev/ttyUSBn` — that numbering is not stable across reboots.

**The trap.** A gripper must be bound to the arm it is physically bolted to, *not* to
whichever hand it feels correct under. Those two differ whenever the GELLO→namespace
assignment is itself crossed — and then "the trigger in my right hand closes the gripper on
my right" is **true while the gripper is still on the wrong arm**. That is exactly what
happened here: the gripper config was flipped to satisfy the operator's hand, it looked
correct with the arms switched off, and the error only surfaced once the arms were
activated.

**Diagnose by splitting the chain into three independently measurable links**, and establish
link 2 *first* — it is the anchor, and recollection is no substitute for measuring it:

```bash
# 1. GELLO device -> namespace. Fully objective, no human judgement:
#    record both topics while ONE trigger is squeezed, then compare spans.
ros2 topic echo /left/gripper/gripper_client/target_gripper_width_percent --field data
ros2 topic echo /right/gripper/gripper_client/target_gripper_width_percent --field data

# 2. namespace -> arm. Activate one side and watch which arm follows.
ros2 control set_controller_state joint_impedance_controller active -c /left/controller_manager

# 3. namespace -> gripper. Stop the GELLO publisher first (30 Hz, it overrides manual
#    goals), drive the two to visibly opposite states, and check the CLOSED one is bolted
#    to the arm identified in step 2.
ros2 action send_goal /left/gripper/robotiq_gripper_controller/gripper_cmd \
  control_msgs/action/GripperCommand "{command: {position: 0.78, max_effort: 1.0}}"
ros2 action send_goal /right/gripper/robotiq_gripper_controller/gripper_cmd \
  control_msgs/action/GripperCommand "{command: {position: 0.0, max_effort: 1.0}}"
```

Do **not** make "which one moved, left or right?" the primary test. Both readings (robot
frame vs operator frame) are self-consistent, so the answer merely confirms whatever
hypothesis you already held — it produced two confident wrong answers in a single session.

**`gello_joint_directions` does not follow the device.** It lives in
`franka_fr3_arm_controllers/config/controllers.yaml` and is keyed by **namespace**, but it
belongs to the physical unit `BDEDB387`, whose `joint_signs` are wrong standalone. The
working configuration is:

```
left  : BDEDB387 [-1,1,1,-1,1,1,-1] * [-1,-1,1,1,1,1,-1]   = [1,-1,1,-1,1,1,1]
right : 38F23AFA [1,-1,1,-1,1,1,1]  * default [1]*7        = [1,-1,1,-1,1,1,1]
```

Only the **product** `joint_signs * gello_joint_directions` steers behaviour. If you ever
rebind which GELLO serves which arm, you must move this override by hand — and be aware that
reproducing the same product on the other arm is **not** sufficient. That was tried on
2026-08-16: the devices were swapped, both products were preserved at `[1,-1,1,-1,1,1,1]`,
and joint 2 still ran backwards on the right arm, needing a further flip. Treat a rebind as
requiring a fresh per-joint direction check (`gello_calibration_check.py --side <s> --live`),
not as bookkeeping.

### A frozen gripper reading usually means nobody touched it

`gripper_position_raw` is read straight from the servo with no normalisation, so a GELLO at
rest reads a **bit-identical** value sample after sample. 858 identical samples is normal,
not a dead servo. The driver's `_read_group` *raises* on a failed sync read rather than
returning stale data, so any value you receive at all came from a successful read of every
servo. Check the resting value against `gripper_range_rad`: at rest it should sit at or just
past the open end (measured 3.3050 vs a configured open of 3.293).


# Troubleshooting

## Everything looks perfect but the base does not move

**Check the clock skew first.** See *Clock sync is mandatory* above. The signature is
distinctive and localises the fault in one step:

```bash
ros2 topic echo /swerve_drive_controller/cmd_vel       # x: 0.05  -> command IS arriving
ros2 topic echo /swerve_drive_controller/cmd_vel_out   # all 0.0  -> controller REJECTED it
```

`cmd_vel` non-zero + `cmd_vel_out` zero = the controller is discarding commands as stale.
No error is logged anywhere. (Observed once at **+31 s** skew.)

> **Sampling these topics is timing-sensitive.** A short `ros2 topic echo` while nobody is
> pressing a pedal shows zeros and reads exactly like "no command". Record to a file over a
> long window while the pedal is held.

## `ros2 topic echo` says a laptop-local topic "does not appear to be published"

`configs/tmr_laptop_env.sh` pins DDS to the Ethernet NIC, and that whitelist blocks
visibility of topics published by **other processes on the same laptop**. The data still
flows to the robot correctly. For local inspection use a scratch terminal with:

```bash
unset FASTRTPS_DEFAULT_PROFILES_FILE
```

## Robot: `tmrv0_2.launch.py` not found in `franka_bringup`

The package resolved to `~/ros2_ws`, which holds a **stale** franka stack (~4 months behind
`~/tams_ws`). Check:

```bash
echo "$AMENT_PREFIX_PATH" | tr ':' '\n' | grep -c ros2_ws   # want 0
ros2 pkg prefix franka_bringup                              # want .../tams_ws/...
```

`.bashrc` sources `~/tams_ws/install/**local_setup.bash**` — *not* `setup.bash`.
`setup.bash` chains to the workspace's recorded parent (`~/ros2_ws`, because tams_ws was
built while it was sourced) and re-injects the stale overlay. Since colcon prepends
only-if-absent, once `ros2_ws` is in front it **stays** in front. `local_setup.bash` adds
only tams_ws's own packages.

> `~/ros2_ws` is kept on disk as a fallback. It is the only source of the ZED wrapper, which
> is therefore not on the path by default — `source ~/ros2_ws/install/setup.bash` when needed.

## Robot: a terminal still resolves the wrong workspace after fixing `.bashrc`

`exec bash` **inherits the environment** — `AMENT_PREFIX_PATH` survives, and colcon will not
move a prefix that is already present, so re-sourcing cannot fix it either. **Open a genuinely
new terminal.** To repair one in place:

```bash
unset AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH AMENT_CURRENT_PREFIX
exec bash
```

## Robot: every ROS package looks "missing" over SSH

The login shell is **zsh**, where `source /opt/ros/humble/setup.bash` silently fails (it
needs `BASH_SOURCE`). Run remote ROS commands as `bash -ic "..."`. Note `bash -lc` is also
wrong — a *login* shell reads `.bash_profile`, not `.bashrc`, so nothing gets sourced.

Also beware zsh aborting an entire command on an unmatched glob (`ls ~/x/*foo*`), which
silently truncates probe output and can read as "not installed".

## Spine refuses every move with HTTP 424 "invalid state or busy"

Happens after a motion request times out: the device sticks in a busy state while still
reporting `SwitchedOn` with `error_code 0`. `fault-reset` does nothing (there is no fault).
Cycle the state machine — this commands **no** motion:

```bash
ros2 service call /franka_spine_node/switch_off franka_spine_msgs/srv/SwitchOff
ros2 service call /franka_spine_node/switch_on  franka_spine_msgs/srv/SwitchOn
```

Device health checks (read-only):

```bash
curl -sk https://172.16.16.10/spine/api/state        # "SwitchedOn"
curl -sk https://172.16.16.10/spine/api/position-mm  # {"position":398}
curl -sk https://172.16.16.10/spine/api/error        # {"error_code":0,...}
curl -sk https://172.16.16.10/spine/api/swagger.json # authoritative endpoint list
```

> Raw `curl -X POST` to this device returns **nginx 400** regardless of state — the app never
> sees it. Only GETs work by hand. To POST, replicate what `requests` sends: a Session with
> `Content-Type: application/json`, `verify=False`, and `json=<data>`.

## Known bug: `franka_spine_server` position units

`GetPosition` is documented in metres but returns the device's raw **millimetres**
(e.g. `398`). `GetParameters` *does* convert (limits are genuine metres, 0–0.77) and
`start_motion` *does* convert (m → mm). `spine_bridge._normalise_position()` detects the
discrepancy by magnitude rather than hard-coding a ÷1000, so it keeps working if the vendor
fixes it.

## Different USB IDs on a New Computer

The USB IDs assigned to the GELLO devices may differ on a new computer. To check the current device IDs, run:

```bash
ls /dev/serial/by-id/
```

Then update the corresponding entries in:

```text
franka_gello_state_publisher/config/franka_gello_duo.yaml
```

To identify the USB foot pedal devices, run:

```bash
ls -l /dev/input/by-id/
udevadm info -q property -n /dev/input/eventXX | grep -E 'ID_PATH|ID_INPUT'
```

If the USB paths differ, update the `ID_PATH==` values in
`configs/99-pcsensor-footswitch.rules` and reinstall the rule (see *Distinguish the two
foot switches* above). `pedal_state_publisher` takes its device nodes from the
`device1_candidates` / `device2_candidates` parameters — first existing wins — so the
`by-path` fallbacks keep it working even before the rule is installed.

# License & third-party code

This repository is licensed under the Apache License, Version 2.0 (see [LICENSE](LICENSE)).

`src/franka_fr3_arm_controllers/` is adapted from [gello_software](https://github.com/wuphilipp/gello_software), which vendors it under Apache-2.0; the directory retains its own [LICENSE](src/franka_fr3_arm_controllers/LICENSE) and [NOTICE](src/franka_fr3_arm_controllers/NOTICE) (Franka Robotics GmbH), which apply to that code.

`src/franka_spine_msgs/` is vendored verbatim from
[franka_ros2](https://github.com/frankarobotics/franka_ros2) (`franka_spine/franka_spine_msgs`,
version 2.5.1, Apache-2.0, Franka Robotics GmbH). It is copied here only so that
`tmr_pedal_teleop`'s spine bridge can be built on a teleop laptop that does not have the
full `franka_ros2` source tree; the interface definitions are unmodified.









