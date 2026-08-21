# Bringing up teleoperation + LABS on a new laptop

Start-to-finish setup for the **Franka Mobile FR3 Duo ("TMR")** with episode recording:
GELLO-driven arms, foot-pedal-driven base and spine, and [LABS](https://github.com/frankarobotics/labs)
capturing episodes to MCAP → LeRobot.

**Who owns what.** This workspace drives *all* the hardware — arms, grippers, GELLO, base,
spine, pedals. LABS only records and provides the operator UI. Its `franka-robot`,
`controller-coordinator`, `franka-gello` and `robotiq-gripper` services stay switched off,
and the ZED camera runs on the Jetson Orin rather than on the laptop.

**Which machine runs what.** Three hosts, and getting this wrong is the most common
setup mistake:

| Host | Runs |
|---|---|
| **Robot** | base + spine bringup, `franka_fr3_arm_controllers`, `franka_gripper_manager` (these bind to the FR3 hardware interface and cannot bridge over DDS) |
| **Laptop** | GELLO publisher, pedals, `tmr_pedal_teleop`, all LABS containers |
| **Jetson Orin** | the ZED camera wrapper |

**The pedals do double duty.** All six are spent on base motion plus two spine combos, so
they cannot also carry recording controls. Pressing **`m`** flips what the whole set means:

| Mode | Pedals | Base + spine |
|---|---|---|
| `DRIVE` (default) | move the base, jog the spine | live |
| `RECORD` | drive LABS data collection | held still |

---

## 0. What you need

**Hardware**

- Teleop laptop: amd64, ≥8 physical cores, ≥16 GB RAM, Ubuntu with Docker
- Wired Ethernet to the robot subnet `172.16.16.0/24` (WiFi may stay up; DDS is pinned to
  the Ethernet NIC)
- 2× Franka GELLO (USB, ROBOTIS OpenRB-150)
- 2× PCsensor foot switch (3 pedals each, USB)
- 2× Robotiq 2F-85 with FTDI RS-485 adapters
- Jetson Orin with the ZED camera
- ≥70 GB free disk, plus room for episodes

**Known addresses** (verified on the robot, not guesses)

| What | Address |
|---|---|
| Base + spine | `172.16.16.10` |
| RIGHT arm | `172.16.16.11` |
| LEFT arm | `172.16.16.12` |

---

## 1. Clone and build this workspace

```bash
cd ~/Documents/code
git clone git@github.com:EBiM-Benchmark/teleoperation.git
cd teleoperation
git checkout feat/labs-integration
```

Set up the Pixi ROS 2 Humble environment and build — see the main
[README](../README.md#deployment-guide) for the full walkthrough. In short:

```bash
cd ~/my_ros_ws && pixi shell            # provides ros2 (Humble, Python 3.9)
cd ~/Documents/code/teleoperation
pip install dynamixel-sdk tyro evdev
colcon build --symlink-install
source install/setup.bash
```

> **Build note:** `franka_spine_msgs` needs CMake to use the Pixi environment's Python. If
> message generation cannot find NumPy:
> `colcon build --symlink-install --cmake-args -DPython3_EXECUTABLE=$(which python3)`

### Device permissions

GELLO is on serial, the foot switches are on `/dev/input/event*`:

```bash
sudo usermod -aG dialout,input $USER
# log out and back in, then confirm:
groups        # must list both 'dialout' and 'input'
```

### Pin the foot switches to stable device nodes

Both switches are physically identical (VID:PID `3553:b001`, empty USB serial), so they
can only be told apart by which port they are in:

```bash
sudo cp configs/99-pcsensor-footswitch.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && sudo udevadm trigger
ls -l /dev/f_pedal_l /dev/f_pedal_r
```

`/dev/f_pedal_l` is **FS1**, `/dev/f_pedal_r` is **FS2**.

> USB paths are not stable across replugging — they changed twice in one session on the
> original laptop. `pedal_state_publisher` falls back to autodetecting by vendor:product
> and ordering by USB path, which is deterministic but arbitrary with respect to which
> switch is physically on the left. **Check it:** press one pedal, see whether it reports
> `1x` or `2x`, and swap `device1_candidates`/`device2_candidates` if it comes out
> backwards.

### Update the per-device IDs

USB IDs differ per machine. On this laptop, find and update:

```bash
ls /dev/serial/by-id/     # GELLO (OpenRB-150) and Robotiq (FTDI) devices
```

- GELLO → `src/franka_gello_state_publisher/config/franka_gello_duo.yaml` (`com_port`) —
  this one is read on the **laptop**
- Robotiq → `src/franka_gripper_manager/config/tmr_duo_config_robotiq.yaml` (`com_port`) —
  read on the **robot**, so edit it in the robot's `~/tams_ws` checkout

> ⚠ Bind each gripper to the arm it is **bolted to**, not to whichever hand feels right.
> The operator's hands are crossed with respect to the namespaces on this robot — the
> GELLO in your right hand drives the `left` namespace. See the header comment in
> `tmr_duo_config_robotiq.yaml`; it documents a real mis-binding that only surfaced once
> the arms were powered.

---

## 2. Clock sync — do this before you debug anything else

`SwerveDriveController` ages every command against **its own** clock and silently discards
anything older than 0.5 s. If the laptop and robot clocks differ by more than that, the
base never moves and **nothing is logged anywhere**.

```bash
timedatectl | grep synchronized          # want: yes

# measure the actual skew
L1=$(date +%s.%N); R=$(ssh tmr-user@companion date +%s.%N); L2=$(date +%s.%N)
python3 -c "print(f'skew: {$R-($L1+$L2)/2:+.3f} s')"
```

`configs/sync_robot_clock.sh` fixes it in one command. **Restart the teleop nodes
afterwards** — a step change in the clock upsets running ROS timers.

The TMR's companion has no reachable NTP source, so this drifts. Re-check it whenever the
robot has been off for a while.

---

## 3. Networking: everything on Fast DDS, domain 0

The TMR's own stack and the Jetson both run **native Fast DDS on `ROS_DOMAIN_ID` 0** —
nothing in the robot's shell rc sets a domain. All participants in a ROS 2 graph must share
one RMW implementation, so LABS moves to Fast DDS rather than the robot moving to
CycloneDDS.

**On the laptop**, before running anything:

```bash
source configs/tmr_laptop_env.sh
```

Then edit `configs/fastdds_laptop_discovery.xml` so `<address>` is **this laptop's**
Ethernet IP:

```bash
ip -4 addr show | grep 172.16.16
```

This is the single most common setup failure. The whitelist takes the **local** interface
to bind to, not the robot's address. Get it wrong and there is no discovery at all — every
device shows OFFLINE and no topic appears.

Verify multicast works over the link, then that you can see the robot:

```bash
# on the robot:  ros2 multicast receive
ros2 multicast send

ros2 topic list | grep -E "swerve|spine"
ros2 action list | grep spine
```

> **Gotcha:** the interface whitelist also hides topics published by *other processes on
> this same laptop*, so `ros2 topic echo` on a local topic will report "does not appear to
> be published" even though data reaches the robot fine. Unset
> `FASTRTPS_DEFAULT_PROFILES_FILE` in a scratch shell to inspect local topics.

> **Domain 0 is the default domain**, so anything else ROS 2 on the LAN joins your graph.
> The interface whitelist is what actually isolates you. If you later see cross-talk, move
> the robot, Jetson and LABS to one non-zero domain together.

---

## 4. Install LABS

We have **read-only** access to `frankarobotics/labs`, so our changes cannot live there.
They ship in this repo under `labs_integration/` and are applied on top of a clean
checkout.

```bash
cd ~/Documents/code
git clone git@github.com:frankarobotics/labs.git
cd labs
git checkout 6a62eb5          # the commit the patch is built against
./bootstrap.sh && bash
task install:dependencies
git submodule update --init

~/Documents/code/teleoperation/labs_integration/apply_to_labs.sh ~/Documents/code/labs
```

That applies three source fixes and installs `deployments/tmr_station/`. Then do the one
thing the script cannot:

```bash
# set <address> to THIS host's IP on the robot subnet
nano ~/Documents/code/labs/deployments/tmr_station/fastdds_labs.xml
```

Build and start:

```bash
cd ~/Documents/code/labs/deployments/tmr_station
task build        # first time only, ~20 min
task start        # Tilt dashboard on :10360
```

You should get: data-collection (`:3001`, Swagger at `/docs`), data-recorder (`:3002`),
data-processor, the UI on **`:4000`**, and postgres on `:5433`.

### What the patch changes, and why

If you ever need to reapply these by hand:

| File | Change | Why |
|---|---|---|
| `dataset-builder/.../lerobot_mcap_reader.py` | `_handle_state_topic` gains a `TwistStamped` branch | It handled `TwistStamped` only as an *action*. Base velocity **observations** were logged as "unsupported schema" and silently dropped. |
| `data-collection/.../franka_robot.py` | `trigger_controller_coordinator` returns early with no coordinators | Otherwise `pending` starts at 0, `_on_done` never fires, and every teleop endpoint burns its full 7 s timeout before succeeding. |
| `data-collection/.../teleop.py` | `start_syncing` advances SYNCING → FOLLOWING with no coordinators | SYNCING → FOLLOWING is driven by a coordinator reporting in. With none, the workflow sticks in SYNCING — and the UI's **Start recording** button only exists in the FOLLOWING button set. |
| three `entrypoint.sh` | add `net.core.wmem_max` | The DDS profile asks for a 10 MiB send buffer; Ubuntu's 212992 B default silently clamps it. |

---

## 5. Configure the ZED on the Jetson

The ZED does **not** run on the laptop. On the Jetson, run the stock
`zed_wrapper/zed_camera.launch.py` with:

```bash
export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE=/path/to/fastdds_jetson.xml   # Jetson's OWN IP
ros2 launch zed_wrapper zed_camera.launch.py camera_name:=head_camera camera_model:=zedm
```

`camera_name:=head_camera` is what produces `/head_camera/zed_node/rgb/image_rect_color`,
which is the topic the station config expects. Copy `fastdds_labs.xml` to the Jetson and
put the **Jetson's** IP in its `interfaceWhiteList`.

Confirm from the laptop:

```bash
ros2 topic hz /head_camera/zed_node/rgb/image_rect_color
```

> ⚠ **Bandwidth.** That topic is *uncompressed* `sensor_msgs/Image` — roughly 80 MB/s at
> HD720@30, 186 MB/s at HD1080@30. Sizing the link is a real decision, not a detail.
> Publishing `CompressedImage` instead is **not** an option: the whole LABS video path
> keys on the schema being exactly `sensor_msgs/msg/Image`. If the link cannot carry it,
> lower the ZED's resolution/FPS in the Jetson's config.

---

## 6. Start the robot-side stacks

**`franka_fr3_arm_controllers` and `franka_gripper_manager` run on the robot, not the
laptop** — they bind to the FR3 hardware interface and cannot bridge over DDS the way the
base and spine do. They live in `~/tams_ws/src/` on the robot and are built there. Only the
base/spine/pedal/LABS side runs on the laptop.

Each block is its own terminal on the robot (`~/tams_ws` is sourced by `.bashrc`):

```bash
# base + spine
ros2 launch franka_bringup tmrv0_2.launch.py controller_name:=swerve_drive_controller
ros2 launch franka_spine_server spine.launch.py spine_ip:=172.16.16.10

# arms  (note: robot_config_file, not config_file)
ros2 launch franka_fr3_arm_controllers franka_fr3_arm_controllers.launch.py \
  robot_config_file:=tmr_duo_config.yaml

# grippers - one controller_manager per gripper, independent of the arm stacks
ros2 launch franka_gripper_manager robotiq_gripper_controller_client.launch.py \
  config_file:=tmr_duo_config_robotiq.yaml
```

> ⚠ Bring up **one arm at a time** the first time and confirm the left/right mapping before
> two 7-DOF arms share a workspace. See the main README's arm-teleoperation section.

---

## 7. Start the laptop side

Every terminal: inside `pixi shell`, with `source install/setup.bash` and
`source configs/tmr_laptop_env.sh`.

**GELLO leaders** (drives both the arms and the grippers)

```bash
ros2 launch franka_gello_state_publisher main.launch.py config_file:=franka_gello_duo.yaml
```

**Base, spine, pedals and LABS**

```bash
ros2 launch tmr_pedal_teleop mobile_teleop.launch.py \
  labs_url:=http://localhost:3001 \
  task_id:=b06d85e1-c053-49db-acbb-685fc64ddf8a
```

`task_id` comes from `labs_integration/tmr_station/config_tasks.yml` and is **required** to
start a recording. LABS keeps its selected task in a browser cookie, so there is no
server-side "current task" to read — an episode started by pedal uses the id given here,
which can differ from what the browser shows. Keep them in sync by hand.

That launch starts eight nodes:

| Node | Role |
|---|---|
| `pedal_state_publisher` | reads both foot switches via evdev, publishes `/pedal/state` |
| `keyboard_state_publisher` | reads `m` (and w/a/s/d/q/e) — **its terminal must have focus** |
| `mode_manager` | owns DRIVE/RECORD, latches it on `/teleop/pedal_mode` |
| `base_bridge` | pedals → `/swerve_drive_controller/cmd_vel` |
| `spine_bridge` | spine combos → `MoveAbsolute`; also publishes `/spine/target_height` |
| `labs_pedal_bridge` | pedals → LABS REST API in RECORD mode |
| `mobile_base_state_bridge` | odometry → `/mobile_base/pose` + `/mobile_base/twist` |
| `spine_state_publisher` | polls the spine → `/spine/joint_states` |

Add `record:=false` for motion only, with no LABS bridge or state publishers.

---

## 8. Pedal reference

### DRIVE mode

| Pedal | Motion | | Pedal | Motion |
|---|---|---|---|---|
| FS1.a | x+ (forward) | | FS2.a | x− (backward) |
| FS1.b | y+ (strafe left) | | FS2.b | y− (strafe right) |
| FS1.c | rotate CW | | FS2.c | rotate CCW |

Spine, hold to jog: **FS1.a + FS2.c = up**, **FS1.c + FS2.a = down**. While a spine combo
is held the base is held still.

### RECORD mode

`recording_state` wins over `workflow_state`, matching the LABS UI's own button logic.

| LABS state | FS1.a | FS1.b | FS1.c | FS2.a | FS2.b | FS2.c |
|---|---|---|---|---|---|---|
| IDLE | — | start teleop | — | — | — | — |
| READY | — | stop teleop | sync robots | — | — | — |
| SYNCING | — | stop teleop | — | — | — | — |
| FOLLOWING | **start recording** | stop teleop | — | — | — | — |
| RECORDING | **stop recording** | — | — | — | — | — |
| REVIEWING | — | — | — | save successful | save failed | discard |

Retune in `src/tmr_pedal_teleop/config/pedal_map.yaml` (`map_<STATE>_<TOKEN>`) — no code
change needed. Speeds, combos and topics live in the same file.

---

## 9. First-run checklist

Work down it; each step is independently checkable and failures higher up cause confusing
symptoms lower down.

**1. DDS reachability**

```bash
ros2 topic list | grep -E "swerve|spine|head_camera"
```
All three must appear from the laptop. Nothing below works until they do.

**2. Mode switch, robot powered off**

```bash
ros2 topic echo /teleop/pedal_mode                      # press m -> DRIVE <-> RECORD
ros2 topic echo /swerve_drive_controller/cmd_vel
```
In RECORD the twist must be **all zeros but still publishing at 20 Hz**, and stomping any
pedal must change nothing. Start the `echo` *after* the launch, to confirm the
TRANSIENT_LOCAL latch delivers the mode to a late subscriber.

**3. Pedal → LABS, robot still off**

With the pedals in RECORD, walk the table:
FS1.b (start teleop) → FS1.c (sync) → FS1.a (start rec) → FS1.a (stop rec) → FS2.a (save).

```bash
docker compose -f ~/Documents/code/labs/deployments/tmr_station/docker-compose.yml \
  logs -f data-collection | grep "State transition"
curl -s localhost:3001/api/v1/system/info | jq
```
The browser UI on `:4000` must show the same states.

**4. Confirm the base odometry topic** — it is a parameter because it has not been checked
on hardware:

```bash
ros2 topic list | grep -i swerve
ros2 topic info /swerve_drive_controller/odometry
```
If it differs, set `mobile_base_state_bridge.odom_topic` in `pedal_map.yaml` **and** the
matching entries in the station and recorder configs.

**5. Base drives.** Power the robot, DRIVE mode, stomp FS1.a. If it does not move,
compare:

```bash
ros2 topic echo /swerve_drive_controller/cmd_vel       # x: 0.05  -> command IS arriving
ros2 topic echo /swerve_drive_controller/cmd_vel_out   # all 0.0  -> controller REJECTED it
```
Rejected almost always means clock skew (§2).

**6. Record an episode** with arm, base and spine motion in it, then check every topic
actually landed:

```bash
ros2 bag info ~/Documents/code/labs/data/raw_episodes/YYYY/MM/DD/<uuid>/mcap/mcap_0.mcap
```
Every topic in `config_data_recorder.yml` must be present with a **non-zero** message
count. Zero means a wrong name or DDS not carrying it.

**7. Export the dataset**

```bash
cd ~/Documents/code/labs
task dataset-builder-convert-episode DEPLOYMENT_DIR=/workspace/deployments/tmr_station
```
Grep the log for `Skipping configured` and `Ignoring configured observation topic` — those
two warnings are how *every* misconfiguration in this system announces itself.

**8. Fix `modality.json`** against reality, not arithmetic:

```bash
jq '.features["observation.state"].shape, .features.action.shape' \
   data/datasets/lerobot/<name>/meta/info.json          # expect [62] and [23]
```
The committed offsets assume 7 arm joints and 1 gripper joint per side. Correct the file if
the real shapes differ.

**9. Safety re-check.** With the base driving, press `m` — it must stop within one control
cycle. Then kill `pedal_state_publisher`; the base must stop via the controller's own 0.5 s
watchdog. **Always shut down laptop-side nodes before the robot**, so the watchdog halts
the base rather than the base holding a last command.

---

## 10. Troubleshooting

**The base does not move and nothing is logged.** Clock skew (§2), nine times out of ten.
`cmd_vel_out` all-zero while `cmd_vel` is non-zero confirms it.

**Every device is OFFLINE in the LABS UI.** `<address>` in `fastdds_labs.xml` is wrong —
it must be this host's own IP, not the robot's.

**`ros2 topic echo` says a laptop-local topic is not published.** Expected: the interface
whitelist hides same-host publishers. Unset `FASTRTPS_DEFAULT_PROFILES_FILE` in a scratch
shell.

**Pedals do nothing.** Are you in DRIVE or RECORD? Check `ros2 topic echo
/teleop/pedal_mode`. If `m` does nothing, the `keyboard_state_publisher` terminal does not
have focus — it uses a blocking `stdin.read`.

**`m` also types into the terminal.** Expected — it is a normal keystroke. The *pedals*
are `grab()`-ed exclusively, so they never leak; the keyboard is not.

**A pedal reports the wrong switch (`1x` vs `2x`).** USB path ordering. Swap
`device1_candidates` / `device2_candidates`, or fix the udev rule for the ports actually in
use.

**Spine refuses every move, HTTP 424.** The device is in a bad state:

```bash
ros2 service call /franka_spine_node/switch_off franka_spine_msgs/srv/SwitchOff
ros2 service call /franka_spine_node/switch_on  franka_spine_msgs/srv/SwitchOn
curl -sk https://172.16.16.10/spine/api/state
```
If it starts *during* recording, lower `spine_state_publisher.poll_rate` — its polling
competes with `spine_bridge`'s jog steps.

**Dataset conversion fails with `SynchronizationError`.** Some configured topic did not
publish across the whole episode. LABS takes the latest first-message time and the earliest
last-message time over *all* topics and rejects the episode if either is more than 1 s from
the bounds — so one gappy publisher fails the whole conversion. Check `ros2 bag info` for a
topic with a suspiciously low message count.

**A dataset column is silently missing.** A topic in `config_station.yml` is absent from
`config_data_recorder.yml`, or is written namespace-relative rather than absolute. The
station README has a cross-check script; run it after any config edit.

---

## Related

- [`README.md`](../README.md) — GELLO calibration, arm bring-up, robot interface reference
- [`labs_integration/tmr_station/README.md`](../labs_integration/tmr_station/README.md) —
  what the station config does and why, and the absolute-topic rule
- [`src/tmr_pedal_teleop/config/pedal_map.yaml`](../src/tmr_pedal_teleop/config/pedal_map.yaml) —
  every tunable in one file
