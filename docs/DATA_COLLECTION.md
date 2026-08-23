# Data collection and visualisation

How to record a teleoperation episode and look at what you recorded. For operating the robot
itself see [`RUNBOOK.md`](RUNBOOK.md); for first-time laptop setup see
[`LABS_INTEGRATION.md`](LABS_INTEGRATION.md).

Everything below writes plain **MCAP** bags with [`../record_bag.bash`](../record_bag.bash).
That is deliberately independent of LABS: the bag proves the topics exist and are gapless
*before* you commit to the LABS pipeline, and MCAP is what LABS' `lerobot_mcap_reader`
consumes, so the same bag converts later.

---

## 1. Bring everything up

```bash
# LAPTOP - GELLO leaders, foot pedals, state publishers; also corrects clock skew
cd ~/teleoperation
./start_teleop.bash --record --task-id b06d85e1-c053-49db-acbb-685fc64ddf8a

# ROBOT - sensors, spine, arms, grippers, home the arms, base LAST, activate arms
~/start_robot.bash --restart
```

`--record` matters: without it `mobile_base_state_bridge` and `spine_state_publisher` do not
run, and you lose 14 of the 62 state dimensions with no error anywhere.

Order matters too. Creating a DDS participant is a 15-25 s discovery burst on this network,
and such a burst aborts FCI control loops that are already running - laptop first, base last.
See [`RUNBOOK.md`](RUNBOOK.md) S1.

## 2. Check before you record

```bash
./record_bag.bash --check
```

**You want `28/28`.** This takes ~30 s because it spins for 25 s; Fast DDS discovery here is
slow and a shorter spin reports a half-discovered graph.

If something is missing, [`RUNBOOK.md`](RUNBOOK.md) S8 has the table. The usual causes:

| missing | cause |
|---|---|
| `/mobile_base/*`, `/spine/joint_states` | teleop started without `--record` |
| `/head_camera/...` | ZED not started - run `~/start_zed.bash` on the robot |
| `/lidar_*/scan` | robot started with `--no-sensors` |
| `/wrist_camera_*` | check `lsusb \| grep 0b5b` - both D405s should be present |

## 3. Record

```bash
./record_bag.bash --name pick_place        # everything; Ctrl+C to stop
./record_bag.bash --no-video --name test   # state/action/lidar only, much smaller
./record_bag.bash --out ~/episodes/run1
```

Bags land in `~/teleop_bags/<timestamp>_<name>/` as MCAP. Roughly **3 MB/s** without video;
with all three cameras expect ~10x that.

**Read the summary it prints on exit.** It lists any topic that recorded **zero messages** -
the one failure mode `--check` cannot see, because a controller can advertise a topic and
never publish to it. Three separate bugs of exactly this kind were found on 2026-08-23.

## 4. Verify what you recorded

```bash
./record_bag.bash --info                   # or, by hand:
docker exec -u $(id -u):20 -e HOME=/tmp gello-humble bash -lc \
  'source /opt/ros/humble/setup.bash && ros2 bag info /workspace/teleop_bags/<BAG>'
```

Check three things:

1. **No `Count: 0`.** One silent topic fails the whole LeRobot conversion, not just its own
   column - LABS' `TemporalSynchronizer` compares the latest first-message time and earliest
   last-message time across every topic and rejects the episode if either is more than 1 s
   from the bounds.
2. **Duration** matches how long you actually recorded.
3. **Rates look sane** - divide count by duration. Expect roughly: arm state ~1000 Hz,
   grippers ~275 Hz, GELLO ~30 Hz, lidars ~34 Hz, spine 50 Hz, base pose/twist ~50 Hz,
   cameras 15-30 Hz.

---

## 5. Visualising a recorded bag

All GUI tools run **inside the Humble container** - the laptop host runs Kilted and cannot
talk to a Humble bag or the robot. `start_teleop.bash` passes the X11 socket through, so
windows appear on your desktop.

> If a GUI fails with `cannot connect to display`, the container predates the X11 change.
> Recreate it: `./start_teleop.bash stop && docker rm -f gello-humble && ./start_teleop.bash`

A shell inside the container, used by everything below:

```bash
./start_teleop.bash shell
```

It warns if the container has no X11 socket and tells you how to recreate it. Note that
your `$HOME` is mounted at `/workspace`, so bags are at `/workspace/teleop_bags/...` inside.

> Running `ros2 ...` on the **host** will not work. The host is ROS 2 Kilted while the robot
> and the bags are Humble; cross-distro, `ros2 node list` returns empty and service calls
> time out. That is why everything runs in the container.

### Replay the bag

```bash
ros2 bag play /workspace/teleop_bags/<BAG> --loop
```

Everything below subscribes to the replay exactly as if the robot were live. Use
`--rate 0.5` to slow down, `--start-offset 30` to skip ahead, `-p` to start paused
(space steps).

> ⚠ Replay publishes on the **same topics as the live robot**. Do it with the robot stack
> **down**, or a replayed `/swerve_drive_controller/cmd_vel` will drive the real base.

### Camera images

```bash
ros2 run rqt_image_view rqt_image_view
```

Pick the topic from the dropdown:
`/wrist_camera_left/camera/color/image_raw`, `/wrist_camera_right/...`, or
`/head_camera/zed_node/rgb/color/rect/image`.

### Lidar, TF and the robot model

```bash
rviz2
```

Set **Fixed Frame** to `base_link` (or `lidar_front` if TF is not being replayed), then
**Add -> By topic** and pick `/lidar_front/scan` and `/lidar_rear/scan` as LaserScan
displays. Add **TF** to see the frames.

For the live robot with the full model, the sensor package ships a ready config:

```bash
# on the ROBOT, over ssh -X
ros2 launch franka_mobile_sensors franka_mobile_sensors.launch.py \
  start_cameras:=false start_lidars:=true start_rviz:=true
```

### Numeric signals over time

```bash
ros2 run rqt_plot rqt_plot
```

Add topic fields by path, for example:

```
/mobile_base/twist/twist/linear/x
/spine/joint_states/position[0]
/left/gello/joint_states/position[0]
/left/franka_robot_state_broadcaster/measured_joint_states/position[0]
```

Plotting the GELLO joint against the matching arm joint is the quickest way to confirm the
follower actually tracked the leader.

### Quick numbers without a GUI

```bash
ros2 topic echo --once /mobile_base/pose
ros2 topic hz /lidar_front/scan
```

### Plotting from Python

`matplotlib` and `numpy` are available in the container; `mcap`, `rosbags` and `pandas` are
not. Either install one (`pip3 install mcap-ros2-support`) or replay the bag and subscribe
with `rclpy`, which needs no extra dependency.

---

## 6. Known gaps

* **The D455 base cameras are unplugged.** They drew 2160 mA and starved the ZED (512 mA) on
  a shared port until it looped on `CAMERA REBOOTING`. They were unused anyway - the sensor
  stage runs `start_cameras:=false`. If you want them back they need a powered hub.
* **LABS itself is not installed on this laptop** - no `~/Documents/code/labs`, nothing on
  `:3001`. `--record --task-id` starts the LABS bridge, which will simply fail to reach the
  API until LABS is up. The bag is unaffected. See `LABS_INTEGRATION.md` S4.
* **Topic names differ from the LABS defaults** on this robot, and the configs here are
  corrected accordingly: `/<side>/gripper/joint_states` (not `gripper_joint_states`),
  `/head_camera/zed_node/rgb/color/rect/image` (not `rgb/image_rect_color`), and
  `/swerve_drive_controller/odom` (not `odometry`). Each of those cost a silent, empty column
  before it was found.
