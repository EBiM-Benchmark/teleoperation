# TMR Teleoperation & Data Collection Pipeline

Complete snapshot of our three-machine teleoperation and MCAP data-collection setup.
`station/` and `station/robot/` were refreshed 2026-08-29 from the live station and
companion; `laptop/` is still the 2026-08-26 packaging (no access to `tams123` from this
session). LABS is **not** used by this pipeline — recording is done with plain `ros2 bag`
(MCAP) via `station/record_bag.bash`.

## Machine layout

| Directory | Machine | Role |
|---|---|---|
| `laptop/` | operator laptop (`tams123`) | GELLO leader arms, foot pedals, state publishers, teleop start scripts. Pixi-managed ROS 2 workspace. |
| `station/` | recording PC (`ebim@ebimHP`) | episode recording (`record_bag.bash`), bag inspection/conversion (`bag_to_video.py`, `verify_topics.py`), central docs. Synced from the live station on 2026-08-29. |
| `station/robot/` | TMR companion (`tmr-user@companion`, Jetson AGX Orin) | robot-side bringup: spine, arms, grippers, base, sensors. Deployed to the companion's home dir with `station/robot/deploy.bash`. Synced from the live companion on 2026-08-29. |

## What's new since the 2026-08-26 packaging

- Task-based episode recording (`record_bag.bash --task/--target/--status`) and a
  configurable bag root (`--bag-root` / `TMR_BAG_ROOT`) instead of a fixed `~/teleop_bags`.
- Fast DDS now defaults to UDP-only on the companion (the SHM transport fails there
  repeatedly) and the station renders a live wired-interface whitelist
  (`render_dds_profile.sh`) instead of using a hardcoded, DHCP-fragile address.
- `start_robot.bash` bounds every spawner wait with `timeout`, auto-repairs the base and
  `franka_robot_state_broadcaster` after the bringup's own spawner races, and sweeps
  leftover orphan processes on `--restart` even when nothing looks stale.
- `tmr_health.py` (arm/base health without creating a new DDS participant, which is itself
  what used to trigger the failures) and `verify_topics.py` (confirms a dataset's topics
  carry real data, not just that a publisher exists) are now started/available by default.
- A companion-side Xbox gamepad (`start_gamepad.bash`) can drive the base alongside the
  foot pedals, with any pressed pedal always taking priority.
- `camera_viewer.py` / `start_camera_viewer.bash` (read-only 3-camera operator view) and
  `bag_to_video.py` (export a bag's cameras to MP4) round out the station-side tooling.
- `station/robot/fastdds_udp_only.xml` picked up a real bugfix from the companion: the
  previous `<receiverBufferSize>` element name is invalid and made Fast DDS silently
  reject the whole profile at parse time.

## Where to start

1. `station/docs/RUNBOOK.md` — operating the robot day-to-day.
2. `station/docs/DATA_COLLECTION.md` — recording an episode and checking the bag.
3. `laptop/RUNBOOK.md` and `laptop/README.md` — laptop-side setup and teleop bringup.
4. `station/robot/README.md` — what each robot-side script does and why.

Typical session:

```bash
# LAPTOP
cd laptop && ./start_teleop.bash --record --task-id <task-uuid>

# COMPANION (robot)
~/start_robot.bash --restart

# STATION (ebimHP) — record an episode
./record_bag.bash
```

## What was excluded

Build artifacts (`build*/`, `install*/`, `log*/`), `.git` history (upstream:
https://github.com/EBiM-Benchmark/teleoperation), `.pixi` envs, all `*.bak*` backup
variants, and everything LABS-related.

## Known issues in this snapshot

- `laptop/src/tmr_pedal_teleop/launch/mobile_teleop.launch.py` and
  `laptop/src/tmr_pedal_teleop/tmr_pedal_teleop/base_bridge.py` contain **unresolved git
  conflict markers** (`<<<<<<< Updated upstream` / `>>>>>>> Stashed changes`) carried over
  from the laptop working tree. The `station/src/` copies of these packages are clean —
  resolve against those before building the laptop workspace.
- `station/docs/DATA_COLLECTION.md` links to a `LABS_INTEGRATION.md` that was removed from
  this bundle (LABS is unused).
