# TMR Teleoperation & Data Collection Pipeline

Complete snapshot of our three-machine teleoperation and MCAP data-collection setup,
packaged 2026-08-26 from the live machines. LABS is **not** used by this pipeline —
recording is done with plain `ros2 bag` (MCAP) via `station/record_bag.bash`.

## Machine layout

| Directory | Machine | Role |
|---|---|---|
| `laptop/` | operator laptop (`tams123`) | GELLO leader arms, foot pedals, state publishers, teleop start scripts. Pixi-managed ROS 2 workspace. |
| `station/` | recording PC (`ebim@ebimHP`) | episode recording (`record_bag.bash`), bag inspection/conversion (`bag_to_video.py`, `verify_topics.py`), central docs. |
| `station/robot/` | TMR companion (`tmr-user@companion`, Jetson AGX Orin) | robot-side bringup: spine, arms, grippers, base, sensors. Deployed to the companion's home dir with `station/robot/deploy.bash`. Synced from the live companion on 2026-08-26. |

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
