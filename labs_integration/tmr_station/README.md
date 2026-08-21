# TMR Station — Franka Mobile FR3 Duo

LABS deployment for the **Mobile FR3 Duo ("TMR")**: two FR3 arms with Robotiq grippers on
an omnidirectional swerve base with a vertical spine.

Here LABS is **recording + UI only**. All hardware is driven by the teleoperation
workspace ([EBiM-Benchmark/teleoperation](https://github.com/EBiM-Benchmark/teleoperation),
branch `feat/tmr-mobile-teleop`), which owns the arms, grippers, GELLO leaders, base,
spine and foot pedals.

## Before you start it: three things to edit

1. **`fastdds_labs.xml` → `<address>`** — set it to *this host's* IP on the robot subnet
   (`ip -4 addr show | grep 172.16.16`). It is the local interface to bind to, not the
   robot's address. Getting this wrong means no DDS discovery at all and every device
   shows OFFLINE.
2. **`config_station.yml` → the arm state topics** — verify against the running stack:
   ```bash
   ros2 topic list | grep -E "franka_robot_state_broadcaster|gripper_joint_states"
   ```
   The exact broadcaster topic set depends on the `franka_ros2` version the teleoperation
   workspace builds against.
3. **The teleoperation side's `pedal_map.yaml` → `labs_pedal_bridge.task_id`** — a task
   UUID from `config_tasks.yml` here. Without it the pedal cannot start a recording.

## How it differs from `example_station`

| Area | Change | Why |
|---|---|---|
| DDS | CycloneDDS/domain 100 → **native Fast DDS/domain 0**, `cyclonedds.xml` → `fastdds_labs.xml` | The TMR's own stack and the Jetson Orin are Fast DDS on domain 0. All participants in a ROS 2 graph must share one RMW, so LABS moves rather than the robot. |
| Networking | localhost-only → **multi-host** (LABS PC ↔ TMR ↔ Jetson) | `example_station` pinned CycloneDDS to `127.0.0.1`; every node ran on one box. |
| Services | `franka-robot`, `controller-coordinator`, `franka-gello`, `robotiq-gripper`, `zed-camera-head` commented out of `docker-compose.yml`, `Tiltfile` and `Taskfile.yml` | Teleoperation owns the hardware; the ZED runs on the Jetson. |
| Embodiment | Added `mobile_base` + `spine` teleop robots and `mobile_base_state` + `spine_state` observers | `example_station` has no notion of a mobile base. |
| Topics | All **absolute** (leading `/`) | See "The absolute-topic rule" below. |
| `leader_topic` | Always `''` | Declares recording semantics only; LABS must not republish leader→follower when teleoperation already commands the robot. |
| `follower_device` | Never `franka_fr3` | That exact value makes `franka_robot.py` create controller-coordinator clients, which never resolve with the coordinator container disabled. |

## The absolute-topic rule

`dataset-builder`'s `topic_manifest.py` reads `leader_topic`/`follower_topic` straight out
of `config_station.yml` and **never calls `resolve_topic()`**, then string-matches them
against `config_data_recorder.yml`. Namespace-relative topics never match, and
`_filter_recorded_topics` drops them with only a `logger.warning` — so the dataset silently
loses those columns. `example_station` has this problem today: its action topics are
relative while its recorder list is absolute.

Absolute topics pass through `resolve_topic()` unchanged, so runtime behaviour is
identical and the manifest matches. **Keep every topic in this file absolute.**

Cross-check after any config edit:

```bash
grep -oE "'/[^']+'" config_data_recorder.yml | tr -d "'" | sort -u > /tmp/rec.txt
grep -oE "^\s+(- |follower_topic: |topic: )/\S+" config_station.yml \
  | grep -oE "/\S+" | sort -u > /tmp/cfg.txt
comm -23 /tmp/cfg.txt /tmp/rec.txt   # must be empty
```

## Every configured topic must publish continuously

`TemporalSynchronizer` takes the **latest** first-message time and the **earliest**
last-message time across all configured topics, and raises `SynchronizationError` if
either is more than 1 s from the episode bounds. An event-driven publisher fails the whole
episode conversion, not just its own column.

This is why the teleoperation side publishes at a fixed rate even when idle:
`base_bridge` sends a zero twist at 20 Hz, `spine_bridge` republishes the current height
as the commanded target, and `spine_state_publisher` republishes its last known reading at
50 Hz while polling the device at only 5 Hz.

## Patches this station depends on

Three small changes in `services/` (upstream gaps, worth contributing back):

- `dataset-builder/.../lerobot_mcap_reader.py` — `_handle_state_topic` gained a
  `TwistStamped` branch. Without it the base's velocity observations are logged as an
  unsupported schema and dropped.
- `data-collection/.../franka_robot.py` — `trigger_controller_coordinator` returns early
  when no coordinators are configured, instead of burning a 7 s timeout on every teleop
  endpoint.
- `data-collection/.../teleop.py` — `start_syncing` advances SYNCING → FOLLOWING when no
  coordinators exist. The UI's **Start recording** button only exists in the FOLLOWING
  button set, so without this it never appears.

Plus `net.core.wmem_max` added to the kept services' `entrypoint.sh`: the DDS profile asks
for a 10 MiB send buffer, and Ubuntu's 212992 B default would silently clamp it.

## Known issue: camera bandwidth

`/head_camera/zed_node/rgb/image_rect_color` is **uncompressed** `sensor_msgs/Image`. With
the ZED on the Jetson it now crosses the network: roughly 80 MB/s at HD720@30 and 186 MB/s
at HD1080@30. Size the link and the ZED config (on the Jetson) accordingly.

Publishing `CompressedImage` instead is not an option — `lerobot_mcap_reader` explicitly
drops that schema and `data-processor` only converts topics whose schema is exactly
`sensor_msgs/msg/Image`.

## `modality.json` is not authoritative until you check it

It is purely descriptive; nothing validates it against the data. The committed offsets
assume 7 arm joints and 1 gripper joint per side. After the first conversion:

```bash
jq '.features["observation.state"].shape, .features.action.shape' \
   ../../data/datasets/lerobot/<name>/meta/info.json    # expect [62] and [23]
```

and correct the file if they differ. Note `example_station`'s own `modality.json` is wrong
in exactly this way — it declares 56 state dims by assuming `measured_joint_states` also
yields velocity, but `_handle_joint_state` reads `.position` only.
