# LABS integration delta

Everything needed to turn a stock [LABS](https://github.com/frankarobotics/labs) checkout
into the TMR recording station.

**Why it lives here and not in LABS.** `frankarobotics/labs` is the vendor's repository and
we have read access only, so our changes cannot be pushed there. Keeping them in this repo
also means the recorder's topic list and the nodes that publish those topics are versioned
together — which matters, because they have to agree exactly or episodes silently lose
columns.

## Contents

| Path | What |
|---|---|
| `apply_to_labs.sh` | applies the patch and installs the station config into a LABS checkout |
| `labs-tmr-integration.patch` | three source fixes + the `wmem_max` sysctl, against LABS `6a62eb5` |
| `tmr_station/` | the station config: Fast DDS profile, trimmed compose, embodiment, recorder topic list, GR00T modality |

## Use

```bash
git clone git@github.com:frankarobotics/labs.git ~/Documents/code/labs
cd ~/Documents/code/labs && git checkout 6a62eb5
./bootstrap.sh && bash && task install:dependencies && git submodule update --init

~/Documents/code/teleoperation/labs_integration/apply_to_labs.sh ~/Documents/code/labs
```

Re-running is safe — an already-applied patch is detected and skipped, and an existing
`tmr_station/` is moved aside rather than overwritten.

Then edit `deployments/tmr_station/fastdds_labs.xml` so `<address>` is **this host's** IP on
the robot subnet, and `task build && task start`.

Full walkthrough: [`../docs/LABS_INTEGRATION.md`](../docs/LABS_INTEGRATION.md).

## What the patch changes

| File | Change | Why |
|---|---|---|
| `dataset-builder/.../lerobot_mcap_reader.py` | `_handle_state_topic` gains a `TwistStamped` branch | `TwistStamped` was handled only as an *action*. Base velocity **observations** were logged as "unsupported schema" and dropped. |
| `data-collection/.../franka_robot.py` | `trigger_controller_coordinator` returns early with no coordinators | Otherwise `pending` starts at 0, `_on_done` never fires, and every teleop endpoint burns a full 7 s timeout before succeeding. |
| `data-collection/.../teleop.py` | `start_syncing` advances SYNCING → FOLLOWING with no coordinators | That transition is driven by a coordinator reporting in. With none the workflow sticks in SYNCING — and the UI's **Start recording** button only exists in the FOLLOWING button set. |
| `data-collection`, `data-recorder`, `realsense-camera` `entrypoint.sh` | add `net.core.wmem_max` | The DDS profile asks for a 10 MiB send buffer; Ubuntu's 212992 B default silently clamps it. |
| `.gitignore` | un-ignore `deployments/tmr_station` | Optional — only matters if you also want it tracked inside your LABS checkout. |

All three source fixes are upstream gaps rather than TMR-specific hacks, and are worth
contributing back to `frankarobotics/labs`.

## Keeping the patch current

After changing anything under `services/` in your LABS checkout:

```bash
cd ~/Documents/code/labs
git diff -- services .gitignore \
  > ~/Documents/code/teleoperation/labs_integration/labs-tmr-integration.patch
```

And after changing the station config:

```bash
rm -rf ~/Documents/code/teleoperation/labs_integration/tmr_station
cp -R deployments/tmr_station ~/Documents/code/teleoperation/labs_integration/tmr_station
```

If you bump LABS past `6a62eb5`, update `BASE_COMMIT` in `apply_to_labs.sh` too.
