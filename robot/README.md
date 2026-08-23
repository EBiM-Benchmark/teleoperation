# Robot-side scripts

These run **on the TMR companion** (`companion`, `tmr-user@172.16.16.50`, a Jetson AGX
Orin), not on the laptop. They live in the companion's home directory; this folder is the
versioned copy.

They are here because they were previously stored **only** on the robot, where their only
history was a scatter of `.bak` files (`start_robot.bash.bak`,
`start_robot.bash.before-sigkill`, ...). Losing or reflashing that machine would have lost
them.

| file | role |
|---|---|
| `start_robot.bash` | brings up every robot stack in order: spine, arms, grippers, home the arms, then the base **last**. `--restart` stops what is running and sweeps orphans; `--skip-arms` for base+spine only; `--no-home` skips homing |
| `activate_arms.py` | activates `joint_impedance_controller` on **both** arms from ONE DDS participant. Using two `ros2 control` calls kills the first arm — see [`../docs/RUNBOOK.md`](../docs/RUNBOOK.md) S3 |
| `base_health.sh` | read-only: reports whether the mobile base can actually move. The one check that exposes the silent `unconfigured` failure |
| `base_nudge.py` | commands the base directly with properly stamped messages (`ros2 topic pub` does not stamp) |
| `fastdds_wifi.xml` | DDS-on-WiFi profile. **Not in use** — see RUNBOOK S9 |
| `start_base.bash`, `start_upper.bash`, `start_zed.bash` | older single-stage helpers, superseded by `start_robot.bash` but kept as reference |

`teleop_home_pose.yaml` is deliberately **not** duplicated here: it is owned by
[`../configs/teleop_home_pose.yaml`](../configs/teleop_home_pose.yaml) and deployed from
there, so there is one source of truth.

## Deploying

```bash
./robot/deploy.bash            # copy repo -> robot, backing up anything that differs
./robot/deploy.bash --check    # show what differs, change nothing
./robot/deploy.bash --pull     # copy robot -> repo, to capture edits made on the robot
```

## The sync problem, and how to remove it

Copying is a workaround: edit on the robot and the repo silently goes stale. `--check`
before every session is the mitigation.

The real fix is to make the robot's home-directory files **symlinks into a clone**, so
there is nothing to sync:

```bash
# on the robot, once
git clone <this repo> ~/teleoperation
cd ~
for f in start_robot.bash activate_arms.py base_health.sh base_nudge.py; do
  [ -L "$f" ] || mv "$f" "$f.pre-symlink"
  ln -sf ~/teleoperation/robot/"$f" "$f"
done
```

After that, `git pull` on the robot is the deployment step and edits are committed where
they are made. This has **not** been done yet — the robot has no clone of this repo, and
setting one up needs git credentials on that machine.

## Passwordless clock correction (one-time setup)

Clock skew over 0.5 s makes the base silently discard `cmd_vel` and the arms reject GELLO
samples, with nothing logged — so `start_teleop.bash` now checks and corrects it
automatically. Correcting needs root on the robot, and prompting for a password defeats the
point, so a **narrow** rule allows exactly one script:

```bash
# 1. laptop -> robot key auth, so no password for the measurement half
ssh-copy-id companion

# 2. install the helper + sudoers rule (asks for the robot password ONCE)
scp robot/tmr-set-clock robot/tmr-clock-sudoers companion:/tmp/
ssh -t companion 'sudo install -m 0755 -o root -g root /tmp/tmr-set-clock /usr/local/sbin/tmr-set-clock &&
                  sudo install -m 0440 -o root -g root /tmp/tmr-clock-sudoers /etc/sudoers.d/tmr-clock &&
                  sudo visudo -cf /etc/sudoers.d/tmr-clock &&
                  rm -f /tmp/tmr-set-clock /tmp/tmr-clock-sudoers && echo INSTALLED'

# 3. verify - should print a skew line with no prompt
./configs/sync_robot_clock.sh --check
```

`tmr-set-clock` takes one signed-seconds argument and **validates it strictly**, because it
runs as root without a password. `sync_robot_clock.sh` prefers it and falls back to the old
interactive path when it is absent.

> The robot password is deliberately **not** stored anywhere in this repo. Key auth plus this
> one rule achieves the same "no prompt" result without a secret on disk, which would
> otherwise end up committed or copied.
