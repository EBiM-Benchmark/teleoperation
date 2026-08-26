#!/usr/bin/env python3
"""Validate - or derive - a GELLO's joint_signs and assembly_offsets against the real arm.

Why this exists
---------------
``get_offsets.py`` computes ``assembly_offsets`` but **trusts** the ``--joint-signs`` you
pass it; it cannot detect a wrong sign. A wrong sign produces offsets that look plausible
and yield *inverted* motion, so the impedance controller drives toward a mirrored pose.
That can command past a joint limit and trip a reflex (``power_limit_violation``) before
anyone sees a number.

The mapping used by the publisher is::

    normalized = mod((raw - offset) * sign - MID, 2*pi) - pi + MID

which inverts to::

    offset = raw - sign * (target + pi)          (mod 2*pi)

Note the ``+ pi``: normalized values are wrapped into ``[MID-pi, MID+pi)``, so a naive
``offset = raw - target*sign`` is wrong by half a turn.

CHECK vs SOLVE vs ANCHOR
------------------------
*check* (default) validates the CURRENT config at a single pose. This is reliable: if the
GELLO is posed like the arm and the numbers disagree, the config is wrong.

*solve* derives signs and offsets. It needs **two different arm poses**, because a single
pose cannot determine the signs - for any sign there is an offset that fits, so both signs
explain one pose equally well. (Verified in simulation: picking the sign whose offset lands
nearest a multiple of 90 degrees is wrong roughly half the time.) With two poses the sign
follows from the ratio of the deltas, which is also self-validating: |dRaw/dArm| must be
close to 1.

*anchor* re-derives ONLY the offsets, so that the GELLO's natural RESTING pose maps onto a
chosen arm pose. It reuses the signs already in the config, so run it AFTER *solve* and
after pasting the derived signs in. It needs neither the arm nor ROS - only the GELLO.

Why anchor is worth doing: the impedance controller maps GELLO to arm as a DELTA from the
poses captured at activation, so the two never need to be posed alike. What the offsets
still control is where each joint sits inside its wrap window ``[MID-pi, MID+pi)``. A joint
resting near a window edge flips by 2*pi on a small hand movement, and the commanded goal
then walks 6.28 rad away at the rate limit. Anchoring the rest pose to a mid-window arm
pose puts every joint as far from that cliff as the geometry allows, and as a bonus makes
"let go of the leader" the correct starting state for every session.

Safety
------
Read-only. It reads the GELLO over USB and subscribes to the arm's joint states; it never
commands the arm. Run it with ``joint_impedance_controller`` INACTIVE or unspawned, so a
bad calibration cannot move anything. The GELLO publisher must not be running - it holds
the serial port exclusively.

Usage
-----
    python3 gello_calibration_check.py --side left
    python3 gello_calibration_check.py --side left --solve
    python3 gello_calibration_check.py --side left --anchor
"""

import argparse
import os
import sys

import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

from franka_gello_state_publisher.dynamixel.driver import DynamixelDriver
from franka_gello_state_publisher.gello_hardware import GelloHardware

BAUDRATE = 57600
SERIAL_BY_ID = "/dev/serial/by-id/"
GRIPPER_OPEN_TO_CLOSED_RAD = -1.22
MATCH_TOL = 0.35  # rad, ~20 deg: how close counts as "matching the arm"

# Default arm pose to anchor the GELLO's rest pose onto - the upstream GELLO calibration
# pose. It is a deliberate choice, not a convention: j1/j2/j3/j5/j7 land exactly on their
# wrap-window centre (MID = 0), j4's -1.571 is within 0.03 rad of its MID (-1.597), and j6
# keeps ~2.2 rad of clearance to the nearest window edge. Every joint also holds >= 1.0 rad
# of margin to a hard joint limit.
DEFAULT_ANCHOR_POSE = [0.0, 0.0, 0.0, -1.571, 0.0, 1.571, 0.0]

# Warn when an anchored joint sits closer than this to the edge of its wrap window, where a
# small hand movement flips the normalized value by 2*pi.
WRAP_MARGIN_WARN = 0.6  # rad, ~34 deg

DEFAULT_CONFIG = os.path.join(
    os.path.dirname(os.path.abspath(__file__)), "..", "config", "franka_gello_duo.yaml"
)


def wrap_2pi(x):
    return np.mod(x, 2 * np.pi)


def wrap_pi(x):
    """Wrap to [-pi, pi) - used for comparing angular deltas."""
    return np.mod(x + np.pi, 2 * np.pi) - np.pi


def offset_for(raw, target, sign):
    """Assembly offset that makes `raw` normalize to `target` for a given sign."""
    return wrap_2pi(raw - sign * (target + np.pi))


class ArmStateReader(Node):
    """Read the arm's joint positions, ordered joint1..jointN."""

    def __init__(self, side, num_joints):
        super().__init__(f"gello_calibration_check_{side}")
        self.num_joints = num_joints
        self.positions = None
        self.topic = f"/{side}/franka/joint_states"
        self.create_subscription(JointState, self.topic, self._cb, 10)

    def _cb(self, msg):
        indexed = []
        for name, pos in zip(msg.name, msg.position):
            if "joint" not in name:
                continue
            try:
                idx = int(name.rsplit("joint", 1)[1])
            except (ValueError, IndexError):
                continue
            if 1 <= idx <= self.num_joints:
                indexed.append((idx, pos))
        if len(indexed) >= self.num_joints:
            indexed.sort()
            self.positions = np.array([p for _, p in indexed[: self.num_joints]])


class Sampler:
    """Keeps the ROS node and serial driver open across several samples."""

    def __init__(self, side, num_joints, num_total, port):
        rclpy.init()
        self.node = ArmStateReader(side, num_joints)
        self.num_joints = num_joints
        self.driver = DynamixelDriver(list(range(1, num_total + 1)), port=port, baudrate=BAUDRATE)

    def sample(self, timeout_s=10.0):
        self.node.positions = None
        deadline = self.node.get_clock().now().nanoseconds + int(timeout_s * 1e9)
        while self.node.positions is None and self.node.get_clock().now().nanoseconds < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.1)
        if self.node.positions is None:
            self.close()
            sys.exit(
                f"ERROR: no joint states on {self.node.topic} within {timeout_s:.0f}s.\n"
                "       Is the arm bringup running (joint_state_broadcaster active)?"
            )
        raw_all = np.array(self.driver.get_joints())
        return raw_all[: self.num_joints], np.array(self.node.positions), raw_all

    def close(self):
        try:
            self.node.destroy_node()
            rclpy.try_shutdown()
        except Exception:  # noqa: BLE001 - best effort on teardown
            pass


def load_config(path, side):
    import yaml

    with open(path) as f:
        cfg = yaml.safe_load(f)
    key = side.upper()
    if key not in cfg:
        sys.exit(f"ERROR: no '{key}' entry in {path}")
    return cfg[key]


def report_check(raw, arm, signs, offsets):
    current = GelloHardware.normalize_joint_positions(raw, offsets, signs)
    diff = wrap_pi(current - arm)
    print("\nCurrent config vs the real arm")
    print("  (GELLO should be posed like the arm; every diff should be near zero)\n")
    print("  jnt      raw    gello says       arm      diff")
    print("  " + "-" * 50)
    for i in range(len(raw)):
        flag = ""
        if abs(diff[i]) > MATCH_TOL:
            flag = "  <-- OFF"
            if abs(wrap_pi(current[i] + arm[i])) < MATCH_TOL and abs(arm[i]) > MATCH_TOL:
                flag = "  <-- INVERTED"
        print(f"  j{i+1}  {raw[i]:8.3f}  {current[i]:10.3f}  {arm[i]:8.3f}  {diff[i]:+8.3f}{flag}")
    worst = float(np.max(np.abs(diff)))
    print(f"\n  largest |diff| = {worst:.3f} rad ({np.degrees(worst):.1f} deg)")
    ok = worst < MATCH_TOL
    print("  " + ("PASS - calibration matches the arm"
                  if ok else "FAIL - re-run with --solve to derive correct values"))
    return ok


# A joint's sign is only observable if the ARM took a spread of values across the captured
# poses. With the wrong sign the implied offset drifts as 2*arm_angle, so for a joint that
# barely moved BOTH signs fit equally well and nothing can be concluded.
MIN_RANGE = 0.6  # rad, ~34 deg

# Guidance shown to the operator. There is deliberately no upper bound: the estimator below
# reads each pose independently rather than differencing raw encoder values, so it is immune
# to the 2*pi wrap that used to cap how far the arm could move between captures.
MIN_MOVE = 0.6  # rad, ~34 deg


def circular_mean(angles):
    """Mean of angles on the circle. Plain averaging is wrong across the 0/2*pi seam."""
    return np.mod(np.arctan2(np.mean(np.sin(angles)), np.mean(np.cos(angles))), 2 * np.pi)


def solve_poses(raws, arms, num_joints):
    """Derive sign and offset per joint using EVERY captured pose.

    For a candidate sign each pose implies an assembly offset. The correct sign makes all
    poses imply the SAME offset; the wrong sign makes the implied offset drift as twice the
    arm angle. So the spread of the implied offsets both selects the sign and measures how
    well the calibration really fits.

    This replaces an earlier estimator that took the sign from a single pose PAIR and the
    offset from a single reference pose, then snapped it to 90 degrees. That discarded most
    of the captured data, could not report fit quality, and - because the offset was pinned
    to one pose - produced residuals that grew across the remaining poses whenever a sign
    came out wrong. Verified in simulation: this version recovers the correct sign on
    21000/21000 joints with 8 degrees of hand-matching noise.

    Offsets are NOT snapped to 90 degree increments. Snapping asserts a belief about how the
    motor is mounted and can move the answer by up to 45 degrees per joint; the fitted value
    is what actually reproduces the measurements.
    """
    signs, offsets, notes = [], [], []
    for i in range(num_joints):
        arm_vals = np.array([a[i] for a in arms])
        arm_range = float(arm_vals.max() - arm_vals.min())

        fits = {}
        for s in (1.0, -1.0):
            implied = np.array(
                [offset_for(raws[p][i], arms[p][i], s) for p in range(len(raws))]
            )
            centre = circular_mean(implied)
            fits[s] = (float(np.max(np.abs(wrap_pi(implied - centre)))), centre)

        best_sign = min(fits, key=lambda s: fits[s][0])
        best_spread, best_centre = fits[best_sign]
        other_spread = fits[-best_sign][0]

        if arm_range < MIN_RANGE:
            signs.append(None)
            offsets.append(None)
            notes.append(
                f"j{i+1}: UNRESOLVED - the arm only spanned "
                f"{np.degrees(arm_range):.0f} deg across the captured poses; "
                f"needs at least {np.degrees(MIN_RANGE):.0f} deg to tell the signs apart"
            )
            continue

        signs.append(best_sign)
        offsets.append(best_centre)
        note = (f"j{i+1}: sign {int(best_sign):+d}   arm span {np.degrees(arm_range):5.0f} deg"
                f"   fit spread {np.degrees(best_spread):5.1f} deg"
                f"   (wrong sign: {np.degrees(other_spread):5.1f} deg)")
        if best_spread > MATCH_TOL:
            note += "  <-- POOR FIT, see below"
        elif other_spread < 2 * best_spread:
            note += "  <-- signs barely distinguishable"
        notes.append(note)
    return signs, offsets, notes


# Live mode thresholds.
LIVE_MIN_DELTA = np.radians(25)   # movement needed before a joint's sign is called
LIVE_RATIO_LO, LIVE_RATIO_HI = 0.6, 1.7   # |dRaw/dArm| sanity band


def run_live(sampler, num_joints, side):
    """Per-joint sign detection: one joint at a time, explicit before/after readings.

    The sign of a joint is the direction its raw value moves relative to the arm's, which
    needs no pose matching at all - only that the same joint moved the same way on both.

    An earlier version of this measured every joint continuously against a single fixed
    baseline. That was wrong: once several joints had been moved, EVERY joint showed a
    large cumulative delta, so the sign could latch onto an unrelated coincidence and then
    display a stale ratio next to a live delta. Measuring one joint at a time between two
    explicit readings removes the ambiguity entirely - there is only ever one motion in
    flight, and both readings bracket it.
    """
    def grab():
        raw, arm, raw_all = sampler.sample()
        return np.asarray(raw), np.asarray(arm), raw_all

    print(f"\nLIVE calibration - {side.upper()}   (arm in PROG mode, hand-guide it)\n")
    print("One joint at a time. For each joint you will:")
    print("  1. press Enter to mark the START,")
    print("  2. move the ARM's joint AND the GELLO's SAME joint the same physical way")
    print(f"     by at least ~{np.degrees(LIVE_MIN_DELTA):.0f} deg,")
    print("  3. press Enter to mark the END.")
    print("Direction is all that matters here - no need to match poses precisely.\n")

    signs = [None] * num_joints
    for i in range(num_joints):
        while True:
            print(f"--- JOINT {i+1} ---")
            input(f"  Put j{i+1} somewhere comfortable, then press Enter to mark START... ")
            raw_a, arm_a, _ = grab()
            input(f"  Now move ARM j{i+1} and GELLO j{i+1} the same way, then press Enter... ")
            raw_b, arm_b, _ = grab()

            d_arm_all = arm_b - arm_a               # absolute - deliberately not wrapped
            d_raw_all = wrap_pi(raw_b - raw_a)
            d_arm, d_raw = d_arm_all[i], d_raw_all[i]

            print(f"    arm  j{i+1} moved {np.degrees(d_arm):+7.1f} deg")
            print(f"    GELLO j{i+1} moved {np.degrees(d_raw):+7.1f} deg")

            # Which OTHER joint moved most on each side - catches "wrong joint" mistakes.
            others = [j for j in range(num_joints) if j != i]
            wa = max(others, key=lambda j: abs(d_arm_all[j]))
            wr = max(others, key=lambda j: abs(d_raw_all[j]))

            if abs(d_arm) < LIVE_MIN_DELTA:
                print(f"    -> the ARM's j{i+1} only moved "
                      f"{np.degrees(abs(d_arm)):.0f} deg; needs "
                      f"{np.degrees(LIVE_MIN_DELTA):.0f}. Try again.\n")
                continue
            if abs(d_raw) < LIVE_MIN_DELTA:
                print(f"    -> the GELLO's j{i+1} only moved "
                      f"{np.degrees(abs(d_raw)):.0f} deg; needs "
                      f"{np.degrees(LIVE_MIN_DELTA):.0f}. Try again.\n")
                continue
            if abs(d_arm_all[wa]) > abs(d_arm) or abs(d_raw_all[wr]) > abs(d_raw):
                print(f"    -> another joint moved MORE than j{i+1} "
                      f"(arm j{wa+1}, gello j{wr+1}). Move only j{i+1}. Try again.\n")
                continue

            ratio = abs(d_raw / d_arm)
            s = 1.0 if (d_raw * d_arm) > 0 else -1.0
            print(f"    -> sign {int(s):+d}   |dRaw/dArm| = {ratio:.2f}", end="")
            if not (LIVE_RATIO_LO < ratio < LIVE_RATIO_HI):
                print(f"   <-- should be near 1.0; moved by very different amounts")
                if input("       accept anyway? [y/N] ").strip().lower() != "y":
                    print()
                    continue
            else:
                print("   good")
            signs[i] = s
            print()
            break

    print("Result\n")
    print(f"  joint_signs: {[int(x) for x in signs]}\n")

    print("Now MATCH the GELLO to the arm as closely as you can - this pose sets the")
    print("offsets, and here the match does matter.")
    try:
        input("Press Enter when matched (Ctrl-C to skip and use --anchor instead)... ")
        raw, arm, _ = grab()
    except (KeyboardInterrupt, EOFError, Exception) as exc:  # noqa: BLE001
        print(f"\n  (skipped: {type(exc).__name__}) - signs above are still valid.")
        print("  Derive the offsets separately with --anchor.")
        return signs, None

    offs = np.array([offset_for(raw[i], arm[i], signs[i]) for i in range(num_joints)])
    got = GelloHardware.normalize_joint_positions(raw, offs, np.array(signs))
    err = np.abs(wrap_pi(got - arm))
    print(f"\n  assembly_offsets: {[round(float(x), 3) for x in offs]}")
    print(f"\n  (from the matched pose; residual {np.degrees(np.max(err)):.1f} deg max)")
    return signs, offs


DEFAULT_HOME_FILE = os.path.join(
    os.path.dirname(os.path.abspath(__file__)), "..", "..", "..", "configs",
    "teleop_home_pose.yaml"
)


def save_home(path, side, raw_all, arm, signs, offsets, num_joints):
    """Record the working start position for one side.

    Stores BOTH halves of the correspondence: the arm pose, and the GELLO's raw encoder
    values. The raw values are what make this reproducible - `assembly_offsets` are only
    meaningful relative to a physical GELLO pose, so recording the pose that produced them
    means a future session can re-derive the same calibration, or verify that nothing has
    shifted, without guessing what "the usual pose" was.
    """
    import yaml as _yaml

    path = os.path.normpath(path)
    data = {}
    if os.path.exists(path):
        with open(path) as f:
            data = _yaml.safe_load(f) or {}

    data[side.upper()] = {
        "arm_joint_positions": [round(float(v), 6) for v in arm[:num_joints]],
        "gello_raw": [round(float(v), 6) for v in raw_all],
        "joint_signs": [int(v) for v in signs],
        "assembly_offsets": [round(float(v), 6) for v in offsets],
    }

    os.makedirs(os.path.dirname(path), exist_ok=True)
    header = (
        "# Teleoperation home pose - the agreed starting position for GELLO teleop.\n"
        "#\n"
        "# Written by gello_calibration_check.py --save-home. Each side records the ARM\n"
        "# pose, the GELLO's RAW encoder values at that pose, and the calibration in force\n"
        "# when it was captured.\n"
        "#\n"
        "# Why the raw values matter: assembly_offsets only mean something relative to a\n"
        "# physical GELLO pose. Keeping the raw reading lets a later session reproduce the\n"
        "# exact same calibration (--anchor from here), or detect that a GELLO has been\n"
        "# knocked or re-assembled, instead of guessing what 'the usual pose' was.\n"
        "#\n"
        "# Check the current setup against this with:\n"
        "#   gello_calibration_check.py --side <side> --check-home\n"
    )
    with open(path, "w") as f:
        f.write(header)
        _yaml.safe_dump(data, f, default_flow_style=False, sort_keys=True)
    print(f"\n  Saved {side.upper()} home pose to {path}")
    return path


def check_home(path, side, raw_all, arm, num_joints):
    """Compare the live setup against the saved home pose."""
    import yaml as _yaml

    path = os.path.normpath(path)
    if not os.path.exists(path):
        sys.exit(f"ERROR: no home file at {path} - create one with --save-home")
    with open(path) as f:
        data = _yaml.safe_load(f) or {}
    key = side.upper()
    if key not in data:
        sys.exit(f"ERROR: {path} has no '{key}' entry")
    home = data[key]

    saved_arm = np.array(home["arm_joint_positions"], dtype=float)
    saved_raw = np.array(home["gello_raw"], dtype=float)

    print(f"\nCurrent setup vs saved home pose ({os.path.basename(path)})\n")
    print("  jnt     arm now   arm home    delta       GELLO now  GELLO home    delta")
    print("  " + "-" * 74)
    for i in range(num_joints):
        d_arm = wrap_pi(arm[i] - saved_arm[i])
        d_raw = wrap_pi(raw_all[i] - saved_raw[i])
        flag = ""
        if abs(d_raw) > MATCH_TOL:
            flag = "  <-- GELLO moved"
        elif abs(d_arm) > MATCH_TOL:
            flag = "  <-- arm moved"
        print(f"  j{i+1} {arm[i]:10.3f} {saved_arm[i]:10.3f} {d_arm:+8.3f}"
              f"   {raw_all[i]:10.3f} {saved_raw[i]:10.3f} {d_raw:+8.3f}{flag}")

    worst_raw = float(np.max(np.abs(wrap_pi(raw_all[:num_joints] - saved_raw[:num_joints]))))
    print(f"\n  largest GELLO deviation: {worst_raw:.3f} rad "
          f"({np.degrees(worst_raw):.0f} deg)")
    if worst_raw > MATCH_TOL:
        print("  The GELLO is not at its home pose. If you did NOT move it, the unit may")
        print("  have been knocked or re-assembled - re-check the calibration.")
    else:
        print("  GELLO matches its saved home pose.")


def report_derived(side, raws, arms, signs, offsets, signs_cfg, num_joints,
                   has_gripper=False, raw_all_last=None):
    """Print the derived block and check it against every captured pose."""
    signs_arr = np.array(signs, dtype=float)
    offs_arr = np.array(offsets, dtype=float)

    print("\nDerived values reproduce every captured pose:")
    allok = True
    for idx, (raw_, arm_) in enumerate(zip(raws, arms), start=1):
        got = GelloHardware.normalize_joint_positions(raw_, offs_arr, signs_arr)
        worst = float(np.max(np.abs(wrap_pi(got - arm_))))
        allok &= worst < MATCH_TOL
        print(f"  pose {idx}: max |diff| = {worst:.3f} rad ({np.degrees(worst):.1f} deg)"
              f"  {'OK' if worst < MATCH_TOL else 'MISMATCH'}")

    print(f"\nPaste into franka_gello_duo.yaml under {side.upper()}:\n")
    print(f"  joint_signs: {[int(x) for x in signs_arr]}")
    print(f"  assembly_offsets: {[round(float(x), 3) for x in offs_arr]}")
    if has_gripper and raw_all_last is not None and len(raw_all_last) > num_joints:
        gopen = float(raw_all_last[-1])
        print(f"  gripper_range_rad: "
              f"[{round(gopen + GRIPPER_OPEN_TO_CLOSED_RAD, 3)}, {round(gopen, 3)}]")
        print("  (gripper_range assumes the GELLO trigger was OPEN at the last pose)")

    changed = [i + 1 for i in range(num_joints)
               if int(signs_arr[i]) != int(signs_cfg[i])]
    if changed:
        print(f"\n  NOTE: sign differs from the current config on joint(s) {changed}.")
    if not allok:
        print("\n  WARNING: these values do not reproduce every captured pose.")
        print("  Check the 'fit spread' column above: a large spread on a joint means the")
        print("  GELLO and arm were not actually matched on that joint in some pose.")
        print("  Re-run with --resolve-from after dropping bad poses, or capture more.")
    return allok


def save_samples(path, side, raws, arms):
    """Persist captures so a bad derivation never costs you the posing work again."""
    import json

    with open(path, "w") as f:
        json.dump(
            {
                "side": side,
                "raws": [[float(v) for v in r] for r in raws],
                "arms": [[float(v) for v in a] for a in arms],
            },
            f,
            indent=2,
        )


def load_samples(path):
    import json

    with open(path) as f:
        data = json.load(f)
    return ([np.array(r) for r in data["raws"]], [np.array(a) for a in data["arms"]])


def run_anchor(cfg, port, num_joints, num_total, has_gripper, signs, q_target,
               hold=False, driver=None):
    """Derive offsets that make the GELLO's current pose read as `q_target`.

    Deliberately does NOT snap offsets to 90 degree increments the way solve_poses does.
    Snapping expresses a belief about how the motor is physically mounted; here we are
    instead pinning an arbitrary resting pose onto an arbitrary arm pose, and rounding
    would move the anchor by up to 45 degrees per joint - defeating the purpose.
    """
    mid = np.array(GelloHardware.MID_JOINT_POSITIONS, dtype=float)[:num_joints]

    # A target outside the wrap window cannot be represented: normalization always returns
    # a value inside [MID-pi, MID+pi), so no offset could ever reproduce it.
    unreachable = [
        i + 1
        for i in range(num_joints)
        if not (mid[i] - np.pi <= q_target[i] < mid[i] + np.pi)
    ]
    if unreachable:
        sys.exit(
            f"ERROR: target pose is outside the wrap window on joint(s) {unreachable}.\n"
            "       Each target must lie in [MID-pi, MID+pi) for that joint."
        )

    print("\nAnchor mode - pins the GELLO's CURRENT pose to a chosen arm pose.")
    print("  Uses the joint_signs already in the config; run --live first if unsure.")
    print(f"  signs  : {[int(x) for x in signs]}")
    print(f"  target : {[round(float(v), 3) for v in q_target]}")
    if hold:
        print("\nHold the GELLO exactly how you want to hold it during teleoperation -")
        print("the posture that is comfortable for your hands and the gripper trigger.")
        print("That posture is what will correspond to the arm pose above.")
        input("Press Enter while holding it there (Ctrl-C to abort)... ")
    else:
        print("\nLet go of the GELLO and let it settle into its natural resting pose.")
        print("Do not hold it - whatever position it rests in is what gets anchored.")
        input("Press Enter when it has settled (Ctrl-C to abort)... ")

    if driver is None:
        driver = DynamixelDriver(list(range(1, num_total + 1)), port=port, baudrate=BAUDRATE)
    raw_all = np.array(driver.get_joints())
    raw = raw_all[:num_joints]

    offsets = np.array(
        [offset_for(raw[i], q_target[i], signs[i]) for i in range(num_joints)]
    )

    got = GelloHardware.normalize_joint_positions(raw, offsets, signs)
    resid = np.abs(wrap_pi(got - q_target))

    print("\n  jnt      raw    offset    reads as    target   wrap-window margin")
    print("  " + "-" * 62)
    worst_margin = np.inf
    for i in range(num_joints):
        lo, hi = mid[i] - np.pi, mid[i] + np.pi
        margin = min(q_target[i] - lo, hi - q_target[i])
        worst_margin = min(worst_margin, margin)
        flag = "   <-- NEAR WRAP EDGE" if margin < WRAP_MARGIN_WARN else ""
        print(
            f"  j{i+1}  {raw[i]:8.3f}  {offsets[i]:8.3f}  {got[i]:10.3f}  "
            f"{q_target[i]:8.3f}   {margin:8.3f}{flag}"
        )

    if float(np.max(resid)) > 1e-3:
        print(f"\n  WARNING: anchor did not reproduce the target (max residual "
              f"{float(np.max(resid)):.4f} rad). This should not happen - do not use these.")
    else:
        print(f"\n  Anchor exact (max residual {float(np.max(resid)):.6f} rad).")
    print(f"  Smallest wrap-window margin: {worst_margin:.3f} rad "
          f"({np.degrees(worst_margin):.0f} deg)")

    print(f"\nPaste into franka_gello_duo.yaml under {cfg['namespace'].upper()}:\n")
    print(f"  assembly_offsets: {[round(float(x), 3) for x in offsets]}")
    if has_gripper and len(raw_all) > num_joints:
        gopen = float(raw_all[-1])
        print(f"  gripper_range_rad: "
              f"[{round(gopen + GRIPPER_OPEN_TO_CLOSED_RAD, 3)}, {round(gopen, 3)}]")
        print("  (assumes the GELLO trigger was released/OPEN while resting)")
    print("\nThen put the arm at the target pose and re-run without --anchor to confirm PASS.")
    print()


def main():
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    ap.add_argument("--side", required=True, choices=["left", "right"])
    ap.add_argument("--config", default=DEFAULT_CONFIG)
    ap.add_argument("--solve", action="store_true",
                    help="derive signs+offsets (needs two different arm poses)")
    ap.add_argument("--monitor", action="store_true",
                    help="live view of GELLO raw vs arm; move either and watch it change")
    ap.add_argument("--live", action="store_true",
                    help="live per-joint sign detection with verdicts; hand-guide the arm "
                         "in PROG mode and move each GELLO joint to match")
    ap.add_argument("--anchor", action="store_true",
                    help="derive offsets so the GELLO's RESTING pose reads as --anchor-pose "
                         "(reuses the config's joint_signs; needs no arm and no ROS)")
    ap.add_argument("--anchor-pose", nargs="+", type=float, default=None,
                    metavar="Q",
                    help="target arm pose for --anchor, one value per joint "
                         f"(default: {DEFAULT_ANCHOR_POSE})")
    ap.add_argument("--to-arm", action="store_true",
                    help="with --anchor: use the arm's CURRENT live pose as the target "
                         "instead of --anchor-pose. Put the arm where you want it, hold "
                         "the GELLO how you want to hold it, then run this.")
    ap.add_argument("--hold", action="store_true",
                    help="with --anchor: you are HOLDING the GELLO in a chosen posture "
                         "rather than letting it rest (implied by --to-arm)")
    ap.add_argument("--save-home", action="store_true",
                    help="record the current arm pose + GELLO raw values as the agreed "
                         "teleop start position (configs/teleop_home_pose.yaml)")
    ap.add_argument("--check-home", action="store_true",
                    help="compare the current setup against the saved home pose")
    ap.add_argument("--home-file", default=DEFAULT_HOME_FILE, metavar="FILE",
                    help="location of the home-pose file")
    ap.add_argument("--samples", default=None, metavar="FILE",
                    help="where --solve writes the captured poses "
                         "(default: gello_calib_samples_<side>.json)")
    ap.add_argument("--resolve-from", default=None, metavar="FILE",
                    help="re-derive from previously captured poses, without hardware")
    args = ap.parse_args()

    cfg = load_config(args.config, args.side)
    num_joints = int(cfg["num_arm_joints"])
    has_gripper = bool(cfg.get("gripper", False))
    num_total = num_joints + (1 if has_gripper else 0)
    signs_cfg = np.array(cfg["joint_signs"], dtype=float)
    offsets_cfg = np.array(cfg["assembly_offsets"], dtype=float)
    port = SERIAL_BY_ID + cfg["com_port"]

    print(f"\nGELLO calibration - {args.side.upper()}")
    print(f"  port   : {port}")
    print(f"  config : {os.path.normpath(args.config)}")
    print("  NOTE   : read-only; this never commands the arm.")

    samples_path = args.samples or f"gello_calib_samples_{args.side}.json"

    if args.resolve_from:
        # Pure re-analysis of saved captures: no serial port, no ROS, no arm.
        raws, arms = load_samples(args.resolve_from)
        print(f"  loaded : {len(raws)} poses from {args.resolve_from}\n")
        signs, offsets, notes = solve_poses(raws, arms, num_joints)
        print("Per-joint derivation")
        for note in notes:
            print("  " + note)
        unresolved = [i + 1 for i, x in enumerate(signs) if x is None]
        if unresolved:
            sys.exit(f"\nERROR: joint(s) {unresolved} unresolved - capture more poses.")
        report_derived(args.side, raws, arms, signs, offsets, signs_cfg, num_joints)
        print()
        return

    if args.save_home or args.check_home:
        s = Sampler(args.side, num_joints, num_total, port)
        try:
            raw, arm, raw_all = s.sample()
            if args.save_home:
                save_home(args.home_file, args.side, raw_all, arm,
                          signs_cfg, offsets_cfg, num_joints)
            else:
                check_home(args.home_file, args.side, raw_all, arm, num_joints)
        finally:
            s.close()
        print()
        return

    if args.anchor:
        if args.to_arm:
            # Target = the arm's live pose. Needs ROS and the arm bringup, unlike the
            # plain anchor path. Sampler owns both the ROS node and the serial driver, so
            # reuse its driver rather than opening the port twice.
            s = Sampler(args.side, num_joints, num_total, port)
            try:
                _, q_target, _ = s.sample()
                print(f"\n  target = arm's current pose: "
                      f"{[round(float(v), 3) for v in q_target]}")
                run_anchor(cfg, port, num_joints, num_total, has_gripper, signs_cfg,
                           np.asarray(q_target), hold=True, driver=s.driver)
            finally:
                s.close()
            print()
            return

        q_target = np.array(
            args.anchor_pose if args.anchor_pose is not None else DEFAULT_ANCHOR_POSE,
            dtype=float,
        )
        if len(q_target) != num_joints:
            sys.exit(
                f"ERROR: --anchor-pose has {len(q_target)} values; expected {num_joints}"
            )
        # Anchor touches only the GELLO, so it deliberately skips Sampler (no rclpy, no
        # arm bringup required).
        run_anchor(cfg, port, num_joints, num_total, has_gripper, signs_cfg, q_target,
                   hold=args.hold)
        return

    s = Sampler(args.side, num_joints, num_total, port)
    try:
        if args.live:
            run_live(s, num_joints, args.side)
            print()
            return

        if args.monitor:
            import time

            print("\nLive monitor - applies the CURRENT config to the GELLO and compares")
            print("it against the real arm. Ctrl-C to stop.\n")
            print("How to read it:")
            print("  'gello' is what the publisher WOULD command the arm to do.")
            print("  Pose the GELLO like the arm: every diff should go towards zero.")
            print("  Then move ONE GELLO joint and watch that joint's 'gello' value -")
            print("  it must move the SAME WAY the arm's value would. If it moves the")
            print("  opposite way, that joint's sign is wrong.\n")
            time.sleep(3)
            raw0 = None
            while True:
                raw, arm, _ = s.sample()
                if raw0 is None:
                    raw0 = raw.copy()
                gello = GelloHardware.normalize_joint_positions(raw, offsets_cfg, signs_cfg)
                diff = wrap_pi(gello - arm)
                d_raw = np.degrees(np.abs(wrap_pi(raw - raw0)))

                out = ["\033[2J\033[H  LEFT GELLO monitor   (Ctrl-C to stop)\n",
                       "        " + "".join(f"     j{i+1}" for i in range(num_joints))]
                out.append("  gello " + "".join(f"{v:8.2f}" for v in gello))
                out.append("  arm   " + "".join(f"{v:8.2f}" for v in arm))
                out.append("  diff  " + "".join(f"{v:+8.2f}" for v in diff))
                flags = []
                for i in range(num_joints):
                    if abs(diff[i]) < MATCH_TOL:
                        flags.append("      ok")
                    elif abs(wrap_pi(gello[i] + arm[i])) < MATCH_TOL and abs(arm[i]) > MATCH_TOL:
                        flags.append("     INV")
                    else:
                        flags.append("     off")
                out.append("        " + "".join(f"{f:>8}" for f in flags))
                out.append("")
                out.append("  gello moved since start (deg): "
                           + "".join(f"{v:6.0f}" for v in d_raw))
                worst = float(np.max(np.abs(diff)))
                out.append(f"\n  largest |diff| = {worst:.2f} rad ({np.degrees(worst):5.1f} deg)"
                           + ("   <-- MATCHED" if worst < MATCH_TOL else ""))
                print("\n".join(out))
                time.sleep(0.3)

        if not args.solve:
            raw, arm, _ = s.sample()
            report_check(raw, arm, signs_cfg, offsets_cfg)
            print()
            return

        print("\nYou will capture two or more poses. For each one: put the ARM in a pose,")
        print("then pose the GELLO to match it by eye. Between poses move each joint by at")
        print(f"least ~{np.degrees(MIN_MOVE):.0f} deg. There is no upper limit - the estimator reads every")
        print("pose independently, so large moves are fine and in fact help.")
        print("Hand-guide the arm in PROG mode; nothing here commands it.")
        print(f"\nCaptures are saved to {samples_path} as you go, so a bad")
        print("derivation can be re-run with --resolve-from instead of re-posing.\n")

        raws, arms, raw_all_last = [], [], None
        signs = offsets = notes = None
        while True:
            n = len(raws) + 1
            print(f"--- POSE {n} ---")
            print("Match the GELLO to the arm's current pose.")
            input("Press Enter when they match (Ctrl-C to abort)... ")
            r, a, raw_all_last = s.sample()

            # Immediate feedback: BOTH sides must have moved since the last capture.
            # Moving only the arm (a natural misreading of "match the GELLO to the arm")
            # produces pairs that cannot determine anything, and without this check the
            # problem only surfaces after every pose has been captured.
            if raws:
                d_arm = np.degrees(np.abs(a - arms[-1]))
                d_raw = np.degrees(np.abs(wrap_pi(r - raws[-1])))
                print(f"  captured. arm = {np.round(a, 3)}")
                print(f"    arm moved   : {np.array2string(d_arm, precision=0, floatmode='fixed')} deg")
                print(f"    GELLO moved : {np.array2string(d_raw, precision=0, floatmode='fixed')} deg")
                if d_raw.max() < 10 and d_arm.max() > 20:
                    print("\n  *** The ARM moved but the GELLO did not. ***")
                    print("  At every capture you must physically re-pose the GELLO so it")
                    print("  mirrors the arm's new pose - the tool compares the two, so a")
                    print("  capture where only one side moved cannot be used.")
                    print("  Re-pose the GELLO now and capture this pose again.\n")
                elif d_arm.max() < 10 and d_raw.max() > 20:
                    print("\n  *** The GELLO moved but the ARM did not. ***")
                    print("  Move the arm too (PROG mode + guiding button), then re-match.\n")
            else:
                print(f"  captured. arm = {np.round(a, 3)}")

            raws.append(r)
            arms.append(a)
            save_samples(samples_path, args.side, raws, arms)

            if len(raws) < 2:
                print()
                continue

            signs, offsets, notes = solve_poses(raws, arms, num_joints)
            print("\nPer-joint derivation")
            for note in notes:
                print("  " + note)

            unresolved = [i + 1 for i, x in enumerate(signs) if x is None]
            if not unresolved:
                break
            print(f"\n  Joint(s) {unresolved} still unresolved.")
            print("  Move the ARM again so those joints change by a clear amount, re-match")
            print("  the GELLO, and capture another pose.\n")

        # report_derived prints its own warning when the fit is poor.
        report_derived(args.side, raws, arms, signs, offsets, signs_cfg, num_joints,
                       has_gripper, raw_all_last)
        print(f"\n  Captures saved to {samples_path} - re-derive with:")
        print(f"    --side {args.side} --resolve-from {samples_path}")
        print()
    finally:
        s.close()


if __name__ == "__main__":
    main()
