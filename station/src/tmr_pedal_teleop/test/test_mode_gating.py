"""base_bridge must stop the base on a mode switch WITHOUT stopping the command stream.

This is the safety property of the whole DRIVE/RECORD split. The swerve controller ages
each command against its own clock and zeroes the base if nothing arrives within
``cmd_vel_timeout`` (0.5 s), so the correct way to hold the base still is to keep
publishing a zero twist at 20 Hz - not to stop publishing. An early `return` in
``_publish`` would look correct and would still stop the base, but only after a 0.5 s
watchdog lapse, and it would break the "base is being commanded" liveness signal.
"""

import os
import sys

import ros_stubs

ros_stubs.install()

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from tmr_pedal_teleop import base_bridge  # noqa: E402
from tmr_pedal_teleop.mode_manager import DRIVE, RECORD  # noqa: E402


def _make():
    node = base_bridge.BaseBridge()
    return (
        node,
        node.timers[0],
        node.subs["/pedal/state"],
        node.subs["/teleop/pedal_mode"],
    )


def _vec(msg):
    return (msg.twist.linear.x, msg.twist.linear.y, msg.twist.angular.z)


def _last(node):
    return node.pubs["/swerve_drive_controller/cmd_vel"].msgs[-1]


def _count(node):
    return len(node.pubs["/swerve_drive_controller/cmd_vel"].msgs)


def test_drive_mode_moves_the_base():
    node, tick, pedal, _ = _make()
    pedal(ros_stubs.String("1A"))
    tick()
    assert _vec(_last(node)) == (0.05, 0.0, 0.0)


def test_record_mode_zeroes_the_base_but_keeps_publishing():
    node, tick, pedal, mode = _make()
    pedal(ros_stubs.String("1A"))
    tick()
    before = _count(node)

    mode(ros_stubs.String(RECORD))
    tick()

    assert _vec(_last(node)) == (0.0, 0.0, 0.0), "base must stop"
    assert _count(node) == before + 1, "the watchdog must keep being fed"


def test_every_pedal_is_inert_in_record_mode():
    node, tick, pedal, mode = _make()
    mode(ros_stubs.String(RECORD))
    before = _count(node)

    for token in ("1A", "1B", "1C", "2A", "2B", "2C", "1A+2C"):
        pedal(ros_stubs.String(token))
        tick()

    published = node.pubs["/swerve_drive_controller/cmd_vel"].msgs[before:]
    assert published, "stream must continue"
    assert all(_vec(m) == (0.0, 0.0, 0.0) for m in published)


def test_switching_back_to_drive_restores_motion():
    node, tick, pedal, mode = _make()
    mode(ros_stubs.String(RECORD))
    pedal(ros_stubs.String("2A"))
    tick()
    assert _vec(_last(node)) == (0.0, 0.0, 0.0)

    mode(ros_stubs.String(DRIVE))
    pedal(ros_stubs.String("2A"))
    tick()
    assert _vec(_last(node)) == (-0.05, 0.0, 0.0)


def test_stale_pedal_publisher_still_stops_the_base():
    """The pre-existing watchdog must survive the mode change."""
    node, tick, pedal, _ = _make()
    pedal(ros_stubs.String("1A"))
    tick()
    assert _vec(_last(node)) == (0.05, 0.0, 0.0)

    node._clock.t += 5.0  # far beyond pedal_timeout
    tick()
    assert _vec(_last(node)) == (0.0, 0.0, 0.0)


def test_spine_combo_still_suppresses_base_motion():
    node, tick, pedal, _ = _make()
    pedal(ros_stubs.String("1A+2C"))  # the spine "up" combo
    tick()
    assert _vec(_last(node)) == (0.0, 0.0, 0.0)


def test_defaults_to_drive_without_a_mode_manager():
    """A bare `ros2 run base_bridge` must still drive."""
    node, tick, pedal, _ = _make()
    assert node.mode == DRIVE


def _joy(x=0.0, y=0.0, yaw=0.0, enabled=True, turbo=False):
    axes = [y, x, 0.0, yaw]
    buttons = [0] * 6
    buttons[5] = int(enabled)  # RB
    buttons[4] = int(turbo)  # LB
    return ros_stubs.Joy(axes, buttons)


def test_gamepad_drives_when_no_pedal_is_pressed():
    node, tick, pedal, _ = _make()
    pedal(ros_stubs.String("NONE"))
    node.subs["/teleop/gamepad/joy"](_joy(x=1.0, y=-0.5, yaw=0.5))
    tick()
    assert _vec(_last(node)) == (0.05, -0.025, 0.025)


def test_any_pedal_press_has_priority_over_gamepad():
    node, tick, pedal, _ = _make()
    node.subs["/teleop/gamepad/joy"](_joy(x=-1.0))
    pedal(ros_stubs.String("1A"))
    tick()
    assert _vec(_last(node)) == (0.05, 0.0, 0.0)


def test_record_pedal_stops_gamepad_until_released():
    node, tick, pedal, mode = _make()
    mode(ros_stubs.String(RECORD))
    node.subs["/teleop/gamepad/joy"](_joy(x=1.0))

    pedal(ros_stubs.String("1A"))
    tick()
    assert _vec(_last(node)) == (0.0, 0.0, 0.0)

    pedal(ros_stubs.String("NONE"))
    tick()
    assert _vec(_last(node)) == (0.05, 0.0, 0.0)


def test_gamepad_requires_deadman_and_fresh_messages():
    node, tick, pedal, _ = _make()
    pedal(ros_stubs.String("NONE"))
    joy = node.subs["/teleop/gamepad/joy"]

    joy(_joy(x=1.0, enabled=False))
    tick()
    assert _vec(_last(node)) == (0.0, 0.0, 0.0)

    joy(_joy(x=1.0))
    node._clock.t += 1.0
    tick()
    assert _vec(_last(node)) == (0.0, 0.0, 0.0)


def test_gamepad_turbo_uses_controller_limits():
    node, tick, pedal, _ = _make()
    pedal(ros_stubs.String("NONE"))
    node.subs["/teleop/gamepad/joy"](_joy(x=1.0, yaw=-1.0, turbo=True))
    tick()
    assert _vec(_last(node)) == (0.1, 0.0, -0.1)
