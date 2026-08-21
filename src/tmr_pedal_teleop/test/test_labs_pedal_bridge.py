"""labs_pedal_bridge: pedal edge detection, debounce, mode gating and the LABS action map.

``/pedal/state`` is a 20 Hz *level* heartbeat of the currently-pressed set, not an event
stream, so everything here turns on getting rising-edge detection right. The HTTP layer is
replaced with a recorder; what is under test is which call each pedal resolves to.
"""

import os
import sys

import ros_stubs

ros_stubs.install()

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from tmr_pedal_teleop import labs_pedal_bridge as lpb  # noqa: E402
from tmr_pedal_teleop.mode_manager import DRIVE, RECORD  # noqa: E402

TASK = "b06d85e1-c053-49db-acbb-685fc64ddf8a"


class Bridge(lpb.LabsPedalBridge):
    """Worker thread and HTTP neutered; tests drive _dispatch and set state directly."""

    def _worker_loop(self):
        pass

    def _refresh_state(self):
        pass

    def _post(self, path, query=None):
        self.posted.append((path, dict(query or {})))


def _make(task_id=TASK, mode=RECORD, workflow="FOLLOWING", recording="IDLE"):
    b = Bridge()
    b.posted = []
    b.task_id = task_id
    b.mode = mode
    b.workflow_state, b.recording_state = workflow, recording
    return b


def _press(b, token, at=None):
    """Release, advance past the debounce, then press - one clean rising edge."""
    b.subs["/pedal/state"](ros_stubs.String("NONE"))
    b._clock.t = at if at is not None else b._clock.t + 1.0
    b.subs["/pedal/state"](ros_stubs.String(token))
    while not b._q.empty():
        b._dispatch(b._q.get())


def test_drive_mode_ignores_pedals():
    b = _make(mode=DRIVE)
    _press(b, "1A")
    assert b.posted == []


def test_following_start_recording_carries_the_task_id():
    b = _make()
    _press(b, "1A")
    assert b.posted == [("/api/v1/recording/start", {"task_id": TASK})]


def test_held_pedal_fires_only_once():
    b = _make()
    _press(b, "1A")
    b.posted.clear()
    for _ in range(5):
        b._clock.t += 0.05
        b.subs["/pedal/state"](ros_stubs.String("1A"))
    while not b._q.empty():
        b._dispatch(b._q.get())
    assert b.posted == []


def test_debounce_is_press_to_press():
    b = _make()
    _press(b, "1A", at=10.0)
    b.posted.clear()

    # Released and re-pressed 0.2 s after the accepted press: inside the 0.3 s window.
    b.subs["/pedal/state"](ros_stubs.String("NONE"))
    b._clock.t = 10.2
    b.subs["/pedal/state"](ros_stubs.String("1A"))
    while not b._q.empty():
        b._dispatch(b._q.get())
    assert b.posted == []

    # 1.0 s after: outside the window.
    b.subs["/pedal/state"](ros_stubs.String("NONE"))
    b._clock.t = 11.0
    b.subs["/pedal/state"](ros_stubs.String("1A"))
    while not b._q.empty():
        b._dispatch(b._q.get())
    assert len(b.posted) == 1


def test_recording_state_takes_precedence_over_workflow_state():
    """Mirrors getButtonActions() in the LABS UI."""
    b = _make(recording="RECORDING")
    _press(b, "1A")
    assert b.posted == [("/api/v1/recording/stop", {})]


def test_reviewing_pedals():
    for token, path, query in (
        ("2A", "/api/v1/recording/save", {"label": "REVIEW_SUCCESS"}),
        ("2B", "/api/v1/recording/save", {"label": "REVIEW_FAILED"}),
        ("2C", "/api/v1/recording/discard", {}),
    ):
        b = _make(recording="REVIEWING")
        _press(b, token)
        assert b.posted == [(path, query)], f"{token} -> {b.posted}"


def test_teleop_pedals():
    for workflow, token, path in (
        ("IDLE", "1B", "/api/v1/teleop/start"),
        ("READY", "1B", "/api/v1/teleop/stop"),
        ("READY", "1C", "/api/v1/teleop/start_syncing"),
        ("SYNCING", "1B", "/api/v1/teleop/stop"),
        ("FOLLOWING", "1B", "/api/v1/teleop/stop"),
    ):
        b = _make(workflow=workflow)
        _press(b, token)
        assert b.posted == [(path, {})], f"{workflow}/{token} -> {b.posted}"


def test_unbound_pedal_is_a_noop():
    b = _make(recording="REVIEWING")
    _press(b, "1C")  # only bound in READY
    assert b.posted == []


def test_start_recording_refused_without_a_task_id():
    b = _make(task_id="")
    _press(b, "1A")
    assert b.posted == []
    assert any("refused" in line for line in b._logger.lines)


def test_pedal_held_across_the_mode_switch_is_not_replayed():
    """Resting a foot on a pedal while pressing 'm' must not trigger an action."""
    b = _make(mode=DRIVE)
    b._clock.t = 1.0
    b.subs["/pedal/state"](ros_stubs.String("1A"))  # pressed while still in DRIVE
    b.subs["/teleop/pedal_mode"](ros_stubs.String(RECORD))
    b._clock.t += 0.05
    b.subs["/pedal/state"](ros_stubs.String("1A"))  # same level, still held
    while not b._q.empty():
        b._dispatch(b._q.get())
    assert b.posted == []


def test_every_mapped_action_is_implemented():
    b = _make()
    for state, bindings in b.action_map.items():
        for token, action in bindings.items():
            assert action in lpb.ACTIONS, f"map_{state}_{token} = {action!r} is not an action"
