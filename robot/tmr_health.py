#!/usr/bin/env python3
"""Continuously report whether the arms and base are still operable, WITHOUT ever creating
a new DDS participant to find out.

    python3 tmr_health.py                 # run it (start_robot.bash does this for you)
    cat /tmp/tmr_health.json              # check health - costs no DDS at all
    python3 tmr_health.py --once          # one-shot report to stdout

THE PROBLEM THIS SOLVES
On this rig, checking health is what breaks health. Every `ros2` CLI call - `recover.py
--check`, `ros2 topic hz`, `ros2 control list_controllers` - creates a NEW DDS participant,
and participant creation is a 15-25 s discovery burst. Aimed at three live 1 kHz FCI loops,
that burst is enough to make one miss its deadline, and ros2_control then demotes the
hardware to `unconfigured`. Repeated polling during teleop caused exactly the "one arm or
the base suddenly stopped working" failures it was meant to detect.

The gamepad demo (franka_bringup/mobile_teleop.launch.py) never has this problem because
every participant it needs is created during launch and none is ever added afterwards.

THE FIX
This daemon is started BY the bringup, so it is one more participant created at startup -
never mid-session. It then polls the controller_managers over SERVICE calls on its own
existing participant, which costs no discovery, and writes the result to a plain file.
Reading that file is `cat`: no node, no participant, no burst. Safe during teleop.

WHAT IT WATCHES
Per unit (left arm / right arm / base): the hardware component's lifecycle state and how
many command interfaces the controller has claimed. `hardware=active` plus a non-zero
claim count is the only honest signal that commands still reach the motors - the controller
keeps reporting `active` with healthy odom long after the hardware has dropped out.
"""

import argparse
import json
import os
import sys
import tempfile
import time

import rclpy
from rclpy.node import Node
from controller_manager_msgs.srv import ListHardwareComponents, ListControllers

UNITS = [("left", "/left"), ("right", "/right"), ("base", "")]
CALL_TIMEOUT = 4.0


def spin_until(node, future, timeout):
    rclpy.spin_until_future_complete(node, future, timeout_sec=timeout)
    return future.result() if future.done() else None


class Health(Node):
    def __init__(self, path, period):
        super().__init__("tmr_health")
        self.path = path
        self.hw = {}
        self.ctl = {}
        for name, ns in UNITS:
            self.hw[name] = self.create_client(
                ListHardwareComponents, f"{ns}/controller_manager/list_hardware_components")
            self.ctl[name] = self.create_client(
                ListControllers, f"{ns}/controller_manager/list_controllers")
        self.last = {}
        self.period = period
        # No timer on purpose. tick() calls spin_until_future_complete, and doing that
        # from inside a timer callback is a re-entrant spin - rclpy raises
        # ExternalShutdownException. The poll loop lives in main() instead.

    def unit_state(self, name):
        """(hardware_state, claimed_interfaces) - both None when unreachable."""
        state, claimed = None, None
        cli = self.hw[name]
        if cli.service_is_ready():
            res = spin_until(self, cli.call_async(ListHardwareComponents.Request()), CALL_TIMEOUT)
            if res is not None:
                for c in res.component:
                    # One system component per unit; take the first that is a system.
                    state = c.state.label
                    break
        cli = self.ctl[name]
        if cli.service_is_ready():
            res = spin_until(self, cli.call_async(ListControllers.Request()), CALL_TIMEOUT)
            if res is not None:
                claimed = 0
                for c in res.controller:
                    if c.state == "active":
                        claimed += len(getattr(c, "claimed_interfaces", []) or [])
        return state, claimed

    def tick(self):
        report = {"stamp": time.time(), "units": {}}
        degraded = []
        for name, _ns in UNITS:
            state, claimed = self.unit_state(name)
            ok = (state == "active") and bool(claimed)
            report["units"][name] = {
                "hardware": state or "unreachable",
                "claimed": claimed if claimed is not None else -1,
                "ok": ok,
            }
            if not ok:
                degraded.append(name)
            # Log only transitions, so the bringup log stays readable but never misses one.
            prev = self.last.get(name)
            if prev is not None and prev != ok:
                if ok:
                    self.get_logger().info(f"{name}: RECOVERED (hardware={state}, claimed={claimed})")
                else:
                    self.get_logger().error(
                        f"{name}: LOST (hardware={state or 'unreachable'}, claimed={claimed}). "
                        f"Commands no longer reach it. Repair with: python3 ~/recover.py"
                        + (" --base" if name == "base" else ""))
            self.last[name] = ok
        report["degraded"] = degraded
        report["healthy"] = not degraded

        # Atomic write, so a reader never sees a half-written file.
        d = os.path.dirname(self.path) or "."
        fd, tmp = tempfile.mkstemp(dir=d, prefix=".tmr_health.")
        with os.fdopen(fd, "w") as f:
            json.dump(report, f, indent=2)
        os.replace(tmp, self.path)


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--path", default="/tmp/tmr_health.json")
    ap.add_argument("--period", type=float, default=5.0, help="seconds between polls")
    ap.add_argument("--once", action="store_true", help="one report to stdout, then exit")
    args = ap.parse_args()

    rclpy.init()
    node = Health(args.path, args.period)
    try:
        # Give service discovery a moment; on this network a participant needs several
        # seconds before the controller_manager services resolve.
        deadline = time.time() + 8.0
        while time.time() < deadline:
            rclpy.spin_once(node, timeout_sec=0.2)

        if args.once:
            node.tick()
            print(open(args.path).read())
        else:
            node.get_logger().info(
                f"tmr_health running; read {args.path} instead of polling with ros2 CLI.")
            while rclpy.ok():
                node.tick()
                end = time.time() + node.period
                while rclpy.ok() and time.time() < end:
                    rclpy.spin_once(node, timeout_sec=0.2)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
