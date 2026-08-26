#!/usr/bin/env python3
"""Activate franka_robot_state_broadcaster on BOTH arms from ONE DDS participant.

    python3 activate_broadcasters.py            # configure + activate both
    python3 activate_broadcasters.py --check    # report only

WHY THIS EXISTS
Those two controllers publish 6 of the 28 topics a LeRobot episode needs - 20 of the 62
state dimensions. Teleop does not care (joint_impedance_controller reads ros2_control
state interfaces directly), so nothing complains, and a recording made without them is
silently unconvertible: LABS' TemporalSynchronizer fails the WHOLE episode.

They end up inactive two ways:
  * the bringup spawner times out between 'Configuring' and 'Activating' and dies exit 1
  * recover.py cycles the arm hardware and only re-activates joint_impedance_controller

ONE PARTICIPANT, DELIBERATELY
Two `ros2 service call` invocations would be two new DDS participants, and participant
creation is a 15-25 s discovery burst on this network - enough to make a 1 kHz FCI loop
miss its deadline and fault the hardware. That is how piecemeal repairs cascade here.

SAFE WHILE GELLO IS LIVE: a broadcaster claims only STATE interfaces, never command
interfaces, so it cannot move the arm.
"""

import argparse
import sys

import rclpy
from rclpy.node import Node
from controller_manager_msgs.srv import ListControllers, SwitchController, ConfigureController

NAME = "franka_robot_state_broadcaster"
SIDES = ("left", "right")
TIMEOUT = 15.0


def spin_until(node, future, timeout):
    rclpy.spin_until_future_complete(node, future, timeout_sec=timeout)
    return future.result() if future.done() else None


def state_of(node, side):
    cli = node.create_client(ListControllers, f"/{side}/controller_manager/list_controllers")
    if not cli.wait_for_service(timeout_sec=TIMEOUT):
        return None
    res = spin_until(node, cli.call_async(ListControllers.Request()), TIMEOUT)
    if res is None:
        return None
    for c in res.controller:
        if c.name == NAME:
            return c.state
    return "not-loaded"


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--check", action="store_true", help="report only, change nothing")
    args = ap.parse_args()

    rclpy.init()
    node = Node("activate_broadcasters")
    log = node.get_logger()
    rc = 0

    for side in SIDES:
        st = state_of(node, side)
        if st is None:
            log.warn(f"{side}: controller_manager not reachable; skipping")
            rc = 1
            continue
        log.info(f"{side}: {NAME} is '{st}'")
        if st == "active" or args.check:
            continue

        if st in ("unconfigured", "not-loaded"):
            cfg = node.create_client(ConfigureController, f"/{side}/controller_manager/configure_controller")
            if cfg.wait_for_service(timeout_sec=TIMEOUT):
                req = ConfigureController.Request()
                req.name = NAME
                spin_until(node, cfg.call_async(req), TIMEOUT)

        sw = node.create_client(SwitchController, f"/{side}/controller_manager/switch_controller")
        if not sw.wait_for_service(timeout_sec=TIMEOUT):
            log.error(f"{side}: switch_controller unavailable")
            rc = 1
            continue
        req = SwitchController.Request()
        req.activate_controllers = [NAME]
        req.strictness = SwitchController.Request.BEST_EFFORT
        spin_until(node, sw.call_async(req), TIMEOUT)

        st = state_of(node, side)
        if st == "active":
            log.info(f"{side}: {NAME} ACTIVE")
        else:
            log.error(f"{side}: {NAME} is '{st}' - its 3 topics will record nothing")
            rc = 1

    node.destroy_node()
    rclpy.try_shutdown()
    return rc


if __name__ == "__main__":
    sys.exit(main())
