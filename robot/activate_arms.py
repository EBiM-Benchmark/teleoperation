#!/usr/bin/env python3
"""Activate joint_impedance_controller on BOTH arms from a SINGLE ROS node.

WHY THIS EXISTS

Running `ros2 control set_controller_state ...` twice kills the first arm. Each CLI call
creates a brand new DDS participant, and on this robot participant discovery is a 15-25 s
exchange across a link already carrying three 1 kHz Franka FCI streams. That burst arrives
while the first arm is in Move mode and trips its reflex:

    457671.90  LEFT impedance activated
    457674.66  LEFT libfranka reflex ["communication_constraints_violation"]
    457675.19  RIGHT impedance activated

ros2_control then demotes left_FrankaHardwareInterface to `unconfigured` while the
controller still reports `active`, so GELLO commands silently go nowhere.

This script creates ONE participant, waits for BOTH services to be discovered BEFORE
activating anything, and only then switches the controllers. All discovery traffic is done
while both arms are still idle.

USAGE
    python3 ~/activate_arms.py               # activate both
    python3 ~/activate_arms.py --deactivate  # deactivate both (e.g. before re-homing)

Hands OFF the GELLOs while this runs: the controller captures the GELLO and arm poses in
the same instant, and any movement between activation and the first update becomes an
approach target.
"""
import sys
import rclpy
from rclpy.node import Node
from controller_manager_msgs.srv import (
    ConfigureController,
    ListControllers,
    SwitchController,
)

CONTROLLER = "joint_impedance_controller"
SIDES = ("left", "right")
DISCOVERY_TIMEOUT = 15.0
CALL_TIMEOUT = 10.0


def controller_state(node, cli):
    """Current state of CONTROLLER on that side, or None if it cannot be read."""
    fut = cli.call_async(ListControllers.Request())
    rclpy.spin_until_future_complete(node, fut, timeout_sec=CALL_TIMEOUT)
    res = fut.result()
    if res is None:
        return None
    for c in res.controller:
        if c.name == CONTROLLER:
            return c.state
    return None


def configure(node, cli, side):
    """unconfigured -> inactive. Claims no command interfaces and moves nothing."""
    req = ConfigureController.Request()
    req.name = CONTROLLER
    fut = cli.call_async(req)
    rclpy.spin_until_future_complete(node, fut, timeout_sec=CALL_TIMEOUT)
    res = fut.result()
    ok = bool(res and res.ok)
    if ok:
        node.get_logger().info(f"{side}: {CONTROLLER} configured (unconfigured -> inactive)")
    else:
        node.get_logger().error(f"{side}: configure_controller returned ok=False")
    return ok


def main() -> int:
    deactivate = "--deactivate" in sys.argv[1:]
    rclpy.init()
    node: Node = rclpy.create_node("activate_arms")

    clients = {s: node.create_client(SwitchController,
                                     f"/{s}/controller_manager/switch_controller")
               for s in SIDES}
    # list_/configure_ clients live on this SAME node, so they cost no extra participant.
    # They exist because switch_controller alone cannot rescue a controller that is
    # `unconfigured`: activation is inactive -> active, and from unconfigured the manager
    # answers "is in state 'unconfigured' (1), expected 'active' (3). Releasing
    # interfaces!" and returns ok=False. That happened on 2026-08-27 and left both arms
    # ACTIVE BUT DRIVING NOTHING (claimed=0), with recover.py unable to help either - it
    # only ever said "Restart start_robot.bash". start_robot.bash already knows the fix
    # (wait_for_controller, "spawner race"); it just was not reachable from here.
    list_clients = {s: node.create_client(ListControllers,
                                          f"/{s}/controller_manager/list_controllers")
                    for s in SIDES}
    cfg_clients = {s: node.create_client(ConfigureController,
                                         f"/{s}/controller_manager/configure_controller")
                   for s in SIDES}

    # Discover EVERYTHING first. Doing this up front is the whole point: no new DDS
    # discovery may happen once an arm is in Move mode.
    for side in SIDES:
        for label, cli in (("switch_controller", clients[side]),
                           ("list_controllers", list_clients[side]),
                           ("configure_controller", cfg_clients[side])):
            node.get_logger().info(f"waiting for /{side}/controller_manager/{label} ...")
            if not cli.wait_for_service(timeout_sec=DISCOVERY_TIMEOUT):
                node.get_logger().error(
                    f"/{side}/controller_manager/{label} not available after "
                    f"{DISCOVERY_TIMEOUT:.0f}s - is the arm stack up?")
                node.destroy_node()
                rclpy.shutdown()
                return 1

    verb = "deactivating" if deactivate else "activating"
    node.get_logger().info(f"all services discovered; {verb} now")

    # Configure anything left unconfigured BEFORE the first switch, while no arm is yet
    # in Move mode. Skipped when deactivating: an unconfigured controller is already
    # driving nothing, so there is nothing to stand down.
    if not deactivate:
        for side in SIDES:
            state = controller_state(node, list_clients[side])
            if state == "unconfigured":
                node.get_logger().warn(
                    f"{side}: {CONTROLLER} is 'unconfigured' (spawner race); configuring it.")
                configure(node, cfg_clients[side], side)
            elif state is None:
                node.get_logger().warn(
                    f"{side}: could not read {CONTROLLER} state; attempting the switch anyway.")

    rc = 0
    for side in SIDES:
        req = SwitchController.Request()
        if deactivate:
            req.deactivate_controllers = [CONTROLLER]
        else:
            req.activate_controllers = [CONTROLLER]
        req.strictness = SwitchController.Request.STRICT
        # activate_asap/timeout left at defaults: the switch is a state transition, not a
        # motion, so there is nothing to wait on.
        fut = clients[side].call_async(req)
        rclpy.spin_until_future_complete(node, fut, timeout_sec=10.0)
        res = fut.result()
        if res is None:
            node.get_logger().error(f"{side}: no response from switch_controller")
            rc = 1
        elif res.ok:
            node.get_logger().info(f"{side}: {CONTROLLER} {'deactivated' if deactivate else 'activated'}")
        else:
            node.get_logger().error(f"{side}: switch_controller returned ok=False")
            rc = 1

    node.destroy_node()
    rclpy.shutdown()
    return rc


if __name__ == "__main__":
    try:
        sys.exit(main())
    except KeyboardInterrupt:
        print("\ninterrupted", file=sys.stderr)
        sys.exit(130)
