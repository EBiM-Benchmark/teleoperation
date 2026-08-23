#!/usr/bin/env python3
"""Recover an arm that went offline after a reflex - WITHOUT restarting start_robot.bash.

WHAT GOES WRONG

When an arm hits a speed/torque limit, or its 1 kHz FCI loop misses deadlines, libfranka
aborts the motion with a reflex:

    libfranka: Move command aborted: motion aborted by reflex! ["..."]

ros2_control then demotes <side>_FrankaHardwareInterface to `unconfigured` while
joint_impedance_controller still reports `active`, so the arm silently stops following its
GELLO. A full bringup fixes it, but takes minutes.

WHAT THIS DOES INSTEAD

Three steps per arm, all through ONE DDS participant:

  1. /<side>/action_server/error_recovery   -> libfranka automaticErrorRecovery(), clears
                                               the reflex
  2. set_hardware_component_state           -> <side>_FrankaHardwareInterface back to active
                                               (via inactive if a direct jump is refused)
  3. switch_controller                      -> joint_impedance_controller active again

One participant matters: two `ros2 control` calls create two participants, and the second
one's 15-25 s discovery burst aborts whichever arm is already running. Same reason
activate_arms.py exists.

WHEN THIS IS NOT ENOUGH

automaticErrorRecovery() cannot clear an error that requires manual intervention - a joint
limit violation locks the brakes. If Desk shows the joints locked, unlock them there first,
then run this. If this still fails, fall back to a full bringup.

USAGE
    python3 ~/recover_arms.py                 # both arms
    python3 ~/recover_arms.py --side left     # one arm
    python3 ~/recover_arms.py --no-activate   # recover but leave the controller inactive
"""
import argparse
import sys

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from controller_manager_msgs.srv import ListHardwareComponents, SetHardwareComponentState
from controller_manager_msgs.srv import SwitchController
from franka_msgs.action import ErrorRecovery
from lifecycle_msgs.msg import State

CONTROLLER = "joint_impedance_controller"
DISCOVERY_TIMEOUT = 15.0
CALL_TIMEOUT = 15.0
RECOVERY_TIMEOUT = 30.0


def spin_until(node, fut, timeout):
    rclpy.spin_until_future_complete(node, fut, timeout_sec=timeout)
    return fut.result()


def hw_state(node, list_cli):
    """Map hardware component name -> current state label."""
    res = spin_until(node, list_cli.call_async(ListHardwareComponents.Request()), CALL_TIMEOUT)
    if res is None:
        return {}
    return {c.name: c.state.label for c in res.component}


def recover(node, side, clients, activate=True) -> bool:
    log = node.get_logger()
    ok = True

    before = hw_state(node, clients["list"])
    name = next((n for n in before if n.startswith(side)), f"{side}_FrankaHardwareInterface")
    log.info(f"{side}: hardware '{name}' is '{before.get(name, 'unknown')}'")

    if before.get(name) == "active":
        log.info(f"{side}: already active; only re-checking the controller")
    else:
        # 1. clear the reflex on the robot itself
        log.info(f"{side}: sending error_recovery ...")
        gh = spin_until(node, clients["recover"].send_goal_async(ErrorRecovery.Goal()), CALL_TIMEOUT)
        if gh is None or not gh.accepted:
            log.warn(f"{side}: error_recovery goal not accepted (continuing anyway)")
        else:
            if spin_until(node, gh.get_result_async(), RECOVERY_TIMEOUT) is None:
                log.warn(f"{side}: error_recovery gave no result (continuing anyway)")

        # 2. bring the hardware component back. A direct unconfigured -> active jump is not
        #    always allowed, so fall back to stepping through inactive.
        for targets in ([("active", State.PRIMARY_STATE_ACTIVE)],
                        [("inactive", State.PRIMARY_STATE_INACTIVE),
                         ("active", State.PRIMARY_STATE_ACTIVE)]):
            for label, sid in targets:
                req = SetHardwareComponentState.Request()
                req.name = name
                req.target_state = State(id=sid, label=label)
                res = spin_until(node, clients["set_hw"].call_async(req), CALL_TIMEOUT)
                if res is None or not res.ok:
                    break
            if hw_state(node, clients["list"]).get(name) == "active":
                break

        final = hw_state(node, clients["list"]).get(name, "unknown")
        if final != "active":
            log.error(f"{side}: hardware is '{final}', not active. If Desk shows the joints "
                      f"locked, unlock them there and re-run; otherwise a full bringup is needed.")
            return False
        log.info(f"{side}: hardware active again")

    # 3. controller back on
    if activate:
        req = SwitchController.Request()
        req.activate_controllers = [CONTROLLER]
        req.strictness = SwitchController.Request.BEST_EFFORT
        res = spin_until(node, clients["switch"].call_async(req), CALL_TIMEOUT)
        if res is None or not res.ok:
            log.error(f"{side}: could not activate {CONTROLLER}")
            ok = False
        else:
            log.info(f"{side}: {CONTROLLER} active")
    return ok


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--side", choices=["left", "right"], action="append",
                    help="recover only this side (repeatable); default both")
    ap.add_argument("--no-activate", action="store_true",
                    help="recover the hardware but leave joint_impedance_controller inactive")
    args = ap.parse_args()
    sides = args.side or ["left", "right"]

    rclpy.init()
    node: Node = rclpy.create_node("recover_arms")
    log = node.get_logger()

    per_side = {}
    for s in sides:
        per_side[s] = {
            "list": node.create_client(ListHardwareComponents,
                                       f"/{s}/controller_manager/list_hardware_components"),
            "set_hw": node.create_client(SetHardwareComponentState,
                                         f"/{s}/controller_manager/set_hardware_component_state"),
            "switch": node.create_client(SwitchController,
                                         f"/{s}/controller_manager/switch_controller"),
            "recover": ActionClient(node, ErrorRecovery, f"/{s}/action_server/error_recovery"),
        }

    # Discover everything BEFORE touching anything: no new discovery while an arm is live.
    for s in sides:
        for key in ("list", "set_hw", "switch"):
            if not per_side[s][key].wait_for_service(timeout_sec=DISCOVERY_TIMEOUT):
                log.error(f"/{s}/controller_manager/... ({key}) not available - is the stack up?")
                return 1
        if not per_side[s]["recover"].wait_for_server(timeout_sec=DISCOVERY_TIMEOUT):
            log.warn(f"/{s}/action_server/error_recovery not available; will skip that step")
    log.info("discovered; recovering now")

    rc = 0
    for s in sides:
        if not recover(node, s, per_side[s], activate=not args.no_activate):
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
