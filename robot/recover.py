#!/usr/bin/env python3
"""Recover the TMR after a fault, in place, without restarting start_robot.bash.

ONE script for the whole robot: both arms, the mobile base, and a spine check. It reports
what it finds, fixes what it can, and says plainly when a full bringup is the only option.

    python3 ~/recover.py                  # everything
    python3 ~/recover.py --side left      # one arm only
    python3 ~/recover.py --arms           # skip the base
    python3 ~/recover.py --check          # report only, change nothing
    python3 ~/recover.py --no-activate    # recover but leave controllers inactive

WHAT GOES WRONG, AND WHY THE OBVIOUS CHECK MISLEADS

Two different failures look similar and need different handling:

  * Speed/torque limit or a missed FCI deadline. libfranka aborts the motion with a reflex.
    read() keeps succeeding, so ros2_control demotes NOTHING: the hardware reports `active`,
    the controller reports `active`, and the arm still will not move. Keying recovery off
    the hardware state hides this entirely - an earlier version of this script did exactly
    that and reported success having done nothing.

  * read() itself fails. Then ros2_control demotes the component to `unconfigured`, its
    command interfaces go [unavailable], and no controller can claim them.

So the recovery runs UNCONDITIONALLY and handles both:

    1. deactivate the controller     - it holds the command interfaces, and
                                       perform_command_mode_switch() calls stopRobot()
    2. error_recovery                - libfranka automaticErrorRecovery(), clears the reflex
    3. re-enable the hardware        - only needed in the demoted case
    4. activate the controller       - re-enters perform_command_mode_switch(), which calls
                                       initialize<Mode>Interface() and starts a FRESH control
                                       loop. The aborted Move is never resumed on its own;
                                       this restart is the point.

ONE DDS PARTICIPANT, deliberately. Running these as separate `ros2` commands creates a
participant per call, and each 15-25 s discovery burst can abort whichever arm is still
running - turning a one-arm problem into a two-arm one.

WHEN A BRINGUP IS STILL REQUIRED

  * automaticErrorRecovery() cannot clear an error needing manual intervention: a joint
    limit violation locks the brakes. Unlock them in TMR Desk (https://172.16.16.10/),
    then re-run this. Unlocking alone does NOT require restarting the bash script.
  * If the hardware still will not reach `active`, restart start_robot.bash.
  * Spine 424 after a reboot is not a fault: press power on in Desk.
"""
import argparse
import sys

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from controller_manager_msgs.srv import (ListControllers, ListHardwareComponents,
                                         SetHardwareComponentState, SwitchController)
from franka_msgs.action import ErrorRecovery
from lifecycle_msgs.msg import State

ARM_CONTROLLER = "joint_impedance_controller"
BASE_CONTROLLER = "swerve_drive_controller"
SPINE_URL = "https://172.16.16.10/spine/api/state"
DISCOVERY_TIMEOUT = 15.0
CALL_TIMEOUT = 15.0
RECOVERY_TIMEOUT = 30.0


def spin_until(node, fut, timeout):
    rclpy.spin_until_future_complete(node, fut, timeout_sec=timeout)
    return fut.result()


class Unit:
    """One controller_manager: an arm namespace, or the base at '/'."""

    def __init__(self, node, ns, controller, with_recovery):
        self.node, self.ns, self.controller = node, ns, controller
        self.label = ns.strip("/") or "base"
        pre = ns.rstrip("/")
        self.list_hw = node.create_client(ListHardwareComponents,
                                          f"{pre}/controller_manager/list_hardware_components")
        self.list_ct = node.create_client(ListControllers,
                                          f"{pre}/controller_manager/list_controllers")
        self.set_hw = node.create_client(SetHardwareComponentState,
                                         f"{pre}/controller_manager/set_hardware_component_state")
        self.switch = node.create_client(SwitchController,
                                         f"{pre}/controller_manager/switch_controller")
        self.recovery = (ActionClient(node, ErrorRecovery, f"{pre}/action_server/error_recovery")
                         if with_recovery else None)

    def discover(self):
        for cli in (self.list_hw, self.list_ct, self.set_hw, self.switch):
            if not cli.wait_for_service(timeout_sec=DISCOVERY_TIMEOUT):
                return False
        if self.recovery:
            self.recovery.wait_for_server(timeout_sec=DISCOVERY_TIMEOUT)
        return True

    def hardware(self):
        res = spin_until(self.node, self.list_hw.call_async(ListHardwareComponents.Request()),
                         CALL_TIMEOUT)
        return {c.name: c.state.label for c in res.component} if res else {}

    def controller_state(self):
        res = spin_until(self.node, self.list_ct.call_async(ListControllers.Request()),
                         CALL_TIMEOUT)
        if not res:
            return "unknown"
        return next((c.state for c in res.controller if c.name == self.controller), "absent")

    def claimed(self):
        """True when some controller has claimed this unit's command interfaces."""
        res = spin_until(self.node, self.list_ct.call_async(ListControllers.Request()),
                         CALL_TIMEOUT)
        if not res:
            return False
        for c in res.controller:
            if c.name == self.controller and getattr(c, "claimed_interfaces", []):
                return True
        return False

    def _switch(self, activate=None, deactivate=None):
        req = SwitchController.Request()
        req.activate_controllers = activate or []
        req.deactivate_controllers = deactivate or []
        req.strictness = SwitchController.Request.BEST_EFFORT
        res = spin_until(self.node, self.switch.call_async(req), CALL_TIMEOUT)
        return bool(res and res.ok)

    def report(self):
        hw = self.hardware()
        name = next(iter(hw), "?")
        return (f"  {self.label:<6} hardware={hw.get(name, 'unknown'):<12} "
                f"{self.controller}={self.controller_state()}")

    def recover(self, activate=True) -> bool:
        log = self.node.get_logger()
        hw = self.hardware()
        name = next(iter(hw), None)
        if name is None:
            log.error(f"{self.label}: no hardware component found")
            return False
        log.info(f"{self.label}: hardware '{name}' is '{hw[name]}', "
                 f"{self.controller} is '{self.controller_state()}'")

        # 1. release the command interfaces
        if self._switch(deactivate=[self.controller]):
            log.info(f"{self.label}: {self.controller} deactivated")

        # 2. clear the reflex on the robot itself
        if self.recovery and self.recovery.server_is_ready():
            gh = spin_until(self.node, self.recovery.send_goal_async(ErrorRecovery.Goal()),
                            CALL_TIMEOUT)
            if gh is None or not gh.accepted:
                log.warn(f"{self.label}: error_recovery goal not accepted")
            elif spin_until(self.node, gh.get_result_async(), RECOVERY_TIMEOUT) is None:
                log.warn(f"{self.label}: error_recovery gave no result")
            else:
                log.info(f"{self.label}: error_recovery done")

        # 3. only needed when read() failed and the component was demoted
        if self.hardware().get(name) != "active":
            for chain in ([("active", State.PRIMARY_STATE_ACTIVE)],
                          [("inactive", State.PRIMARY_STATE_INACTIVE),
                           ("active", State.PRIMARY_STATE_ACTIVE)]):
                for label, sid in chain:
                    req = SetHardwareComponentState.Request()
                    req.name = name
                    req.target_state = State(id=sid, label=label)
                    res = spin_until(self.node, self.set_hw.call_async(req), CALL_TIMEOUT)
                    if res is None or not res.ok:
                        break
                if self.hardware().get(name) == "active":
                    break
            if self.hardware().get(name) != "active":
                log.error(f"{self.label}: hardware is '{self.hardware().get(name)}', not active. "
                          f"If Desk shows the joints locked, unlock them and re-run; "
                          f"otherwise restart start_robot.bash.")
                return False
            log.info(f"{self.label}: hardware active again")

        # 4. fresh control loop
        if not activate:
            log.info(f"{self.label}: leaving {self.controller} inactive (--no-activate)")
            return True
        if self._switch(activate=[self.controller]):
            log.info(f"{self.label}: {self.controller} active")
            return True
        log.error(f"{self.label}: could not activate {self.controller}")
        return False


def check_spine(log):
    try:
        import urllib3, requests
        urllib3.disable_warnings(urllib3.exceptions.InsecureRequestWarning)
        r = requests.get(SPINE_URL, verify=False, timeout=5)
        log.info(f"  spine  state={r.text.strip()}")
        log.info("  spine  note: state reads \"SwitchedOn\" even when motion fails with 424. "
                 "If it refuses to move, press power on in Desk (https://172.16.16.10/).")
    except Exception as exc:
        log.warn(f"  spine  unreachable ({exc}). Check robot power and the Desk page.")


def main() -> int:
    ap = argparse.ArgumentParser(description="Recover the TMR in place after a fault.")
    ap.add_argument("--side", choices=["left", "right"], action="append",
                    help="recover only this arm (repeatable)")
    ap.add_argument("--arms", action="store_true", help="skip the base")
    ap.add_argument("--check", action="store_true", help="report only, change nothing")
    ap.add_argument("--no-activate", action="store_true",
                    help="recover but leave controllers inactive")
    args = ap.parse_args()

    rclpy.init()
    node: Node = rclpy.create_node("recover")
    log = node.get_logger()

    units = [Unit(node, f"/{s}", ARM_CONTROLLER, True) for s in (args.side or ["left", "right"])]
    if not args.arms and not args.side:
        units.append(Unit(node, "/", BASE_CONTROLLER, False))

    # Discover everything BEFORE touching anything: no new DDS discovery while an arm is live.
    live = []
    for u in units:
        if u.discover():
            live.append(u)
        else:
            log.warn(f"{u.label}: controller_manager not reachable; skipping")
    if not live:
        log.error("nothing reachable - is the stack running?")
        return 1

    log.info("current state:")
    for u in live:
        log.info(u.report())
    check_spine(log)

    if args.check:
        return 0

    rc = 0
    for u in live:
        if not u.recover(activate=not args.no_activate):
            rc = 1

    log.info("final state:")
    for u in live:
        log.info(u.report())
    if rc:
        log.error("Not fully recovered. Unlock the joints in Desk if they are locked, re-run "
                  "this, and if it still fails restart start_robot.bash.")
    else:
        log.info("Recovered. Hands off the GELLOs - the pose delta was captured at activation.")
    node.destroy_node()
    rclpy.shutdown()
    return rc


if __name__ == "__main__":
    try:
        sys.exit(main())
    except KeyboardInterrupt:
        print("\ninterrupted", file=sys.stderr)
        sys.exit(130)
