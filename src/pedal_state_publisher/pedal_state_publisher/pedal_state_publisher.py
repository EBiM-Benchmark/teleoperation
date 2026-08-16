"""Publish the combined state of the two PCsensor foot switches.

The two PCsensor foot switches are physically identical (VID:PID 3553:b001, empty
USB serial), so they can only be told apart by the evdev device node they are read
from - which in turn is pinned to a USB port by the udev rule shipped in
``configs/99-pcsensor-footswitch.rules`` (``/dev/f_pedal_l`` on USB port 2.1 and
``/dev/f_pedal_r`` on port 2.2, which become switch 1 and switch 2).

Each 3-pedal switch, in keyboard mode, emits the letters ``a``/``b``/``c`` (this is
why reading ``stdin`` could not distinguish the two switches - both send the same
keycodes). Here we read each device separately with ``evdev`` and grab it, so the
keystrokes do not leak into the focused window, and publish the set of currently
pressed pedals as tokens ``1A 1B 1C 2A 2B 2C`` on ``/pedal/state``.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

try:
    import evdev
    from evdev import ecodes
    from select import select
except ImportError as exc:  # pragma: no cover - dependency hint
    raise ImportError(
        "python3-evdev is required. Install with `pip install evdev` "
        "(and add the user to the `input` group)."
    ) from exc


# PCsensor FootSwitch, used for autodetection when the explicit device candidates miss.
PEDAL_VENDOR_ID = 0x3553
PEDAL_PRODUCT_ID = 0xB001

# keycode -> pedal letter. Both foot switches emit a/b/c in keyboard mode; devices
# are distinguished by which node the event arrives on, not by the keycode.
DEFAULT_KEYMAP = {
    ecodes.KEY_A: "a",
    ecodes.KEY_B: "b",
    ecodes.KEY_C: "c",
}


class PedalStatePublisher(Node):
    """Read two foot switches via evdev and publish the pressed-pedal set."""

    def __init__(self):
        super().__init__("pedal_state_publisher")

        # Candidate device nodes for switch 1 (left) and switch 2 (right); first
        # existing wins. The by-path entries are a fallback for when the udev rule
        # has not been applied yet.
        self.declare_parameter(
            "device1_candidates",
            [
                "/dev/f_pedal_l",
                "/dev/input/footswitch1",
                "/dev/input/by-path/pci-0000:00:14.0-usb-0:2.1:1.0-event-kbd",
            ],
        )
        self.declare_parameter(
            "device2_candidates",
            [
                "/dev/f_pedal_r",
                "/dev/input/footswitch2",
                "/dev/input/by-path/pci-0000:00:14.0-usb-0:2.2:1.0-event-kbd",
            ],
        )
        self.declare_parameter("publish_rate", 20.0)  # Hz heartbeat

        self.keymap = dict(DEFAULT_KEYMAP)
        # switch index (1/2) -> set of pressed pedal letters
        self.pressed = {1: set(), 2: set()}
        self.last_state = None

        self.pub = self.create_publisher(String, "/pedal/state", 10)

        self.devices = {}
        self.devices[1] = self._open_device(1, "device1_candidates")
        self.devices[2] = self._open_device(2, "device2_candidates")

        # Fall back to autodetection for whichever switch the explicit candidates did not
        # find. Both switches expose an EMPTY USB serial, so they can only be ordered by
        # their USB path - but the path is NOT stable: on this laptop it changed twice in
        # one session (2.1/2.2 -> 1.1.2/1.1.3 -> 1.2/3) simply from replugging. Hard-coded
        # by-path candidates and the ID_PATH udev rule both break every time that happens,
        # which presents as `[Errno 19] No such device` on every read.
        #
        # Autodetection matches on vendor:product instead, then assigns switch 1 / switch 2
        # by sorted USB path so the result is at least deterministic for a given cabling.
        # That ordering is arbitrary with respect to which switch is physically on the
        # left, so CHECK IT (press one pedal, see whether it reports 1x or 2x) and swap
        # with the explicit parameters if it comes out backwards.
        if not all(self.devices.values()):
            self._autodetect_missing()

        self.fd_to_index = {dev.fd: idx for idx, dev in self.devices.items() if dev}

        if not self.fd_to_index:
            raise RuntimeError(
                "No foot switch devices could be opened. Check that the switches are "
                "plugged in, and that the user is in the `input` group."
            )

        rate = self.get_parameter("publish_rate").value
        # Poll evdev fds and re-publish a heartbeat so bridges can run a watchdog.
        self.timer = self.create_timer(1.0 / max(rate, 1.0), self._poll)
        self.get_logger().info(
            f"Pedal publisher started. Switch1={bool(self.devices[1])} "
            f"Switch2={bool(self.devices[2])}. Publishing /pedal/state."
        )

    def _open_device(self, index, param_name):
        candidates = self.get_parameter(param_name).value
        for path in candidates:
            try:
                dev = evdev.InputDevice(path)
                dev.grab()  # exclusive access: keystrokes do not reach other apps
                self.get_logger().info(f"Foot switch {index}: {path} ({dev.name})")
                return dev
            except (FileNotFoundError, PermissionError, OSError):
                continue
        self.get_logger().warn(f"Foot switch {index}: no candidate device found.")
        return None

    def _autodetect_missing(self):
        """Find PCsensor foot switches by vendor:product and fill any unopened slot."""
        already = {d.path for d in self.devices.values() if d}
        wanted_keys = set(DEFAULT_KEYMAP)
        by_phys = {}
        for path in evdev.list_devices():
            try:
                dev = evdev.InputDevice(path)
            except (PermissionError, OSError):
                continue
            is_switch = dev.info.vendor == PEDAL_VENDOR_ID and dev.info.product == PEDAL_PRODUCT_ID
            # Each switch exposes THREE interfaces on one USB path (keyboard, mouse and a
            # third HID node). The mouse node also advertises EV_KEY, so presence of EV_KEY
            # is not enough to pick the right one - require the actual a/b/c keycodes, and
            # keep only ONE interface per physical switch (keyed by `phys`), or both slots
            # get filled from the same pedal and the second switch is never read.
            keys = set(dev.capabilities().get(evdev.ecodes.EV_KEY, []))
            if is_switch and wanted_keys <= keys and dev.path not in already:
                key = dev.phys or dev.path
                if key not in by_phys:
                    by_phys[key] = dev
                    continue
            dev.close()

        # Sort by USB path so a given cabling always yields the same 1/2 assignment.
        found = [by_phys[k] for k in sorted(by_phys)]

        for index in (1, 2):
            if self.devices[index] or not found:
                continue
            dev = found.pop(0)
            try:
                dev.grab()
            except OSError as exc:
                self.get_logger().warn(f"Foot switch {index}: {dev.path} busy ({exc}).")
                dev.close()
                continue
            self.devices[index] = dev
            self.get_logger().info(
                f"Foot switch {index}: {dev.path} ({dev.name}) [autodetected, phys={dev.phys}]"
            )

        for dev in found:
            dev.close()

    def _poll(self):
        fds = list(self.fd_to_index.keys())
        if fds:
            readable, _, _ = select(fds, [], [], 0.0)
            for fd in readable:
                index = self.fd_to_index[fd]
                dev = self.devices[index]
                try:
                    for event in dev.read():
                        if event.type != ecodes.EV_KEY:
                            continue
                        letter = self.keymap.get(event.code)
                        if letter is None:
                            # Unknown keycode - log so the user can extend the keymap.
                            self.get_logger().warn(
                                f"Switch {index}: unmapped keycode {event.code}",
                                throttle_duration_sec=2.0,
                            )
                            continue
                        if event.value == 1:  # key down
                            self.pressed[index].add(letter)
                        elif event.value == 0:  # key up
                            self.pressed[index].discard(letter)
                        # value == 2 (autorepeat) leaves the pressed set unchanged
                except BlockingIOError:
                    pass
                except OSError as exc:
                    self.get_logger().error(f"Switch {index} read error: {exc}")
        self._publish_state()

    def _publish_state(self):
        tokens = sorted(
            f"{idx}{letter.upper()}"
            for idx, letters in self.pressed.items()
            for letter in letters
        )
        state = "+".join(tokens) if tokens else "NONE"
        msg = String()
        msg.data = state
        self.pub.publish(msg)
        if state != self.last_state:
            self.get_logger().info(f"Pedal state: {state}")
            self.last_state = state

    def destroy_node(self):
        for dev in self.devices.values():
            if dev:
                try:
                    dev.ungrab()
                    dev.close()
                except OSError:
                    pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    try:
        node = PedalStatePublisher()
    except (RuntimeError, ImportError) as exc:
        print(f"pedal_state_publisher failed to start: {exc}", flush=True)
        rclpy.try_shutdown()
        return

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
