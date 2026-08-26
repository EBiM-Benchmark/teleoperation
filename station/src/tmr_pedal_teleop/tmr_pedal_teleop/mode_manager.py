"""Own the foot-pedal mode and publish it on ``/teleop/pedal_mode``.

The two foot switches only have six pedals, and DRIVE mode already spends all of them
on base motion (x+/x-, y+/y-, CW/CCW) plus two spine combos. LABS wants the same pedals
for data collection. Rather than splitting them, ``m`` on the keyboard toggles which
role the whole set plays:

* ``DRIVE``  - ``base_bridge`` / ``spine_bridge`` act on the pedals; ``labs_pedal_bridge``
  ignores them.
* ``RECORD`` - ``labs_pedal_bridge`` acts on the pedals; the motion bridges hold still.

The mode is published with TRANSIENT_LOCAL durability so a bridge that starts (or
restarts) later immediately latches the current mode instead of guessing. It is also
re-published on a slow timer as a heartbeat.

``m`` is read from the normal keyboard via ``keyboard_state_publisher``. That cannot
collide with the pedals: ``pedal_state_publisher`` calls ``evdev`` ``grab()`` on the foot
switches, so their a/b/c keystrokes never reach the terminal.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

DRIVE = "DRIVE"
RECORD = "RECORD"
MODES = (DRIVE, RECORD)

# Latch the mode so late-joining bridges get it without waiting for the heartbeat.
MODE_QOS = QoSProfile(
    depth=1,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    reliability=ReliabilityPolicy.RELIABLE,
)


class ModeManager(Node):
    def __init__(self):
        super().__init__("mode_manager")

        self.declare_parameter("initial_mode", DRIVE)
        self.declare_parameter("toggle_key", "m")
        self.declare_parameter("mode_topic", "/teleop/pedal_mode")
        self.declare_parameter("heartbeat_rate", 1.0)  # Hz

        gp = self.get_parameter
        initial = str(gp("initial_mode").value).upper()
        if initial not in MODES:
            self.get_logger().warn(f"Unknown initial_mode {initial!r}, falling back to {DRIVE}")
            initial = DRIVE
        self.mode = initial
        self.toggle_key = str(gp("toggle_key").value).lower()

        self.pub = self.create_publisher(String, gp("mode_topic").value, MODE_QOS)
        self.create_subscription(String, "/keyboard/state", self._on_key, 10)

        rate = gp("heartbeat_rate").value
        self.create_timer(1.0 / max(rate, 0.1), self._publish)

        self._publish()
        self._log_banner()

    def _on_key(self, msg):
        if msg.data.lower() != self.toggle_key:
            return
        self.mode = RECORD if self.mode == DRIVE else DRIVE
        self._publish()
        self._log_banner()

    def _publish(self):
        msg = String()
        msg.data = self.mode
        self.pub.publish(msg)

    def _log_banner(self):
        if self.mode == DRIVE:
            detail = "pedals drive the base and spine"
        else:
            detail = "pedals drive LABS data collection"
        self.get_logger().info(f"===== PEDAL MODE: {self.mode} ({detail}) =====")


def main(args=None):
    rclpy.init(args=args)
    node = ModeManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
