"""Bridge pedals and an optional gamepad to the TMR base command stream.

Single pedals map to base motion (x+/x-, y+/y-, CW/CCW). When a spine combo is
fully pressed the base is held still so the spine bridge can jog instead. A
TwistStamped is published continuously at ``publish_rate`` (zero when nothing is
pressed) so the swerve controller's watchdog is fed and the base stops on release.

The pedals only act on the base while ``/teleop/pedal_mode`` is ``DRIVE`` (see
``mode_manager``).  An Xbox-style gamepad may drive in either mode while RB is held,
but any currently pressed pedal has priority.  This lets the record pedals retain
their actions while the gamepad drives between pedal presses.  This node remains the
only publisher to the controller, avoiding races between independent command streams.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import Joy

from tmr_pedal_teleop.mode_manager import DRIVE, MODE_QOS


class BaseBridge(Node):
    def __init__(self):
        super().__init__("base_bridge")

        self.declare_parameter("cmd_vel_topic", "/swerve_drive_controller/cmd_vel")
        self.declare_parameter("frame_id", "")
        self.declare_parameter("publish_rate", 20.0)
        # SwerveDriveController clamps x/y to 0.1 m/s and yaw to 0.1 rad/s
        # (franka_bringup/config/controllers.yaml), so anything above that is
        # silently limited. Keep the bare-`ros2 run` defaults conservative.
        self.declare_parameter("linear_speed", 0.05)
        self.declare_parameter("angular_speed", 0.05)
        self.declare_parameter("pedal_timeout", 0.5)
        # token -> (axis, sign); axis in {x, y, yaw}
        self.declare_parameter("map_1A_x", 1.0)
        self.declare_parameter("map_2A_x", -1.0)
        self.declare_parameter("map_1B_y", 1.0)
        self.declare_parameter("map_2B_y", -1.0)
        self.declare_parameter("map_1C_yaw", -1.0)
        self.declare_parameter("map_2C_yaw", 1.0)
        # combos that belong to the spine (base is suppressed while held)
        self.declare_parameter("up_combo", ["1A", "2C"])
        self.declare_parameter("down_combo", ["1C", "2A"])
        self.declare_parameter("mode_topic", "/teleop/pedal_mode")
        # Optional companion-side Xbox controller.  RB is a deadman; LB selects the
        # faster (still controller-limited) speed.  Axis defaults match the existing
        # franka_bringup xbox.config.yaml used by the old gamepad_demo alias.
        self.declare_parameter("gamepad_enabled", True)
        self.declare_parameter("gamepad_topic", "/teleop/gamepad/joy")
        self.declare_parameter("gamepad_timeout", 0.5)
        self.declare_parameter("gamepad_deadzone", 0.3)
        self.declare_parameter("gamepad_axis_x", 1)
        self.declare_parameter("gamepad_axis_y", 0)
        self.declare_parameter("gamepad_axis_yaw", 3)
        self.declare_parameter("gamepad_enable_button", 5)  # RB
        self.declare_parameter("gamepad_turbo_button", 4)  # LB
        self.declare_parameter("gamepad_linear_speed", 0.05)
        self.declare_parameter("gamepad_angular_speed", 0.05)
        self.declare_parameter("gamepad_turbo_linear_speed", 0.1)
        self.declare_parameter("gamepad_turbo_angular_speed", 0.1)

        gp = self.get_parameter
        self.frame_id = gp("frame_id").value
        self.linear_speed = gp("linear_speed").value
        self.angular_speed = gp("angular_speed").value
        self.pedal_timeout = gp("pedal_timeout").value
        self.axis_map = {
            "1A": ("x", gp("map_1A_x").value),
            "2A": ("x", gp("map_2A_x").value),
            "1B": ("y", gp("map_1B_y").value),
            "2B": ("y", gp("map_2B_y").value),
            "1C": ("yaw", gp("map_1C_yaw").value),
            "2C": ("yaw", gp("map_2C_yaw").value),
        }
        self.combos = [set(gp("up_combo").value), set(gp("down_combo").value)]

        self.gamepad_enabled = bool(gp("gamepad_enabled").value)
        self.gamepad_timeout = float(gp("gamepad_timeout").value)
        self.gamepad_deadzone = float(gp("gamepad_deadzone").value)
        self.gamepad_axes = {
            "x": int(gp("gamepad_axis_x").value),
            "y": int(gp("gamepad_axis_y").value),
            "yaw": int(gp("gamepad_axis_yaw").value),
        }
        self.gamepad_enable_button = int(gp("gamepad_enable_button").value)
        self.gamepad_turbo_button = int(gp("gamepad_turbo_button").value)
        self.gamepad_linear_speed = float(gp("gamepad_linear_speed").value)
        self.gamepad_angular_speed = float(gp("gamepad_angular_speed").value)
        self.gamepad_turbo_linear_speed = float(
            gp("gamepad_turbo_linear_speed").value
        )
        self.gamepad_turbo_angular_speed = float(
            gp("gamepad_turbo_angular_speed").value
        )

        self.pressed = set()
        self.last_msg_time = self.get_clock().now()
        self.last_joy_time = None
        self.joy_axes = []
        self.joy_buttons = []
        # Default to DRIVE so a bare `ros2 run` without mode_manager still drives.
        self.mode = DRIVE

        self.create_subscription(String, "/pedal/state", self._on_pedal, 10)
        self.create_subscription(String, gp("mode_topic").value, self._on_mode, MODE_QOS)
        if self.gamepad_enabled:
            self.create_subscription(
                Joy, gp("gamepad_topic").value, self._on_joy, 10
            )
        self.pub = self.create_publisher(TwistStamped, gp("cmd_vel_topic").value, 10)
        rate = gp("publish_rate").value
        self.timer = self.create_timer(1.0 / max(rate, 1.0), self._publish)
        self.get_logger().info(
            f"base_bridge -> {gp('cmd_vel_topic').value} "
            f"(v={self.linear_speed} m/s, w={self.angular_speed} rad/s)"
        )
        if self.gamepad_enabled:
            self.get_logger().info(
                f"gamepad coexistence <- {gp('gamepad_topic').value}: "
                "hold RB to drive; pedals have priority; LB selects turbo"
            )

    def _on_pedal(self, msg):
        self.last_msg_time = self.get_clock().now()
        self.pressed = set() if msg.data == "NONE" else set(msg.data.split("+"))

    def _on_mode(self, msg):
        if msg.data == self.mode:
            return
        self.mode = msg.data
        self.get_logger().info(f"base_bridge mode -> {self.mode}")

    def _on_joy(self, msg):
        self.last_joy_time = self.get_clock().now()
        self.joy_axes = list(msg.axes)
        self.joy_buttons = list(msg.buttons)

    def _combo_active(self):
        return any(combo and combo.issubset(self.pressed) for combo in self.combos)

    def _joy_button(self, index):
        return 0 <= index < len(self.joy_buttons) and bool(self.joy_buttons[index])

    def _joy_axis(self, name):
        index = self.gamepad_axes[name]
        value = self.joy_axes[index] if 0 <= index < len(self.joy_axes) else 0.0
        return value if abs(value) >= self.gamepad_deadzone else 0.0

    def _gamepad_command(self, now):
        if not self.gamepad_enabled or self.last_joy_time is None:
            return None
        stale = (now - self.last_joy_time).nanoseconds > self.gamepad_timeout * 1e9
        if stale or not self._joy_button(self.gamepad_enable_button):
            return None
        turbo = self._joy_button(self.gamepad_turbo_button)
        linear = (
            self.gamepad_turbo_linear_speed if turbo else self.gamepad_linear_speed
        )
        angular = (
            self.gamepad_turbo_angular_speed if turbo else self.gamepad_angular_speed
        )
        return (
            self._joy_axis("x") * linear,
            self._joy_axis("y") * linear,
            self._joy_axis("yaw") * angular,
        )

    def _publish(self):
        x = y = yaw = 0.0
        now = self.get_clock().now()
        pedal_stale = (now - self.last_msg_time).nanoseconds > (
            self.pedal_timeout * 1e9
        )
        pedals_active = not pedal_stale and bool(self.pressed)

        # A pedal press always wins.  In RECORD mode or for a spine combo that means a
        # deliberate zero base command, so recording/spine actions cannot coincide with
        # gamepad motion.  With no pedal down, the gamepad may drive in either mode.
        if pedals_active and self.mode == DRIVE and not self._combo_active():
            for token in self.pressed:
                axis_sign = self.axis_map.get(token)
                if axis_sign is None:
                    continue
                axis, sign = axis_sign
                if axis == "x":
                    x += sign * self.linear_speed
                elif axis == "y":
                    y += sign * self.linear_speed
                elif axis == "yaw":
                    yaw += sign * self.angular_speed
        elif not pedals_active:
            gamepad = self._gamepad_command(now)
            if gamepad is not None:
                x, y, yaw = gamepad

        msg = TwistStamped()
        msg.header.stamp = now.to_msg()
        msg.header.frame_id = self.frame_id
        msg.twist.linear.x = x
        msg.twist.linear.y = y
        msg.twist.angular.z = yaw
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = BaseBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
