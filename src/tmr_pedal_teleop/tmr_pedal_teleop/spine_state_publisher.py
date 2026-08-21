"""Publish the Franka spine height as a JointState so it can be recorded.

``franka_spine_server`` exposes the spine through services and an action only - there is
no state topic at all, so spine height is invisible to any recorder. This node polls
``get_position`` on a timer and publishes ``sensor_msgs/JointState``, which LABS already
handles as an observation (one dimension per named joint).

Polling and publishing are deliberately decoupled:

* ``poll_rate`` is how often the device is asked. The service is backed by HTTPS calls to
  the spine; polling too hard competes with ``spine_bridge``'s jog steps and provokes HTTP
  424 "invalid state or busy". 5 Hz has headroom.
* ``publish_rate`` is how often the last known height is published, and it must be high
  and *gapless*. LABS' ``TemporalSynchronizer`` takes the latest first-message time and
  the earliest last-message time across every configured topic, and raises
  ``SynchronizationError`` if either is more than 1 s from the episode bounds. A topic
  that stalls - because one HTTPS call was slow, or the device was momentarily busy -
  would truncate or fail the whole episode conversion. So every tick republishes the last
  known value rather than skipping.

``GetPosition`` documents metres but returns the device's raw millimetres unconverted
(``GetParameters`` *does* convert, so the limits are genuine metres). The same
detect-don't-hardcode normalisation used by ``spine_bridge`` is applied here, so this
keeps working if the vendor fixes the bug.
"""

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from sensor_msgs.msg import JointState

from franka_spine_msgs.srv import GetParameters, GetPosition


class SpineStatePublisher(Node):
    def __init__(self):
        super().__init__("spine_state_publisher")
        cb = ReentrantCallbackGroup()

        self.declare_parameter("get_position_service", "/franka_spine_node/get_position")
        self.declare_parameter(
            "get_parameters_service", "/franka_spine_node/get_parameters_spine"
        )
        self.declare_parameter("state_topic", "/spine/joint_states")
        self.declare_parameter("joint_name", "spine_z")
        self.declare_parameter("publish_rate", 50.0)  # Hz; must be gapless, see docstring
        self.declare_parameter("poll_rate", 5.0)  # Hz; how often the device is asked

        gp = self.get_parameter
        self.joint_name = gp("joint_name").value
        self.limits = None
        self.position = None  # last known height [m]; None until the first good read
        self._poll_in_flight = False

        self.pub = self.create_publisher(JointState, gp("state_topic").value, 10)
        self.get_pos_cli = self.create_client(
            GetPosition, gp("get_position_service").value, callback_group=cb
        )
        self.get_params_cli = self.create_client(
            GetParameters, gp("get_parameters_service").value, callback_group=cb
        )

        publish_rate = gp("publish_rate").value
        poll_rate = gp("poll_rate").value
        self.create_timer(1.0 / max(publish_rate, 0.1), self._publish, callback_group=cb)
        self.create_timer(1.0 / max(poll_rate, 0.1), self._poll, callback_group=cb)
        self._fetch_limits()
        self.get_logger().info(
            f"spine_state_publisher -> {gp('state_topic').value} "
            f"(publish {publish_rate} Hz, poll {poll_rate} Hz)"
        )

    def _fetch_limits(self):
        if not self.get_params_cli.service_is_ready():
            return
        future = self.get_params_cli.call_async(GetParameters.Request())
        future.add_done_callback(self._on_limits)

    def _on_limits(self, future):
        try:
            resp = future.result()
        except Exception as e:  # noqa: BLE001
            self.get_logger().warn(f"GetParameters failed: {e}")
            return
        if resp and resp.success:
            lim = resp.parameters.user_limits
            self.limits = (lim.lower_limit, lim.upper_limit)
            self.get_logger().info(f"Spine limits: {self.limits} m")

    def _normalise_position(self, raw):
        """Return a position in metres (see the module docstring for why)."""
        if self.limits is None or raw is None:
            return raw
        upper = self.limits[1]
        if upper > 0 and raw > upper * 1.5:
            return raw / 1000.0
        return raw

    def _poll(self):
        if self.limits is None:
            self._fetch_limits()
        # One request at a time: the device rejects overlapping calls as "busy".
        if self._poll_in_flight:
            return
        if not self.get_pos_cli.service_is_ready():
            self.get_logger().warn(
                "Spine get_position service not available.", throttle_duration_sec=5.0
            )
            return
        self._poll_in_flight = True
        future = self.get_pos_cli.call_async(GetPosition.Request())
        future.add_done_callback(self._on_position)

    def _on_position(self, future):
        self._poll_in_flight = False
        try:
            resp = future.result()
        except Exception as e:  # noqa: BLE001
            self.get_logger().warn(
                f"Spine get_position failed: {e}", throttle_duration_sec=5.0
            )
            return
        if not resp or not resp.success:
            self.get_logger().warn(
                "Spine get_position returned success=False", throttle_duration_sec=5.0
            )
            return
        self.position = float(self._normalise_position(resp.position))

    def _publish(self):
        # Republish the last known height rather than skipping a tick: a gap here would
        # truncate or fail the whole episode conversion (see the module docstring).
        # Before the first successful read there is nothing honest to publish, so stay
        # silent - that only ever happens at startup, before recording begins.
        if self.position is None:
            return
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = [self.joint_name]
        msg.position = [self.position]
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = SpineStatePublisher()
    # Same reason spine_bridge needs one: the timer fires service calls whose done
    # callbacks must run while the timer callback is still on the stack.
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
