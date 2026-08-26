"""Bridge spine pedal-combos to the Franka spine (hold-to-jog, incremental).

The spine exposes only absolute moves (``franka_spine_msgs/action/MoveAbsolute``), so a
velocity-style jog has to be synthesised. The obvious approach - command one long move
toward the soft limit and stop it on release - is NOT usable on this hardware:

  * ``franka_spine_server``'s ``Halt`` service posts to ``/spine/api/motion:halt``, which
    does not exist on the device (HTTP 404). The device's swagger declares only
    ``motion:quick-stop``.
  * ``motion:quick-stop`` is a DS402 emergency stop, not a pause: after it the device is
    ``SwitchedOff`` and needs an explicit switch-on before it will move again. Using it on
    every pedal release would be abuse of an e-stop path.
  * Cancelling the action does not command the hardware to stop; it only abandons the
    goal, and the server rejects new goals while ``motion_in_progress``.

So there is no safe way to interrupt a long move. Instead we jog INCREMENTALLY: while the
combo is held, repeatedly command a short absolute move (``jog_step`` metres) from the
current position, chaining the next step only when the previous one finishes. Releasing
the pedals simply stops issuing steps, so the spine always comes to rest on its own.
Worst-case overshoot after release is one ``jog_step``.

Combos (from config):  up = FS1.a + FS2.c ("1A"+"2C"),  down = FS1.c + FS2.a ("1C"+"2A").
The combos only act while ``/teleop/pedal_mode`` is DRIVE; in RECORD mode the pedals
belong to LABS data collection.

This node also publishes the commanded height on ``target_topic`` at every tick, which is
the spine's action dimension in a recorded dataset. It has to be gapless for the whole
episode or LABS' TemporalSynchronizer fails the conversion, so when nothing is jogging it
republishes the current position - the command then really is "stay where you are".
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32, String

from franka_spine_msgs.action import MoveAbsolute
from franka_spine_msgs.srv import GetParameters, GetPosition, SwitchOn

from tmr_pedal_teleop.mode_manager import DRIVE, MODE_QOS


class SpineBridge(Node):
    def __init__(self):
        super().__init__("spine_bridge")
        cb = ReentrantCallbackGroup()

        self.declare_parameter("up_combo", ["1A", "2C"])
        self.declare_parameter("down_combo", ["1C", "2A"])
        # Defaults match the real robot: franka_spine_server registers everything as
        # private names ("~/...") on a node called franka_spine_node, launched with an
        # empty namespace. "get_parameters_spine" is deliberate - the plain name is
        # taken by the standard ROS 2 parameter service.
        self.declare_parameter("move_action", "/franka_spine_node/move_absolute")
        self.declare_parameter("switch_on_service", "/franka_spine_node/switch_on")
        self.declare_parameter(
            "get_parameters_service", "/franka_spine_node/get_parameters_spine"
        )
        self.declare_parameter("get_position_service", "/franka_spine_node/get_position")
        # 0.05 m/s (-> 50 mm/s) is confirmed working by hand on this device.
        self.declare_parameter("velocity", 0.05)
        self.declare_parameter("acceleration", 0.05)
        self.declare_parameter("deceleration", 0.05)
        self.declare_parameter("jog_step", 0.02)  # m per chained move; = max overshoot
        self.declare_parameter("limit_margin", 0.005)
        self.declare_parameter("pedal_timeout", 0.5)
        # After a refused move, wait before retrying. Retrying at tick rate keeps the
        # device "busy" and produces a flood of 424s.
        self.declare_parameter("retry_delay", 0.5)
        self.declare_parameter("mode_topic", "/teleop/pedal_mode")
        # Commanded spine height, republished every tick so the dataset has a spine
        # action dimension. The spine itself offers only an action + services.
        self.declare_parameter("target_topic", "/spine/target_height")
        # Seeds self.position while idle, so target_topic is never empty in an episode
        # that contains no spine jog. Published by spine_state_publisher, which is the
        # single owner of get_position polling - two pollers would fight for the device.
        self.declare_parameter("state_topic", "/spine/joint_states")

        gp = self.get_parameter
        self.up_combo = set(gp("up_combo").value)
        self.down_combo = set(gp("down_combo").value)
        self.velocity = gp("velocity").value
        self.acceleration = gp("acceleration").value
        self.deceleration = gp("deceleration").value
        self.jog_step = gp("jog_step").value
        self.margin = gp("limit_margin").value
        self.pedal_timeout = gp("pedal_timeout").value
        self.retry_delay = gp("retry_delay").value
        self._backoff_until = self.get_clock().now()
        self._pending_target = None
        self._last_target = None  # last value put on target_topic, for gapless publishing

        self.jog_dir = None         # None | "up" | "down": what the pedals currently ask
        self.step_active = False    # a MoveAbsolute is in flight
        self.position = None        # last known spine position [m]
        self.switched_on = False
        self.limits = None          # (lower, upper) [m] once fetched
        self.pressed = set()
        self.last_msg_time = self.get_clock().now()
        # Default to DRIVE so a bare `ros2 run` without mode_manager still jogs.
        self.mode = DRIVE

        self.move_client = ActionClient(
            self, MoveAbsolute, gp("move_action").value, callback_group=cb
        )
        self.switch_on_cli = self.create_client(
            SwitchOn, gp("switch_on_service").value, callback_group=cb
        )
        self.get_params_cli = self.create_client(
            GetParameters, gp("get_parameters_service").value, callback_group=cb
        )
        self.get_pos_cli = self.create_client(
            GetPosition, gp("get_position_service").value, callback_group=cb
        )

        self.target_pub = self.create_publisher(Float32, gp("target_topic").value, 10)
        self.create_subscription(
            JointState, gp("state_topic").value, self._on_spine_state, 10, callback_group=cb
        )

        self.create_subscription(String, "/pedal/state", self._on_pedal, 10, callback_group=cb)
        self.create_subscription(
            String, gp("mode_topic").value, self._on_mode, MODE_QOS, callback_group=cb
        )
        self.create_timer(1.0 / 20.0, self._tick, callback_group=cb)
        self._fetch_limits()
        self.get_logger().info(
            f"spine_bridge ready (incremental jog, step={self.jog_step} m). "
            f"up={sorted(self.up_combo)} down={sorted(self.down_combo)}"
        )

    # ---- limits -----------------------------------------------------------
    def _fetch_limits(self):
        if not self.get_params_cli.service_is_ready():
            return
        future = self.get_params_cli.call_async(GetParameters.Request())
        future.add_done_callback(self._on_limits)

    def _on_limits(self, future):
        try:
            resp = future.result()
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f"GetParameters failed: {exc}")
            return
        if resp and resp.success:
            lim = resp.parameters.user_limits
            self.limits = (lim.lower_limit, lim.upper_limit)
            self.get_logger().info(f"Spine limits: {self.limits} m")

    # ---- position ---------------------------------------------------------
    def _normalise_position(self, raw):
        """Return a position in metres.

        ``franka_spine_server``'s GetPosition documents metres but returns the device's
        raw millimetres unconverted (its GetParameters *does* convert, so the limits are
        genuine metres). Rather than hard-code the bug, detect it: anything well outside
        the soft limits must be millimetres. This keeps working if the vendor fixes it.
        """
        if self.limits is None or raw is None:
            return raw
        upper = self.limits[1]
        if upper > 0 and raw > upper * 1.5:
            return raw / 1000.0
        return raw

    def _refresh_position(self):
        if not self.get_pos_cli.service_is_ready():
            self.get_logger().warn("Spine get_position service not available.", throttle_duration_sec=5.0)
            return
        future = self.get_pos_cli.call_async(GetPosition.Request())
        future.add_done_callback(self._on_position)

    def _on_position(self, future):
        try:
            resp = future.result()
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f"GetPosition failed: {exc}")
            return
        if resp is None or not getattr(resp, "success", True):
            return
        self.position = self._normalise_position(resp.position)
        # Position just became known - start jogging if the pedals are still held.
        if self.jog_dir is not None and not self.step_active:
            self._send_step()

    # ---- pedal input ------------------------------------------------------
    def _on_pedal(self, msg):
        self.last_msg_time = self.get_clock().now()
        self.pressed = set() if msg.data == "NONE" else set(msg.data.split("+"))

    def _on_mode(self, msg):
        if msg.data == self.mode:
            return
        self.mode = msg.data
        self.get_logger().info(f"spine_bridge mode -> {self.mode}")

    def _desired_dir(self):
        # In RECORD mode the pedals belong to LABS. Returning None here (rather than
        # short-circuiting _tick) lets an in-flight jog_step finish normally - this
        # device has no working halt, so abandoning a goal is never the right move.
        if self.mode != DRIVE:
            return None
        up = bool(self.up_combo) and self.up_combo.issubset(self.pressed)
        down = bool(self.down_combo) and self.down_combo.issubset(self.pressed)
        if up and not down:
            return "up"
        if down and not up:
            return "down"
        return None  # neither, or ambiguous both

    def _on_spine_state(self, msg):
        """Seed the idle position from spine_state_publisher.

        Ignored while a step is in flight: during a jog this node's own chained
        get_position reads are authoritative, and _on_step_result depends on them.
        """
        if self.step_active or self.jog_dir is not None:
            return
        try:
            idx = list(msg.name).index("spine_z")
        except ValueError:
            idx = 0
        if idx < len(msg.position):
            self.position = float(msg.position[idx])

    def _publish_target(self):
        """Publish the commanded spine height, so the dataset has a spine action.

        Must be gapless for the whole episode: LABS' TemporalSynchronizer fails the
        conversion if any configured topic starts >1 s late or stops >1 s early. Hence the
        three-step fallback - an in-flight target, else the current position ("stay where
        you are", which is genuinely the command when nothing is jogging), else whatever
        was published last. Only the very first ticks, before any position is known, are
        allowed to publish nothing.
        """
        target = self._pending_target
        if target is None:
            target = self.position
        if target is None:
            # position is briefly None after a refused or errored step, while the truth is
            # re-read. Holding the previous value keeps the stream alive across that gap.
            target = self._last_target
        if target is None:
            return
        self._last_target = target
        msg = Float32()
        msg.data = float(target)
        self.target_pub.publish(msg)

    def _tick(self):
        self._publish_target()

        stale = (self.get_clock().now() - self.last_msg_time).nanoseconds > (
            self.pedal_timeout * 1e9
        )
        desired = None if stale else self._desired_dir()

        if desired != self.jog_dir:
            self.jog_dir = desired
            if desired is None:
                # No halt is issued: there is no working pause on this device. The step
                # in flight (at most jog_step) finishes and the spine stops itself.
                self.get_logger().info("Spine jog released; stopping after current step.")
            else:
                self.get_logger().info(f"Spine jog {desired}.")
                self._begin_jog()
            return

        # Held continuously: chain the next step once the previous one is done.
        if desired is None or self.step_active:
            return
        if self.get_clock().now() < self._backoff_until:
            return  # a refused move is cooling off
        if self.position is None:
            self._refresh_position()  # _on_position sends the step once it lands
            return
        self._send_step()

    # ---- motion -----------------------------------------------------------
    def _begin_jog(self):
        if self.limits is None:
            self._fetch_limits()
            self.get_logger().warn("Spine limits unknown yet; ignoring jog.")
            return
        if not self.switched_on and self.switch_on_cli.service_is_ready():
            self.switch_on_cli.call_async(SwitchOn.Request())
            self.switched_on = True
        # Re-read the true position at the start of every jog so we never accumulate
        # drift from assuming each step landed exactly on its target.
        self._refresh_position()

    def _send_step(self):
        if self.limits is None or self.position is None or self.jog_dir is None:
            return
        if not self.move_client.server_is_ready():
            self.get_logger().warn("Spine action server not available.", throttle_duration_sec=5.0)
            return

        lower, upper = self.limits
        step = self.jog_step if self.jog_dir == "up" else -self.jog_step
        target = self.position + step
        target = max(lower + self.margin, min(upper - self.margin, target))

        # Already at the limit - nothing to command.
        if abs(target - self.position) < 1e-6:
            self.get_logger().info(
                f"Spine at {self.jog_dir} limit ({self.position:.3f} m); not moving further.",
                throttle_duration_sec=2.0,
            )
            return

        goal = MoveAbsolute.Goal()
        goal.position = float(target)
        goal.velocity = float(self.velocity)
        goal.acceleration = float(self.acceleration)
        goal.deceleration = float(self.deceleration)

        self.step_active = True
        self._pending_target = target
        send_future = self.move_client.send_goal_async(goal)
        send_future.add_done_callback(self._on_goal_response)

    def _on_goal_response(self, future):
        try:
            handle = future.result()
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f"Spine goal send failed: {exc}")
            self.step_active = False
            self._pending_target = None  # nothing outstanding; see _publish_target
            return
        if handle is None or not handle.accepted:
            # Usually "another motion is in progress" - back off and retry next tick.
            self.get_logger().warn("Spine step rejected; will retry.", throttle_duration_sec=2.0)
            self.step_active = False
            self._pending_target = None  # nothing outstanding; see _publish_target
            return
        handle.get_result_async().add_done_callback(self._on_step_result)

    def _on_step_result(self, future):
        """Handle a finished step.

        A REJECTED motion still returns a normal result with ``success=False`` (e.g. the
        device answering 424 "invalid state or busy"); it does NOT raise. Advancing the
        target on such a result would march the commanded position away from the spine's
        true position while it never moves, so failures must re-read instead.
        """
        self.step_active = False
        try:
            result = future.result().result
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f"Spine step errored: {exc}")
            self.position = None
            # No move is outstanding any more, so stop reporting one on target_topic.
            self._pending_target = None
            self._backoff_until = self.get_clock().now()
            return

        if not getattr(result, "success", False):
            err = getattr(result, "error", "") or "unknown error"
            self.get_logger().warn(
                f"Spine step did not succeed ({err}); re-reading position.",
                throttle_duration_sec=2.0,
            )
            # Do NOT advance the target. Re-read the truth and pause briefly rather than
            # hammering the device at tick rate, which keeps it "busy".
            self.position = None
            # The move was refused, so it is no longer commanded: clearing this stops
            # target_topic reporting a height the spine never went to.
            self._pending_target = None
            self._backoff_until = self.get_clock().now() + Duration(seconds=self.retry_delay)
            return

        # Succeeded: the spine is at the commanded target.
        self.position = self._pending_target
        self._pending_target = None


def main(args=None):
    rclpy.init(args=args)
    node = SpineBridge()
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
