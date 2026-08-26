"""Minimal rclpy/message stubs so node logic can be tested without a ROS graph.

These tests cover decision logic - mode gating, pedal edge detection, the LABS action
map - which is where the bugs that matter live. Spinning a real ROS graph to exercise it
would be slower, flakier and would not test anything extra.

``install()`` must be called BEFORE importing any tmr_pedal_teleop node module, since it
works by pre-populating ``sys.modules``.
"""

import sys
import types


class Param:
    def __init__(self, value):
        self.value = value


class Time:
    def __init__(self, t):
        self.t = t
        self.nanoseconds = t * 1e9

    def __sub__(self, other):
        return types.SimpleNamespace(nanoseconds=(self.t - other.t) * 1e9)

    def __lt__(self, other):
        return self.t < other.t

    def to_msg(self):
        return "stamp"


class Clock:
    """Manually advanced clock, so tests are deterministic rather than timing-dependent."""

    def __init__(self, t=100.0):
        self.t = t

    def now(self):
        return Time(self.t)


class Logger:
    def __init__(self):
        self.lines = []

    def info(self, m, **k):
        self.lines.append(f"INFO: {m}")

    def warn(self, m, **k):
        self.lines.append(f"WARN: {m}")

    def error(self, m, **k):
        self.lines.append(f"ERROR: {m}")

    def debug(self, m, **k):
        self.lines.append(f"DEBUG: {m}")


class Publisher:
    def __init__(self, topic):
        self.topic = topic
        self.msgs = []

    def publish(self, m):
        self.msgs.append(m)


class Node:
    def __init__(self, name):
        self._name = name
        self._params = {}
        self._clock = Clock()
        self._logger = Logger()
        self.subs = {}
        self.pubs = {}
        self.timers = []

    def declare_parameter(self, name, default):
        self._params[name] = Param(default)

    def get_parameter(self, name):
        return self._params[name]

    def get_clock(self):
        return self._clock

    def get_logger(self):
        return self._logger

    def create_subscription(self, msg_type, topic, cb, qos, callback_group=None):
        self.subs[topic] = cb

    def create_publisher(self, msg_type, topic, qos):
        pub = Publisher(topic)
        self.pubs[topic] = pub
        return pub

    def create_timer(self, period, cb, callback_group=None):
        self.timers.append(cb)
        return None

    def destroy_node(self):
        pass


class String:
    def __init__(self, data=""):
        self.data = data


class Float32:
    def __init__(self, data=0.0):
        self.data = data


class JointState:
    def __init__(self):
        self.header = types.SimpleNamespace(stamp=None, frame_id="")
        self.name = []
        self.position = []


class _Vec:
    def __init__(self):
        self.x = self.y = self.z = 0.0


class _Twist:
    def __init__(self):
        self.linear = _Vec()
        self.angular = _Vec()


class TwistStamped:
    def __init__(self):
        self.header = types.SimpleNamespace(stamp=None, frame_id="")
        self.twist = _Twist()


class _Quat(_Vec):
    def __init__(self):
        super().__init__()
        self.w = 1.0


class _Pose:
    def __init__(self):
        self.position = _Vec()
        self.orientation = _Quat()


class PoseStamped:
    def __init__(self):
        self.header = types.SimpleNamespace(stamp=None, frame_id="")
        self.pose = _Pose()


class Odometry:
    def __init__(self):
        self.header = types.SimpleNamespace(stamp="t", frame_id="odom")
        self.child_frame_id = "base_link"
        self.pose = types.SimpleNamespace(pose=_Pose())
        self.twist = types.SimpleNamespace(twist=_Twist())


def install():
    """Populate sys.modules with the stubs. Idempotent."""
    rclpy = types.ModuleType("rclpy")
    rclpy.init = rclpy.spin = rclpy.try_shutdown = lambda *a, **k: None

    node_mod = types.ModuleType("rclpy.node")
    node_mod.Node = Node

    qos_mod = types.ModuleType("rclpy.qos")
    qos_mod.QoSProfile = lambda **k: object()
    qos_mod.DurabilityPolicy = types.SimpleNamespace(TRANSIENT_LOCAL=1)
    qos_mod.ReliabilityPolicy = types.SimpleNamespace(RELIABLE=1)

    cbg_mod = types.ModuleType("rclpy.callback_groups")
    cbg_mod.ReentrantCallbackGroup = lambda: None

    std = types.ModuleType("std_msgs")
    std_msg = types.ModuleType("std_msgs.msg")
    std_msg.String = String
    std_msg.Float32 = Float32
    std.msg = std_msg

    sensor = types.ModuleType("sensor_msgs")
    sensor_msg = types.ModuleType("sensor_msgs.msg")
    sensor_msg.JointState = JointState
    sensor.msg = sensor_msg

    geo = types.ModuleType("geometry_msgs")
    geo_msg = types.ModuleType("geometry_msgs.msg")
    geo_msg.TwistStamped = TwistStamped
    geo_msg.PoseStamped = PoseStamped
    geo.msg = geo_msg

    nav = types.ModuleType("nav_msgs")
    nav_msg = types.ModuleType("nav_msgs.msg")
    nav_msg.Odometry = Odometry
    nav.msg = nav_msg

    rclpy.node = node_mod
    rclpy.qos = qos_mod
    rclpy.callback_groups = cbg_mod

    for name, mod in [
        ("rclpy", rclpy),
        ("rclpy.node", node_mod),
        ("rclpy.qos", qos_mod),
        ("rclpy.callback_groups", cbg_mod),
        ("std_msgs", std),
        ("std_msgs.msg", std_msg),
        ("sensor_msgs", sensor),
        ("sensor_msgs.msg", sensor_msg),
        ("geometry_msgs", geo),
        ("geometry_msgs.msg", geo_msg),
        ("nav_msgs", nav),
        ("nav_msgs.msg", nav_msg),
    ]:
        sys.modules[name] = mod
