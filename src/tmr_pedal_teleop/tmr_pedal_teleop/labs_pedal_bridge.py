"""Drive LABS data collection from the foot pedals while in RECORD mode.

LABS' own web UI binds a three-pedal footswitch to its three on-screen buttons via a
browser ``keydown`` listener. That cannot work here: ``pedal_state_publisher`` calls
``evdev`` ``grab()`` on both switches, so their a/b/c keystrokes never reach the browser,
and both switches emit identical keycodes anyway. Instead this node talks to the LABS
data-collection REST API directly, which needs no window focus and can tell the two
switches apart - giving all six pedals distinct meanings.

The pedal -> action mapping is state-dependent, mirroring what the LABS UI offers at each
point of its workflow (``TeleoperationActions/config.ts``). ``recording_state`` takes
precedence over ``workflow_state``, exactly as ``getButtonActions`` does in the UI.

Only stdlib HTTP is used so the teleop workspace gains no new Python dependency. All
requests run on a single worker thread, so a slow or unreachable LABS never stalls the
ROS callbacks that keep the base fed.
"""

import json
import queue
import threading
import urllib.error
import urllib.parse
import urllib.request

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from tmr_pedal_teleop.mode_manager import DRIVE, MODE_QOS

TOKENS = ("1A", "1B", "1C", "2A", "2B", "2C")
STATES = ("IDLE", "READY", "SYNCING", "FOLLOWING", "RECORDING", "REVIEWING")

# Recording states take precedence over workflow states when choosing the active row.
RECORDING_STATES = ("RECORDING", "REVIEWING")

# action name -> (path, extra query params). All are POST with an empty body.
ACTIONS = {
    "start_teleop": ("/api/v1/teleop/start", {}),
    "stop_teleop": ("/api/v1/teleop/stop", {}),
    "sync_robots": ("/api/v1/teleop/start_syncing", {}),
    "start_recording": ("/api/v1/recording/start", {}),  # task_id added at dispatch
    "stop_recording": ("/api/v1/recording/stop", {}),
    "save_success": ("/api/v1/recording/save", {"label": "REVIEW_SUCCESS"}),
    "save_failed": ("/api/v1/recording/save", {"label": "REVIEW_FAILED"}),
    "discard": ("/api/v1/recording/discard", {}),
}

# Default mapping. Empty string = pedal does nothing in that state.
DEFAULT_MAP = {
    "IDLE": {"1B": "start_teleop"},
    "READY": {"1B": "stop_teleop", "1C": "sync_robots"},
    "SYNCING": {"1B": "stop_teleop"},
    "FOLLOWING": {"1A": "start_recording", "1B": "stop_teleop"},
    "RECORDING": {"1A": "stop_recording"},
    "REVIEWING": {"2A": "save_success", "2B": "save_failed", "2C": "discard"},
}


class LabsPedalBridge(Node):
    def __init__(self):
        super().__init__("labs_pedal_bridge")

        self.declare_parameter("labs_url", "http://localhost:3001")
        # UUID of the task new episodes are filed under. The LABS UI keeps the selected
        # task in a browser cookie, so there is no server-side "current task" to read -
        # this node needs its own. Take it from the station's config_tasks.yml.
        self.declare_parameter("task_id", "")
        self.declare_parameter("poll_interval", 0.5)  # s between /system/info polls
        self.declare_parameter("min_press_interval", 0.3)  # s debounce per pedal
        # Generous, because LABS' teleop endpoints block on a 7 s coordinator wait when
        # no controller coordinators are deployed (franka_robot.py trigger_controller_
        # coordinator). The LABS-side patch removes that stall; this is the safety net.
        self.declare_parameter("request_timeout", 10.0)
        self.declare_parameter("mode_topic", "/teleop/pedal_mode")
        for state in STATES:
            for token in TOKENS:
                self.declare_parameter(
                    f"map_{state}_{token}", DEFAULT_MAP.get(state, {}).get(token, "")
                )

        gp = self.get_parameter
        self.labs_url = str(gp("labs_url").value).rstrip("/")
        self.task_id = str(gp("task_id").value)
        self.poll_interval = gp("poll_interval").value
        self.min_press_interval = gp("min_press_interval").value
        self.request_timeout = gp("request_timeout").value

        self.action_map = {
            state: {
                token: str(gp(f"map_{state}_{token}").value)
                for token in TOKENS
                if str(gp(f"map_{state}_{token}").value)
            }
            for state in STATES
        }
        for state, bindings in self.action_map.items():
            for token, action in bindings.items():
                if action not in ACTIONS:
                    raise ValueError(
                        f"map_{state}_{token} = {action!r} is not a known action. "
                        f"Known actions: {sorted(ACTIONS)}"
                    )

        self.mode = DRIVE
        self.prev_pressed = set()
        self.workflow_state = "UNKNOWN"
        self.recording_state = "UNKNOWN"
        self._last_press_time = {}

        self._q = queue.Queue()
        self._stop = threading.Event()
        self._worker = threading.Thread(target=self._worker_loop, daemon=True)
        self._worker.start()

        self.create_subscription(String, "/pedal/state", self._on_pedal, 10)
        self.create_subscription(String, gp("mode_topic").value, self._on_mode, MODE_QOS)

        if not self.task_id:
            self.get_logger().warn(
                "No task_id parameter set: 'start_recording' will be refused. "
                "Set it to a task UUID from the station's config_tasks.yml."
            )
        self.get_logger().info(f"labs_pedal_bridge -> {self.labs_url} (mode={self.mode})")

    # ---- ROS callbacks ----------------------------------------------------
    def _on_mode(self, msg):
        if msg.data == self.mode:
            return
        self.mode = msg.data
        # Deliberately do NOT reset prev_pressed. It tracks the true pressed set (updated
        # on every 20 Hz message regardless of mode), so a pedal the operator happens to
        # be resting on when they press 'm' is not seen as a fresh press.
        self.get_logger().info(f"labs_pedal_bridge mode -> {self.mode}")

    def _on_pedal(self, msg):
        pressed = set() if msg.data == "NONE" else set(msg.data.split("+"))
        # /pedal/state is a 20 Hz level heartbeat, so act only on rising edges.
        newly_pressed = pressed - self.prev_pressed
        self.prev_pressed = pressed

        if self.mode == DRIVE:
            return

        now = self.get_clock().now().nanoseconds / 1e9
        for token in sorted(newly_pressed):
            last = self._last_press_time.get(token, 0.0)
            if now - last < self.min_press_interval:
                continue
            self._last_press_time[token] = now
            self._q.put(token)

    # ---- worker thread ----------------------------------------------------
    def _worker_loop(self):
        """Poll LABS for state, and dispatch queued pedal presses.

        Polling happens on the idle path of the same queue wait, so there is exactly one
        thread touching HTTP and no lock is needed.
        """
        while not self._stop.is_set():
            try:
                token = self._q.get(timeout=self.poll_interval)
            except queue.Empty:
                self._refresh_state()
                continue
            # Refresh right before acting so the action matches the true current state
            # rather than one that is up to poll_interval stale.
            self._refresh_state()
            self._dispatch(token)

    def _refresh_state(self):
        info = self._get("/api/v1/system/info")
        if info is None:
            return
        self.workflow_state = info.get("workflow_state", "UNKNOWN")
        self.recording_state = info.get("recording_state", "UNKNOWN")

    def _effective_state(self):
        if self.recording_state in RECORDING_STATES:
            return self.recording_state
        return self.workflow_state

    def _dispatch(self, token):
        state = self._effective_state()
        action = self.action_map.get(state, {}).get(token)
        if not action:
            self.get_logger().info(f"Pedal {token} is unbound in state {state}; ignoring.")
            return

        path, query = ACTIONS[action]
        query = dict(query)
        if action == "start_recording":
            if not self.task_id:
                self.get_logger().error(
                    f"Pedal {token} -> {action} refused: no task_id parameter is set."
                )
                return
            query["task_id"] = self.task_id

        self.get_logger().info(f"Pedal {token} in state {state} -> {action}")
        self._post(path, query)

    # ---- HTTP -------------------------------------------------------------
    def _url(self, path, query=None):
        url = f"{self.labs_url}{path}"
        if query:
            url = f"{url}?{urllib.parse.urlencode(query)}"
        return url

    def _get(self, path):
        req = urllib.request.Request(self._url(path), method="GET")
        try:
            with urllib.request.urlopen(req, timeout=self.request_timeout) as resp:
                return json.loads(resp.read().decode("utf-8"))
        except Exception as e:
            # Log at debug: LABS being down is a normal state while only the robot runs.
            self.get_logger().debug(f"GET {path} failed: {e}")
            return None

    def _post(self, path, query=None):
        req = urllib.request.Request(self._url(path, query), data=b"", method="POST")
        try:
            with urllib.request.urlopen(req, timeout=self.request_timeout) as resp:
                self.get_logger().info(f"POST {path} -> {resp.status}")
        except urllib.error.HTTPError as e:
            body = e.read().decode("utf-8", errors="replace")[:300]
            self.get_logger().error(f"POST {path} -> {e.code}: {body}")
        except Exception as e:
            self.get_logger().error(f"POST {path} failed: {e}")

    def destroy_node(self):
        self._stop.set()
        self._worker.join(timeout=2.0)
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = LabsPedalBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
