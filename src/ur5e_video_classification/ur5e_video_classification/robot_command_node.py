#!/usr/bin/env python3
"""
robot_command_node.py
─────────────────────
Robot Command Node for ur5e_social_motion.

Receives video classification predictions and dispatches the matching
gesture trajectory to the UR5e via the JointTrajectoryController —
the same mechanism used by ur5e_kinematics_control, no MoveIt2 required.

Topic subscriptions:
  /prediction/result     (std_msgs/String)
  /prediction/confidence (std_msgs/Float32)

Topic publications:
  /robot/status (std_msgs/String) — human-readable status updates

ROS parameters:
  confidence_threshold  float  0.90   — minimum confidence (0-1 scale)
  stability_frames      int    3      — consecutive identical predictions required
  command_cooldown      float  2.0    — minimum seconds between commands
  ignore_no_action      bool   true   — skip NoAction predictions
  controller_topic      str           — JointTrajectory controller topic
"""

import threading
import time
from collections import deque
from pathlib import Path

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32

from ament_index_python.packages import get_package_share_directory
from ur5e_video_classification.sequence_runner import (
    run_motion_sequence,
    CONTROLLER_TOPIC_DEFAULT,
)

# ── Package paths ─────────────────────────────────────────────────────────────

PACKAGE_SHARE_DIR = Path(get_package_share_directory("ur5e_video_classification"))
CONFIG_DIR        = PACKAGE_SHARE_DIR / "config"

# ── Action → YAML file mapping ────────────────────────────────────────────────

ACTION_YAML_MAP: dict[str, str] = {
    "Bow":          "ur5e_social_bow.yaml",
    "Cross":        "ur5e_social_cross.yaml",
    "GoldenOrder":  "ur5e_social_golden_order.yaml",
    "HalfSun":      "ur5e_social_half_sun.yaml",
    "Handshake":    "ur5e_social_handshake.yaml",
    "NoAction":     "ur5e_social_idle.yaml",
    "PointDown":    "ur5e_social_point_down.yaml",
    "PraiseTheSun": "ur5e_social_praise_the_sun.yaml",
    "SideLeg":      "ur5e_social_side_leg.yaml",
    "Stop":         "ur5e_social_stop.yaml",
    "Wave":         "ur5e_social_wave.yaml",
}


# ── Helpers ───────────────────────────────────────────────────────────────────

def _publish_status(pub, text: str) -> None:
    msg      = String()
    msg.data = text
    pub.publish(msg)


# ── Background motion thread ──────────────────────────────────────────────────

def _run_sequence_in_thread(
    yaml_path: Path,
    controller_topic: str,
    status_pub,
    logger,
    done_event: threading.Event,
) -> None:
    """
    Executes a gesture trajectory inside a daemon thread.

    Calls run_motion_sequence() which publishes the full JointTrajectory
    and blocks until the controller is expected to have finished.
    """
    try:
        exit_code = run_motion_sequence(
            yaml_path,
            controller_topic=controller_topic,
            logger=logger,
        )

        if exit_code != 0:
            _publish_status(
                status_pub,
                f"FAILED: {yaml_path.name} (exit {exit_code})",
            )
            logger.error(
                f"[motion] Trajectory failed: {yaml_path.name} "
                f"(exit code {exit_code})"
            )

    except Exception as exc:
        logger.error(f"[motion] Unexpected error in motion thread: {exc}")
        _publish_status(status_pub, f"ERROR: {exc}")

    finally:
        done_event.set()


# ── Main node ─────────────────────────────────────────────────────────────────

class RobotCommandNode(Node):
    """
    Listens to gesture predictions and triggers the matching YAML trajectory
    on the UR5e.  Only one trajectory runs at a time; incoming predictions
    are ignored while motion is executing.
    """

    def __init__(self):
        super().__init__("robot_command_node")

        # ── ROS parameters ────────────────────────────────────────────────────
        self.declare_parameter("confidence_threshold", 0.90)
        self.declare_parameter("stability_frames",     3)
        self.declare_parameter("command_cooldown",     2.0)
        self.declare_parameter("ignore_no_action",     True)
        self.declare_parameter("controller_topic",     CONTROLLER_TOPIC_DEFAULT)

        self.confidence_threshold = float(self.get_parameter("confidence_threshold").value)
        self.stability_frames     = int(self.get_parameter("stability_frames").value)
        self.command_cooldown     = float(self.get_parameter("command_cooldown").value)
        self.ignore_no_action     = bool(self.get_parameter("ignore_no_action").value)
        self.controller_topic     = str(self.get_parameter("controller_topic").value)

        # ── Internal state ────────────────────────────────────────────────────
        self.last_prediction    = None
        self.last_confidence    = 0.0
        self.prediction_history = deque(maxlen=self.stability_frames)
        self.last_command_time  = time.time()

        self._motion_lock    = threading.Lock()
        self._motion_running = False
        self._done_event     = threading.Event()

        # ── Subscribers ───────────────────────────────────────────────────────
        self.create_subscription(String,  "/prediction/result",     self._on_result,     10)
        self.create_subscription(Float32, "/prediction/confidence", self._on_confidence, 10)

        # ── Publishers ────────────────────────────────────────────────────────
        self.status_pub = self.create_publisher(String, "/robot/status", 10)

        # ── Timer: polls done_event to reset _motion_running cleanly ─────────
        self.create_timer(0.1, self._check_motion_done)

        self.get_logger().info("Robot Command Node started")
        self.get_logger().info(f"  confidence_threshold : {self.confidence_threshold:.2f}")
        self.get_logger().info(f"  stability_frames     : {self.stability_frames}")
        self.get_logger().info(f"  command_cooldown     : {self.command_cooldown}s")
        self.get_logger().info(f"  ignore_no_action     : {self.ignore_no_action}")
        self.get_logger().info(f"  controller_topic     : {self.controller_topic}")

    # ── Subscription callbacks ────────────────────────────────────────────────

    def _on_result(self, msg: String) -> None:
        self.last_prediction = msg.data
        self.prediction_history.append(msg.data)
        if self._should_send_command():
            self._dispatch_motion()

    def _on_confidence(self, msg: Float32) -> None:
        self.last_confidence = msg.data

    # ── Command gating ────────────────────────────────────────────────────────

    def _should_send_command(self) -> bool:
        # 1. Confidence gate (prediction node publishes 0-100 scale)
        if self.last_confidence < self.confidence_threshold * 100.0:
            return False
        # 2. Stability gate
        if len(self.prediction_history) < self.stability_frames:
            return False
        if len(set(self.prediction_history)) != 1:
            return False
        # 3. Skip NoAction if configured
        if self.ignore_no_action and self.last_prediction == "NoAction":
            return False
        # 4. Cooldown gate
        if time.time() - self.last_command_time < self.command_cooldown:
            return False
        # 5. Busy gate
        with self._motion_lock:
            if self._motion_running:
                self.get_logger().debug(
                    "Motion already running — skipping prediction"
                )
                return False
        return True

    # ── Motion dispatch ───────────────────────────────────────────────────────

    def _dispatch_motion(self) -> None:
        action    = self.last_prediction
        yaml_name = ACTION_YAML_MAP.get(action)

        if yaml_name is None:
            self.get_logger().warn(f"No YAML mapping for action '{action}'")
            return

        yaml_path = CONFIG_DIR / yaml_name
        if not yaml_path.exists():
            self.get_logger().error(f"YAML file not found: {yaml_path}")
            return

        with self._motion_lock:
            self._motion_running = True

        self._done_event.clear()
        self.last_command_time = time.time()
        self.prediction_history.clear()

        status_text = (
            f"Executing: {action} | "
            f"conf={self.last_confidence:.1f}% | "
            f"file={yaml_name}"
        )
        _publish_status(self.status_pub, status_text)
        self.get_logger().info(status_text)

        thread = threading.Thread(
            target=_run_sequence_in_thread,
            args=(
                yaml_path,
                self.controller_topic,
                self.status_pub,
                self.get_logger(),
                self._done_event,
            ),
            daemon=True,
        )
        thread.start()

    # ── Motion completion polling ─────────────────────────────────────────────

    def _check_motion_done(self) -> None:
        """Timer callback: reset _motion_running once the thread signals done."""
        if self._done_event.is_set():
            with self._motion_lock:
                if self._motion_running:
                    self._motion_running = False
                    self.get_logger().info(
                        "Motion complete — ready for next gesture"
                    )
                    _publish_status(self.status_pub, "IDLE — waiting for gesture")
            self._done_event.clear()


# ── Entry point ───────────────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)
    node = RobotCommandNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()