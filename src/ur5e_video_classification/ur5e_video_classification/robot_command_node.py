#!/usr/bin/env python3
"""
Robot Command Node for UR5e

Receives video classification predictions and dispatches motion sequences
to the UR5e robot by running YAML-defined pose sequences in a background thread.

Topic subscriptions:
  /prediction/result     (std_msgs/String)
  /prediction/confidence (std_msgs/Float32)

Topic publications:
  /robot/status (std_msgs/String) - Human-readable status updates
"""

import threading
import time
from collections import deque
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from std_msgs.msg import String, Float32

from ament_index_python.packages import get_package_share_directory
from ur5e_video_classification.sequence_runner import load_sequence_yaml, build_step_config
from ur5e_video_classification.ur5e_move_to_pose_exe import UR5eMoveToPoseViaIK

# ── Package paths ─────────────────────────────────────────────────────────────
PACKAGE_SHARE_DIR = Path(get_package_share_directory("ur5e_social_motion"))
CONFIG_DIR = PACKAGE_SHARE_DIR / "config"

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


# ── Thread-safe motion executor ───────────────────────────────────────────────

def _run_sequence_in_thread(
    yaml_path: Path,
    status_pub,
    logger,
    done_event: threading.Event,
) -> None:
    """
    Executes a YAML motion sequence inside a daemon thread.

    Each step gets its own SingleThreadedExecutor so that spinning
    the pose-motion node does not interfere with the main node's executor.
    """
    try:
        sequence = load_sequence_yaml(yaml_path)
        logger.info(f"[motion] Loaded '{yaml_path.name}' ({len(sequence)} steps)")

        for i, step in enumerate(sequence, start=1):
            step_name = step.get("step_name", f"step_{i}")
            logger.info(f"[motion] Step {i}/{len(sequence)}: {step_name}")

            config = build_step_config(step)
            node = UR5eMoveToPoseViaIK(config=config)
            executor = SingleThreadedExecutor()
            executor.add_node(node)

            try:
                while rclpy.ok() and not node._finished:
                    executor.spin_once(timeout_sec=0.1)
            finally:
                executor.remove_node(node)
                node.destroy_node()

            if node._exit_code != 0:
                _publish_status(
                    status_pub,
                    f"FAILED step '{step_name}' (exit {node._exit_code})",
                )
                logger.error(f"[motion] Step failed: {step_name} (exit {node._exit_code})")
                return

            sleep_after = float(step.get("sleep_after", 0.0))
            if sleep_after > 0.0:
                logger.info(f"[motion] Holding {sleep_after:.2f}s after '{step_name}'")
                time.sleep(sleep_after)

        logger.info(f"[motion] Sequence '{yaml_path.name}' completed successfully")

    except Exception as exc:
        logger.error(f"[motion] Unexpected error in motion thread: {exc}")
    finally:
        done_event.set()


def _publish_status(pub, text: str) -> None:
    msg = String()
    msg.data = text
    pub.publish(msg)


# ── Main node ─────────────────────────────────────────────────────────────────

class RobotCommandNode(Node):
    """
    Listens to gesture predictions and triggers the matching YAML motion
    sequence on the UR5e.  Only one sequence runs at a time; incoming
    predictions are ignored while a motion is executing.
    """

    def __init__(self):
        super().__init__("robot_command_node")

        # ── ROS parameters ────────────────────────────────────────────────────
        self.declare_parameter("confidence_threshold", 0.90)
        self.declare_parameter("stability_frames", 3)
        self.declare_parameter("command_cooldown", 2.0)
        self.declare_parameter("ignore_no_action", True)

        self.confidence_threshold = self.get_parameter("confidence_threshold").value
        self.stability_frames     = self.get_parameter("stability_frames").value
        self.command_cooldown     = self.get_parameter("command_cooldown").value
        self.ignore_no_action     = self.get_parameter("ignore_no_action").value

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

        # ── Timer: poll done_event to reset _motion_running cleanly ──────────
        self.create_timer(0.1, self._check_motion_done)

        self.get_logger().info("Robot Command Node started")
        self.get_logger().info(f"  confidence_threshold : {self.confidence_threshold:.2f}")
        self.get_logger().info(f"  stability_frames     : {self.stability_frames}")
        self.get_logger().info(f"  command_cooldown     : {self.command_cooldown}s")
        self.get_logger().info(f"  ignore_no_action     : {self.ignore_no_action}")

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
        # 1. Confidence gate (node publishes 0-100 range)
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
                self.get_logger().debug("Motion already running — skipping prediction")
                return False
        return True

    # ── Motion dispatch ───────────────────────────────────────────────────────

    def _dispatch_motion(self) -> None:
        action = self.last_prediction
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
            f"Executing: {action} | conf={self.last_confidence:.1f}% | file={yaml_name}"
        )
        _publish_status(self.status_pub, status_text)
        self.get_logger().info(status_text)

        thread = threading.Thread(
            target=_run_sequence_in_thread,
            args=(yaml_path, self.status_pub, self.get_logger(), self._done_event),
            daemon=True,
        )
        thread.start()

    def _check_motion_done(self) -> None:
        """Timer callback: reset _motion_running once the thread signals done."""
        if self._done_event.is_set():
            with self._motion_lock:
                if self._motion_running:
                    self._motion_running = False
                    self.get_logger().info("Motion complete — ready for next gesture")
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
