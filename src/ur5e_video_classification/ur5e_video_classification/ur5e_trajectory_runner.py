#!/usr/bin/env python3
"""
ur5e_trajectory_runner.py
─────────────────────────
Core trajectory executor for ur5e_social_motion.

Mirrors the mechanism of ur5e_kinematics_control exactly:
  • Reads a YAML file whose root key is ``targets``
  • Each target has ``joints_deg`` (6 values) and ``time_sec``
  • Builds a single JointTrajectory message with all waypoints and publishes it once
  • No MoveIt2, no IK — direct joint-space trajectory

Exposes:
  run_trajectory(yaml_path, controller_topic) -> int
      Blocking call suitable for use inside a background thread.
      Returns 0 on success, 1 on failure.

YAML format (same as ur5e_kinematics_control):
─────────────────────────────────────────────
targets:
  - joints_deg: [-90.0, -90.0, -90.0, -180.0, -90.0, 90.0]
    time_sec: 3.0
  - joints_deg: [-90.0, -60.0, -90.0, -160.0, -90.0, 90.0]
    time_sec: 6.0
"""

import math
import yaml
from pathlib import Path
from typing import List, Dict

import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration

# ── Constants ─────────────────────────────────────────────────────────────────

CONTROLLER_TOPIC_DEFAULT = "/joint_trajectory_controller/joint_trajectory"

JOINT_NAMES = [
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
]


# ── YAML loader ───────────────────────────────────────────────────────────────

def load_targets_yaml(yaml_path: Path) -> List[Dict]:
    """
    Loads and validates a targets YAML file.
    Returns a list of validated target dicts with float values.
    Raises RuntimeError on any validation failure.
    """
    if not yaml_path.exists():
        raise RuntimeError(f"YAML file not found: {yaml_path}")

    with open(yaml_path, "r", encoding="utf-8") as f:
        data = yaml.safe_load(f)

    if not isinstance(data, dict):
        raise RuntimeError(f"YAML root must be a dictionary: {yaml_path}")
    if "targets" not in data:
        raise RuntimeError(f"YAML must contain a 'targets' key: {yaml_path}")

    raw = data["targets"]
    if not isinstance(raw, list) or len(raw) == 0:
        raise RuntimeError(f"'targets' must be a non-empty list: {yaml_path}")

    validated = []
    last_time = 0.0

    for i, entry in enumerate(raw):
        if not isinstance(entry, dict):
            raise RuntimeError(f"Target #{i} must be a dictionary")
        if "joints_deg" not in entry or "time_sec" not in entry:
            raise RuntimeError(
                f"Target #{i} must contain 'joints_deg' and 'time_sec'"
            )

        joints_deg = entry["joints_deg"]
        if not isinstance(joints_deg, list) or len(joints_deg) != 6:
            raise RuntimeError(
                f"Target #{i}: 'joints_deg' must contain exactly 6 values"
            )

        joints_deg = [float(x) for x in joints_deg]
        time_sec   = float(entry["time_sec"])

        if time_sec <= last_time:
            raise RuntimeError(
                f"Target #{i}: 'time_sec' must be strictly increasing "
                f"(got {time_sec}, previous was {last_time})"
            )

        last_time = time_sec
        validated.append({"joints_deg": joints_deg, "time_sec": time_sec})

    return validated


# ── ROS node ──────────────────────────────────────────────────────────────────

class UR5eTrajectoryRunner(Node):
    """
    Single-shot trajectory publisher.

    Publishes one JointTrajectory message containing all waypoints from the
    YAML file, then waits for the trajectory duration before marking itself
    as finished.  Designed to be driven by an external executor so it can
    run inside a background thread without blocking the main node.
    """

    def __init__(
        self,
        yaml_path: Path,
        controller_topic: str = CONTROLLER_TOPIC_DEFAULT,
    ):
        super().__init__("ur5e_trajectory_runner")

        self._yaml_path   = yaml_path
        self._finished    = False
        self._exit_code   = 0
        self._sent        = False
        self._send_timer  = None
        self._wait_timer  = None

        # Load before creating the publisher so errors surface immediately
        self._targets = load_targets_yaml(yaml_path)

        self._pub = self.create_publisher(JointTrajectory, controller_topic, 10)

        self.get_logger().info(
            f"[trajectory_runner] Loaded '{yaml_path.name}' "
            f"({len(self._targets)} waypoints)"
        )

        # 1-second warm-up delay (matches working package behaviour)
        self._send_timer = self.create_timer(1.0, self._send_trajectory)

    # ── Internal callbacks ────────────────────────────────────────────────────

    def _send_trajectory(self) -> None:
        if self._sent:
            return

        msg = JointTrajectory()
        msg.joint_names = JOINT_NAMES

        last_time = 0.0
        for target in self._targets:
            point = JointTrajectoryPoint()
            point.positions = [math.radians(q) for q in target["joints_deg"]]

            t      = target["time_sec"]
            sec    = int(t)
            nanosec = int((t - sec) * 1e9)
            point.time_from_start = Duration(sec=sec, nanosec=nanosec)

            msg.points.append(point)
            last_time = t

        self._pub.publish(msg)

        self.get_logger().info(
            f"[trajectory_runner] Published {len(msg.points)} waypoints; "
            f"waiting {last_time:.1f}s for execution"
        )

        self._sent = True
        self._send_timer.cancel()

        # Wait for the controller to finish executing before signalling done
        self._wait_timer = self.create_timer(last_time, self._finish)

    def _finish(self) -> None:
        self.get_logger().info(
            f"[trajectory_runner] '{self._yaml_path.name}' execution complete"
        )
        if self._wait_timer is not None:
            self._wait_timer.cancel()
        self._finished = True


# ── Public API ────────────────────────────────────────────────────────────────

def run_trajectory(
    yaml_path: Path,
    controller_topic: str = CONTROLLER_TOPIC_DEFAULT,
) -> int:
    """
    Blocking call that publishes a trajectory and waits for it to finish.

    Intended to be called from a background thread (e.g. inside
    _run_sequence_in_thread in robot_command_node.py).

    Returns
    -------
    0  — success
    1  — failure (exception during load or publish)
    130 — interrupted (KeyboardInterrupt)
    """
    try:
        node     = UR5eTrajectoryRunner(yaml_path, controller_topic)
        executor = SingleThreadedExecutor()
        executor.add_node(node)

        try:
            while rclpy.ok() and not node._finished:
                executor.spin_once(timeout_sec=0.1)
        except KeyboardInterrupt:
            node._exit_code = 130
        finally:
            executor.remove_node(node)
            node.destroy_node()

        return node._exit_code

    except Exception as exc:
        # Surface load / validation errors as a non-zero exit code
        import logging
        logging.getLogger(__name__).error(
            f"[run_trajectory] Failed for '{yaml_path}': {exc}"
        )
        return 1


# ── Standalone entry point (for quick testing) ────────────────────────────────

def main():
    import sys
    import argparse

    parser = argparse.ArgumentParser(
        description="Publish a JointTrajectory from a YAML file"
    )
    parser.add_argument("yaml_path", help="Path to the targets YAML file")
    parser.add_argument(
        "--topic",
        default=CONTROLLER_TOPIC_DEFAULT,
        help="Controller topic (default: %(default)s)",
    )
    args = parser.parse_args()

    rclpy.init()
    try:
        code = run_trajectory(Path(args.yaml_path), args.topic)
        sys.exit(code)
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
