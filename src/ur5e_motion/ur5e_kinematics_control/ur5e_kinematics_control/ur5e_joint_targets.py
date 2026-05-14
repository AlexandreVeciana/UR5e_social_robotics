#!/usr/bin/env python3

import math
import os
import yaml

import rclpy
from rclpy.node import Node

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration


class MoveUR5eTrajectory(Node):

    def __init__(self):
        super().__init__("move_ur5e_joint_targets")

        self.declare_parameter("trajectory_file", "")
        self.declare_parameter(
            "controller_topic",
            "/joint_trajectory_controller/joint_trajectory"
        )

        self.trajectory_file = str(
            self.get_parameter("trajectory_file").value
        )
        self.controller_topic = str(
            self.get_parameter("controller_topic").value
        )

        if not self.trajectory_file:
            raise RuntimeError("Parameter 'trajectory_file' is empty")

        if not os.path.isfile(self.trajectory_file):
            raise RuntimeError(
                f"Trajectory YAML file not found: {self.trajectory_file}"
            )

        self.targets = self.load_trajectory_file(self.trajectory_file)

        self.joint_names = [
            "shoulder_pan_joint",
            "shoulder_lift_joint",
            "elbow_joint",
            "wrist_1_joint",
            "wrist_2_joint",
            "wrist_3_joint",
        ]

        self.pub = self.create_publisher(
            JointTrajectory,
            self.controller_topic,
            10,
        )

        self.get_logger().info(
            f"Trajectory file: {self.trajectory_file}"
        )
        self.get_logger().info(
            f"Publishing trajectory to: {self.controller_topic}"
        )

        self.start_timer = self.create_timer(1.0, self.send_trajectory)
        self.shutdown_timer = None

        self.sent = False
        self.done = False

    def load_trajectory_file(self, filepath):
        with open(filepath, "r", encoding="utf-8") as f:
            data = yaml.safe_load(f)

        if not isinstance(data, dict):
            raise RuntimeError("YAML root must be a dictionary")

        if "targets" not in data:
            raise RuntimeError("YAML must contain a 'targets' key")

        targets = data["targets"]

        if not isinstance(targets, list) or len(targets) == 0:
            raise RuntimeError("'targets' must be a non-empty list")

        validated_targets = []
        last_time = 0.0

        for i, target in enumerate(targets):
            if not isinstance(target, dict):
                raise RuntimeError(f"Target #{i} must be a dictionary")

            if "joints_deg" not in target or "time_sec" not in target:
                raise RuntimeError(
                    f"Target #{i} must contain 'joints_deg' and 'time_sec'"
                )

            joints_deg = target["joints_deg"]
            time_sec = float(target["time_sec"])

            if not isinstance(joints_deg, list) or len(joints_deg) != 6:
                raise RuntimeError(
                    f"Target #{i}: 'joints_deg' must contain exactly 6 values"
                )

            joints_deg = [float(x) for x in joints_deg]

            if time_sec <= last_time:
                raise RuntimeError(
                    f"Target #{i}: time_sec values must be strictly increasing"
                )

            last_time = time_sec

            validated_targets.append({
                "joints_deg": joints_deg,
                "time_sec": time_sec,
            })

        return validated_targets

    def send_trajectory(self):
        if self.sent:
            return

        msg = JointTrajectory()
        msg.joint_names = self.joint_names

        last_time = 0.0

        for target in self.targets:
            q_deg = target["joints_deg"]
            t_sec = target["time_sec"]

            point = JointTrajectoryPoint()
            point.positions = [math.radians(q) for q in q_deg]

            sec = int(t_sec)
            nanosec = int((t_sec - sec) * 1e9)
            point.time_from_start = Duration(sec=sec, nanosec=nanosec)

            msg.points.append(point)
            last_time = t_sec

        self.pub.publish(msg)

        self.get_logger().info(
            f"Published trajectory with {len(msg.points)} points"
        )
        self.get_logger().info(
            f"Waiting {last_time} s before closing node"
        )

        self.sent = True
        self.start_timer.cancel()
        self.shutdown_timer = self.create_timer(last_time, self.finish_node)

    def finish_node(self):
        self.get_logger().info("Trajectory execution finished. Closing node.")
        if self.shutdown_timer is not None:
            self.shutdown_timer.cancel()
        self.done = True


def main(args=None):
    rclpy.init(args=args)
    node = MoveUR5eTrajectory()

    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()