#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration


class MoveUR5e(Node):

    def __init__(self):
        super().__init__("move_ur5e_joint_target")

        self.declare_parameter(
            "target_deg",
            [0.0, -90.0, 90.0, 0.0, 90.0, 0.0]
        )
        self.declare_parameter(
            "time_sec",
            5.0
        )
        self.declare_parameter(
            "controller_topic",
            "/joint_trajectory_controller/joint_trajectory"
        )

        self.target_deg = self.get_parameter("target_deg").value
        self.time_sec = float(self.get_parameter("time_sec").value)
        self.controller_topic = str(
            self.get_parameter("controller_topic").value
        )

        if len(self.target_deg) != 6:
            self.get_logger().error(
                "Parameter 'target_deg' must contain exactly 6 values"
            )
            raise RuntimeError("Invalid target_deg length")

        self.pub = self.create_publisher(
            JointTrajectory,
            self.controller_topic,
            10
        )

        self.get_logger().info(
            f"Publishing trajectory to: {self.controller_topic}"
        )

        self.start_timer = self.create_timer(1.0, self.send_trajectory)
        self.shutdown_timer = None

        self.sent = False
        self.done = False

    def send_trajectory(self):
        if self.sent:
            return

        target_rad = [math.radians(x) for x in self.target_deg]

        msg = JointTrajectory()
        msg.joint_names = [
            "shoulder_pan_joint",
            "shoulder_lift_joint",
            "elbow_joint",
            "wrist_1_joint",
            "wrist_2_joint",
            "wrist_3_joint",
        ]

        point = JointTrajectoryPoint()
        point.positions = target_rad

        secs = int(self.time_sec)
        nsecs = int((self.time_sec - secs) * 1e9)
        point.time_from_start = Duration(sec=secs, nanosec=nsecs)

        msg.points.append(point)

        self.pub.publish(msg)

        self.get_logger().info(f"Published target_deg = {self.target_deg}")
        self.get_logger().info(f"Published target_rad = {target_rad}")
        self.get_logger().info(f"Waiting {self.time_sec} s before closing node")

        self.sent = True
        self.start_timer.cancel()
        self.shutdown_timer = self.create_timer(self.time_sec, self.finish_node)

    def finish_node(self):
        self.get_logger().info("Motion finished. Closing node.")
        if self.shutdown_timer is not None:
            self.shutdown_timer.cancel()
        self.done = True


def main(args=None):
    rclpy.init(args=args)
    node = MoveUR5e()

    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()