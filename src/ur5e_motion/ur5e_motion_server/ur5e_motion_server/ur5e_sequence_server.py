#!/usr/bin/env python3

import os
import subprocess

import rclpy
from rclpy.node import Node

from ur5e_interfaces.srv import RunSequence


class UR5eSequenceServer(Node):

    def __init__(self):
        super().__init__("ur5e_sequence_server")

        self.declare_parameter(
            "sequences_dir",
            "/home/ubuntu/ur5e_sequences"
        )

        self.sequences_dir = self.get_parameter(
            "sequences_dir"
        ).value

        self.busy = False

        self.srv = self.create_service(
            RunSequence,
            "/ur5e/run_sequence",
            self.run_sequence_callback
        )

        self.get_logger().info("UR5e sequence server ready.")
        self.get_logger().info(
            f"Sequences directory: {self.sequences_dir}"
        )

    def run_sequence_callback(self, request, response):

        if self.busy:
            response.success = False
            response.message = "Robot busy."
            return response

        sequence_name = request.sequence_name

        sequence_file = os.path.join(
            self.sequences_dir,
            sequence_name
        )

        if not os.path.exists(sequence_file):
            response.success = False
            response.message = (
                f"Sequence file not found: {sequence_file}"
            )
            return response

        self.get_logger().info(
            f"Executing sequence: {sequence_name}"
        )

        self.busy = True

        try:

            cmd = [
                "ros2",
                "run",
                "ur5e_kinematics_pymoveit2",
                "ur5e_pose_sequence_simple_exec",
                "--ros-args",
                "-p",
                f"sequence_file:={sequence_file}"
            ]

            result = subprocess.run(
                cmd,
                capture_output=True,
                text=True
            )

            if result.returncode != 0:

                self.get_logger().error(result.stderr)

                response.success = False
                response.message = "Sequence execution failed."

            else:

                self.get_logger().info(result.stdout)

                response.success = True
                response.message = (
                    f"Sequence '{sequence_name}' executed."
                )

        except Exception as e:

            response.success = False
            response.message = str(e)

        self.busy = False

        return response


def main():

    rclpy.init()

    node = UR5eSequenceServer()

    rclpy.spin(node)

    rclpy.shutdown()


if __name__ == "__main__":
    main()
    
