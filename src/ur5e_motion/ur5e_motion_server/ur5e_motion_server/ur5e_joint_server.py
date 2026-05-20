#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import JointState

from pymoveit2 import MoveIt2

from ur5e_interfaces.srv import RunJoints


UR5E_JOINTS = [
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
]


def deg_to_rad_list(values):
    return [float(x) * math.pi / 180.0 for x in values]


class UR5eJointServer(Node):

    def __init__(self):

        super().__init__("ur5e_joint_server")

        self.declare_parameter("group_name", "ur_manipulator")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("ee_frame", "tool0")

        self.group_name = self.get_parameter("group_name").value
        self.base_frame = self.get_parameter("base_frame").value
        self.ee_frame = self.get_parameter("ee_frame").value

        self._last_js = None
        self.busy = False

        self.create_subscription(
            JointState,
            "/joint_states",
            self._js_cb,
            qos_profile_sensor_data,
        )

        self.moveit2 = MoveIt2(
            node=self,
            joint_names=UR5E_JOINTS,
            base_link_name=self.base_frame,
            end_effector_name=self.ee_frame,
            group_name=self.group_name,
        )

        self.srv = self.create_service(
            RunJoints,
            "/ur5e/run_joints",
            self.run_joints_callback,
        )

        self.get_logger().info("UR5e joint server ready.")
        self.get_logger().info("Service: /ur5e/run_joints")

    def _js_cb(self, msg):
        self._last_js = msg

    def run_joints_callback(self, request, response):

        if self.busy:
            response.success = False
            response.message = "Robot busy."
            return response

        self.busy = True

        try:

            joints_rad = deg_to_rad_list(request.joints_deg)

            self.get_logger().info(
                f"Received joint target (deg): "
                f"{request.joints_deg}"
            )

            self.get_logger().info(
                f"Converted joint target (rad): "
                f"{joints_rad}"
            )

            self.moveit2.max_velocity = float(
                request.max_velocity
            )

            self.moveit2.max_acceleration = float(
                request.max_acceleration
            )

            if not request.execute:

                response.success = True
                response.message = (
                    "execute=false -> motion not executed."
                )

                self.busy = False
                return response

            self.get_logger().info(
                "Executing joint motion via MoveIt2..."
            )

            self.moveit2.move_to_configuration(
                joints_rad
            )

            self.moveit2.wait_until_executed()

            self.get_logger().info(
                "Joint motion execution finished."
            )

            response.success = True
            response.message = (
                "Joint motion executed successfully."
            )

        except Exception as e:

            response.success = False
            response.message = str(e)

        self.busy = False

        return response


def main():

    rclpy.init()

    node = UR5eJointServer()

    rclpy.spin(node)

    rclpy.shutdown()


if __name__ == "__main__":
    main()

