#!/usr/bin/env python3
"""
Direct joint-space motion executor for UR5e.
Bypasses IK entirely — takes joint angles directly from config.
"""

import math
import sys
from typing import Optional, Dict, Any

import rclpy
from rclpy.node import Node
from pymoveit2 import MoveIt2

UR5E_JOINTS = [
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
]


class UR5eMoveToJoints(Node):
    """
    Moves the UR5e directly to a joint configuration.
    No IK involved — joint angles are commanded directly to MoveIt2.
    """

    def __init__(self, config: Optional[Dict[str, Any]] = None):
        super().__init__("ur5e_move_to_joints")

        if config is None:
            config = {}

        self.declare_parameter("step_name",               config.get("step_name", "unnamed_step"))
        self.declare_parameter("joints_deg",              config.get("joints_deg", [0.0, -90.0, 90.0, -90.0, -90.0, 0.0]))
        self.declare_parameter("max_velocity_scale",      config.get("max_velocity_scale", 0.15))
        self.declare_parameter("max_acceleration_scale",  config.get("max_acceleration_scale", 0.15))
        self.declare_parameter("execute",                 config.get("execute", True))
        self.declare_parameter("print_joints",            config.get("print_joints", True))

        self.step_name              = str(self.get_parameter("step_name").value)
        self.joints_deg             = [float(x) for x in self.get_parameter("joints_deg").value]
        self.max_velocity_scale     = float(self.get_parameter("max_velocity_scale").value)
        self.max_acceleration_scale = float(self.get_parameter("max_acceleration_scale").value)
        self.execute_motion         = bool(self.get_parameter("execute").value)
        self.print_joints           = bool(self.get_parameter("print_joints").value)

        if len(self.joints_deg) != 6:
            raise ValueError("joints_deg must contain exactly 6 values.")

        self.joints_rad = [math.radians(v) for v in self.joints_deg]

        self.moveit2 = MoveIt2(
            node=self,
            joint_names=UR5E_JOINTS,
            base_link_name="base_link",
            end_effector_name="tool0",
            group_name="ur_manipulator",
        )
        self.moveit2.max_velocity     = self.max_velocity_scale
        self.moveit2.max_acceleration = self.max_acceleration_scale

        self._finished  = False
        self._exit_code = 0

        self._print_summary()
        self.create_timer(0.1, self._run_once)

    def _print_summary(self):
        self.get_logger().info("=" * 60)
        self.get_logger().info(f"Step name    : {self.step_name}")
        self.get_logger().info(f"Joints [deg] : {self.joints_deg}")
        self.get_logger().info(f"Joints [rad] : {[round(v, 4) for v in self.joints_rad]}")
        self.get_logger().info(f"Velocity     : {self.max_velocity_scale}")
        self.get_logger().info(f"Execute      : {self.execute_motion}")
        self.get_logger().info("=" * 60)

    def _run_once(self):
        if self._finished:
            return

        if not self.execute_motion:
            self.get_logger().info("execute:=false — joints validated, no motion sent.")
            self._exit_code = 0
            self._finished  = True
            return

        try:
            self.get_logger().info(f"Moving to joint configuration: {self.step_name}")
            self.moveit2.move_to_configuration(self.joints_rad)
            self.moveit2.wait_until_executed()
            self.get_logger().info(f"Step complete: {self.step_name}")
            self._exit_code = 0
        except Exception as e:
            self.get_logger().error(f"Motion failed: {e}")
            self._exit_code = 1
        finally:
            self._finished = True


def run_joint_motion(config: Dict[str, Any]) -> int:
    node     = UR5eMoveToJoints(config=config)
    executor = rclpy.executors.SingleThreadedExecutor()
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


def main():
    rclpy.init()
    try:
        node = UR5eMoveToJoints()
        while rclpy.ok() and not node._finished:
            rclpy.spin_once(node, timeout_sec=0.1)
        sys.exit(node._exit_code)
    except KeyboardInterrupt:
        sys.exit(130)
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()