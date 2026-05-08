#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from moveit_msgs.srv import GetPositionIK

from pymoveit2 import MoveIt2


UR5E_JOINTS = [
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
]

def quaternion_from_euler(roll: float, pitch: float, yaw: float):
    """Convert RPY (rad) to quaternion (x, y, z, w). No external deps."""
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)

    qw = cr * cp * cy + sr * sp * sy
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy
    return qx, qy, qz, qw

class UR5eIKDemo(Node):
    def __init__(self):
        super().__init__("ur5e_inverse_kinematics_node")

        # Params (compact)
        self.declare_parameter("target_xyz", [0.4, 0.0, 0.3])
        self.declare_parameter("target_rpy", [0.0, math.pi, 0.0])
        self.declare_parameter("seed_joints", [0.0, -math.pi/2, math.pi/2, 0.0, math.pi/2, 0.0])
        self.declare_parameter("group_name", "ur_manipulator")
        self.declare_parameter("ik_link", "tool0")
        self.declare_parameter("execute", False)

        self.target_xyz = [float(x) for x in self.get_parameter("target_xyz").value]
        self.target_rpy = [float(x) for x in self.get_parameter("target_rpy").value]
        self.seed_joints = [float(x) for x in self.get_parameter("seed_joints").value]
        self.group_name = self.get_parameter("group_name").value
        self.ik_link = self.get_parameter("ik_link").value
        self.execute_motion = bool(self.get_parameter("execute").value)

        # Service client
        self.ik_client = self.create_client(GetPositionIK, "/compute_ik")

        # MoveIt2 (only needed if execute==True)
        self.moveit2 = MoveIt2(
            node=self,
            joint_names=UR5E_JOINTS,
            base_link_name="base_link",
            end_effector_name=self.ik_link,
            group_name=self.group_name,
        )
        self.moveit2.max_velocity = 0.3
        self.moveit2.max_acceleration = 0.3

        self._done = False
        self.create_timer(0.1, self._run_once)

    def _run_once(self):
        if self._done:
            return
        self._done = True

        if not self.ik_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("Service /compute_ik not available.")
            rclpy.shutdown()
            return

        req = GetPositionIK.Request()
        req.ik_request.group_name = self.group_name
        req.ik_request.ik_link_name = self.ik_link

        pose = PoseStamped()
        pose.header.frame_id = "base_link"
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = self.target_xyz
        roll, pitch, yaw = self.target_rpy
        qx, qy, qz, qw = quaternion_from_euler(
            roll,
            pitch,
            yaw,
        )
        pose.pose.orientation.x = float(qx)
        pose.pose.orientation.y = float(qy)
        pose.pose.orientation.z = float(qz)
        pose.pose.orientation.w = float(qw)

        req.ik_request.pose_stamped = pose

        js = JointState()
        js.header = pose.header
        js.name = UR5E_JOINTS
        js.position = self.seed_joints
        req.ik_request.robot_state.joint_state = js

        future = self.ik_client.call_async(req)
        future.add_done_callback(self._on_ik_result)

    def _on_ik_result(self, future):
        try:
            res = future.result()
        except Exception as e:
            self.get_logger().error(f"IK service call failed: {e}")
            rclpy.shutdown()
            return

        if res.error_code.val != res.error_code.SUCCESS:
            self.get_logger().error(f"IK failed, error code: {res.error_code.val}")
            rclpy.shutdown()
            return

        sol = res.solution.joint_state

        # Map solution by name -> position
        name_to_pos = {n: p for n, p in zip(sol.name, sol.position)}
        missing = [j for j in UR5E_JOINTS if j not in name_to_pos]
        if missing:
            self.get_logger().error(f"IK solution missing joints: {missing}")
            rclpy.shutdown()
            return

        joint_goal = [float(name_to_pos[j]) for j in UR5E_JOINTS]

        self.get_logger().info("IK solution (UR5e order):")
        for n, p in zip(UR5E_JOINTS, joint_goal):
            self.get_logger().info(f"  {n}: {p:.4f} rad")

        if self.execute_motion:
            self.get_logger().info("Executing IK solution via MoveIt2...")
            self.moveit2.move_to_configuration(joint_goal)
            self.moveit2.wait_until_executed()
            self.get_logger().info("Execution finished.")

        rclpy.shutdown()


def main():
    rclpy.init()
    node = UR5eIKDemo()
    rclpy.spin(node)


if __name__ == "__main__":
    main()
