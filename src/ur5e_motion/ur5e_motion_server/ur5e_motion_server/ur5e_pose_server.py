#!/usr/bin/env python3
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.duration import Duration

from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from moveit_msgs.srv import GetPositionIK

from tf2_ros import Buffer, TransformListener
import tf2_geometry_msgs

from pymoveit2 import MoveIt2

from ur5e_interfaces.srv import RunPose


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


def mm_to_m_list(values):
    return [float(x) / 1000.0 for x in values]


def quat_from_rpy_zyx(roll, pitch, yaw):
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    qw = cr * cp * cy + sr * sp * sy
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy

    return float(qx), float(qy), float(qz), float(qw)


class UR5ePoseServer(Node):

    def __init__(self):
        super().__init__("ur5e_pose_server")

        self.declare_parameter("target_frame", "table")
        self.declare_parameter("planning_frame", "base_link")
        self.declare_parameter("group_name", "ur_manipulator")
        self.declare_parameter("ik_link", "tool0")
        self.declare_parameter("ik_timeout_sec", 1.0)
        self.declare_parameter("max_velocity", 0.3)
        self.declare_parameter("max_acceleration", 0.3)
        self.declare_parameter("print_joints", True)

        self.target_frame = self.get_parameter("target_frame").value
        self.planning_frame = self.get_parameter("planning_frame").value
        self.group_name = self.get_parameter("group_name").value
        self.ik_link = self.get_parameter("ik_link").value
        self.ik_timeout = float(self.get_parameter("ik_timeout_sec").value)
        self.max_velocity = float(self.get_parameter("max_velocity").value)
        self.max_acceleration = float(self.get_parameter("max_acceleration").value)
        self.print_joints = bool(self.get_parameter("print_joints").value)

        self._last_js = None
        self.busy = False

        self.create_subscription(
            JointState,
            "/joint_states",
            self._js_cb,
            qos_profile_sensor_data,
        )

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.ik_client = self.create_client(GetPositionIK, "/compute_ik")

        self.moveit2 = MoveIt2(
            node=self,
            joint_names=UR5E_JOINTS,
            base_link_name=self.planning_frame,
            end_effector_name=self.ik_link,
            group_name=self.group_name,
        )

        self.moveit2.max_velocity = self.max_velocity
        self.moveit2.max_acceleration = self.max_acceleration

        self.srv = self.create_service(
            RunPose,
            "/ur5e/run_pose",
            self.run_pose_callback,
        )

        self.get_logger().info("UR5e pose server ready.")
        self.get_logger().info(f"Service: /ur5e/run_pose")
        self.get_logger().info(f"Target frame: {self.target_frame}")
        self.get_logger().info(f"Planning frame: {self.planning_frame}")
        self.get_logger().info(f"IK link: {self.ik_link}")

    def _js_cb(self, msg):
        self._last_js = msg

    def run_pose_callback(self, request, response):

        if self.busy:
            response.success = False
            response.message = "Robot busy."
            return response

        self.busy = True

        try:
            if not self.ik_client.wait_for_service(timeout_sec=5.0):
                response.success = False
                response.message = "Service /compute_ik not available. Start MoveIt first."
                self.busy = False
                return response

            xyz_m = mm_to_m_list(request.target_xyz_mm)
            rpy_rad = deg_to_rad_list(request.target_rpy_deg)

            pose_target = self._build_pose(xyz_m, rpy_rad)
            pose_base = self._transform_pose_to_planning_frame(pose_target)

            if pose_base is None:
                response.success = False
                response.message = "TF transform failed."
                self.busy = False
                return response

            seed_positions = self._get_seed_positions(
                request.seed_from_joint_states,
                request.seed_joints_deg,
            )

            req = self._build_ik_request(pose_base, seed_positions)

            future = self.ik_client.call_async(req)
            rclpy.spin_until_future_complete(self, future)

            res = future.result()

            if res is None:
                response.success = False
                response.message = "IK service returned no result."
                self.busy = False
                return response

            if res.error_code.val != res.error_code.SUCCESS:
                response.success = False
                response.message = f"IK failed. Error code: {res.error_code.val}"
                self.busy = False
                return response

            joint_goal = self._extract_joint_goal(res)

            if joint_goal is None:
                response.success = False
                response.message = "IK solution missing expected UR5e joints."
                self.busy = False
                return response

            if self.print_joints:
                self.get_logger().info("IK joint goal:")
                for name, value in zip(UR5E_JOINTS, joint_goal):
                    self.get_logger().info(f"  {name}: {value:.4f} rad")

            if not request.execute:
                response.success = True
                response.message = "IK successful. execute=false, motion not executed."
                self.busy = False
                return response

            self.get_logger().info("Executing pose via MoveIt2...")
            self.moveit2.move_to_configuration(joint_goal)
            self.moveit2.wait_until_executed()

            response.success = True
            response.message = "Pose executed successfully."

        except Exception as e:
            response.success = False
            response.message = f"Exception: {e}"

        self.busy = False
        return response

    def _build_pose(self, xyz_m, rpy_rad):
        qx, qy, qz, qw = quat_from_rpy_zyx(*rpy_rad)

        pose = PoseStamped()
        pose.header.frame_id = self.target_frame
        pose.header.stamp = self.get_clock().now().to_msg()

        pose.pose.position.x = xyz_m[0]
        pose.pose.position.y = xyz_m[1]
        pose.pose.position.z = xyz_m[2]

        pose.pose.orientation.x = qx
        pose.pose.orientation.y = qy
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw

        return pose

    def _transform_pose_to_planning_frame(self, pose_in):
        try:
            transform = self.tf_buffer.lookup_transform(
                self.planning_frame,
                pose_in.header.frame_id,
                rclpy.time.Time(),
                timeout=Duration(seconds=2.0),
            )

            pose_out = PoseStamped()
            pose_out.header.frame_id = self.planning_frame
            pose_out.header.stamp = self.get_clock().now().to_msg()

            pose_out.pose = tf2_geometry_msgs.do_transform_pose(
                pose_in.pose,
                transform,
            )

            self.get_logger().info(
                f"Pose transformed to {self.planning_frame}: "
                f"x={pose_out.pose.position.x:.4f}, "
                f"y={pose_out.pose.position.y:.4f}, "
                f"z={pose_out.pose.position.z:.4f}"
            )

            return pose_out

        except Exception as e:
            self.get_logger().error(
                f"Could not transform pose from '{pose_in.header.frame_id}' "
                f"to '{self.planning_frame}': {e}"
            )
            return None

    def _get_seed_positions(self, seed_from_joint_states, seed_joints_deg):

        if seed_from_joint_states and self._last_js is not None:
            name_to_pos = dict(zip(self._last_js.name, self._last_js.position))

            if all(j in name_to_pos for j in UR5E_JOINTS):
                self.get_logger().info("Using /joint_states as IK seed.")
                return [float(name_to_pos[j]) for j in UR5E_JOINTS]

            self.get_logger().warn(
                "/joint_states received but does not contain all UR5e joints."
            )

        self.get_logger().info("Using request seed_joints_deg as IK seed.")
        return deg_to_rad_list(seed_joints_deg)

    def _build_ik_request(self, pose_base, seed_positions):
        req = GetPositionIK.Request()
        req.ik_request.group_name = self.group_name
        req.ik_request.ik_link_name = self.ik_link
        req.ik_request.pose_stamped = pose_base

        req.ik_request.timeout.sec = int(self.ik_timeout)
        req.ik_request.timeout.nanosec = int(
            (self.ik_timeout - int(self.ik_timeout)) * 1e9
        )

        seed = JointState()
        seed.header = pose_base.header
        seed.name = UR5E_JOINTS
        seed.position = seed_positions

        req.ik_request.robot_state.joint_state = seed

        return req

    def _extract_joint_goal(self, ik_response):
        sol = ik_response.solution.joint_state
        name_to_pos = dict(zip(sol.name, sol.position))

        if not all(j in name_to_pos for j in UR5E_JOINTS):
            return None

        return [float(name_to_pos[j]) for j in UR5E_JOINTS]


def main():
    rclpy.init()
    node = UR5ePoseServer()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()