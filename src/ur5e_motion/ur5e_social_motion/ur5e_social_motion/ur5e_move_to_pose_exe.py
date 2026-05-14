#!/usr/bin/env python3

import math
import sys
from typing import Optional, Dict, Any

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

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


def quat_from_rpy_zyx(roll: float, pitch: float, yaw: float):
    """
    Convention:
      R = Rz(yaw) * Ry(pitch) * Rx(roll)

    Returns quaternion in ROS order: (qx, qy, qz, qw).
    """
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


class UR5eMoveToPoseViaIK(Node):
    """
    Workflow:
      Pose goal -> /compute_ik -> joint goal -> MoveIt2 move_to_configuration -> execute

    Input convention for this social robotics package:
      - target_xyz_mm: [x, y, z] in mm
      - target_rpy_deg: [roll, pitch, yaw] in degrees
      - reference_frame:
          * "base"  -> pose already in ROS base_link convention
          * "table" -> intuitive table/workspace frame
    """

    def __init__(self, config: Optional[Dict[str, Any]] = None):
        super().__init__("ur5e_move_to_pose")

        if config is None:
            config = {}

        # --- Declare ROS parameters, using config values as defaults when provided
        self.declare_parameter("step_name", config.get("step_name", "unnamed_step"))

        self.declare_parameter("target_xyz_mm", config.get("target_xyz_mm", [350.0, 0.0, 450.0]))
        self.declare_parameter("target_rpy_deg", config.get("target_rpy_deg", [0.0, 0.0, 0.0]))

        self.declare_parameter("reference_frame", config.get("reference_frame", "table"))
        self.declare_parameter("table_frame_yaw_offset_deg", config.get("table_frame_yaw_offset_deg", 180.0))

        self.declare_parameter("group_name", config.get("group_name", "ur_manipulator"))
        self.declare_parameter("ik_link_name", config.get("ik_link_name", "tool0"))
        self.declare_parameter(
            "seed_joints_deg",
            config.get("seed_joints_deg", [0.0, -90.0, 90.0, -90.0, -90.0, 0.0]),
        )
        self.declare_parameter("use_seed_joints", config.get("use_seed_joints", True))
        self.declare_parameter("seed_from_joint_states", config.get("seed_from_joint_states", True))
        self.declare_parameter("ik_timeout_sec", config.get("ik_timeout_sec", 0.2))

        self.declare_parameter("max_velocity_scale", config.get("max_velocity_scale", 0.15))
        self.declare_parameter("max_acceleration_scale", config.get("max_acceleration_scale", 0.15))
        self.declare_parameter("execute", config.get("execute", True))

        self.declare_parameter("print_joints", config.get("print_joints", True))

        # --- Read final values from ROS parameters
        self.step_name = str(self.get_parameter("step_name").value)

        self.target_xyz_mm = [float(x) for x in self.get_parameter("target_xyz_mm").value]
        self.target_rpy_deg = [float(x) for x in self.get_parameter("target_rpy_deg").value]

        self.reference_frame = str(self.get_parameter("reference_frame").value).strip().lower()
        self.table_frame_yaw_offset_deg = float(self.get_parameter("table_frame_yaw_offset_deg").value)

        self.group_name = str(self.get_parameter("group_name").value)
        self.ik_link_name = str(self.get_parameter("ik_link_name").value)
        self.seed_joints_deg = [float(x) for x in self.get_parameter("seed_joints_deg").value]
        self.use_seed_joints = bool(self.get_parameter("use_seed_joints").value)
        self.seed_from_joint_states = bool(self.get_parameter("seed_from_joint_states").value)
        self.ik_timeout = float(self.get_parameter("ik_timeout_sec").value)

        self.max_velocity_scale = float(self.get_parameter("max_velocity_scale").value)
        self.max_acceleration_scale = float(self.get_parameter("max_acceleration_scale").value)
        self.execute_motion = bool(self.get_parameter("execute").value)

        self.print_joints = bool(self.get_parameter("print_joints").value)
        # Basic validation
        if len(self.target_xyz_mm) != 3:
            raise ValueError("Parameter 'target_xyz_mm' must contain exactly 3 values.")
        if len(self.target_rpy_deg) != 3:
            raise ValueError("Parameter 'target_rpy_deg' must contain exactly 3 values.")
        if len(self.seed_joints_deg) != 6:
            raise ValueError("Parameter 'seed_joints_deg' must contain exactly 6 values.")
        if self.reference_frame not in ("base", "table"):
            raise ValueError("Parameter 'reference_frame' must be either 'base' or 'table'.")

        # Precompute converted values for logging
        self.target_xyz_m = [v / 1000.0 for v in self.target_xyz_mm]
        self.target_rpy_rad = [math.radians(v) for v in self.target_rpy_deg]
        self.seed_joints_rad = [math.radians(v) for v in self.seed_joints_deg]

        # Cache latest joint state
        self._last_js = None
        self.create_subscription(
            JointState,
            "/joint_states",
            self._js_cb,
            qos_profile_sensor_data,
        )

        # IK service client
        self.ik_client = self.create_client(GetPositionIK, "/compute_ik")

        # MoveIt2 joint execution
        self.moveit2 = MoveIt2(
            node=self,
            joint_names=UR5E_JOINTS,
            base_link_name="base_link",
            end_effector_name=self.ik_link_name,
            group_name=self.group_name,
        )
        self.moveit2.max_velocity = self.max_velocity_scale
        self.moveit2.max_acceleration = self.max_acceleration_scale

        self._started = False
        self._finished = False
        self._exit_code = 0

        self._print_summary()
        self.create_timer(0.1, self._run_once)

    def _js_cb(self, msg: JointState):
        self._last_js = msg

    def _print_summary(self):
        self.get_logger().info("=" * 70)
        self.get_logger().info(f"Step name              : {self.step_name}")
        self.get_logger().info(f"Reference frame        : {self.reference_frame}")
        self.get_logger().info(f"Target XYZ [mm]        : {self.target_xyz_mm}")
        self.get_logger().info(f"Target XYZ [m]         : {[round(v, 4) for v in self.target_xyz_m]}")
        self.get_logger().info(f"Target RPY [deg]       : {self.target_rpy_deg}")
        self.get_logger().info(f"Target RPY [rad]       : {[round(v, 4) for v in self.target_rpy_rad]}")
        self.get_logger().info(f"Group name             : {self.group_name}")
        self.get_logger().info(f"IK link name           : {self.ik_link_name}")
        self.get_logger().info(f"Use seed joints        : {self.use_seed_joints}")
        self.get_logger().info(f"Seed from joint_states : {self.seed_from_joint_states}")
        self.get_logger().info(f"Seed joints [deg]      : {self.seed_joints_deg}")
        self.get_logger().info(f"Seed joints [rad]      : {[round(v, 4) for v in self.seed_joints_rad]}")
        self.get_logger().info(f"Execute                : {self.execute_motion}")
        self.get_logger().info(f"Velocity scale         : {self.max_velocity_scale}")
        self.get_logger().info(f"Acceleration scale     : {self.max_acceleration_scale}")
        self.get_logger().info("=" * 70)

    def _convert_input_pose_to_base(self, xyz_m, rpy_rad):
        if self.reference_frame == "base":
            return list(xyz_m), list(rpy_rad)

        # table -> base_link
        x, y, z = xyz_m
        roll, pitch, yaw = rpy_rad

        yaw_offset = math.radians(self.table_frame_yaw_offset_deg)

        xyz_base = [-x, -y, z]
        rpy_base = [roll, pitch, yaw + yaw_offset]

        return xyz_base, rpy_base

    def _finish_success(self):
        self._exit_code = 0
        self._finished = True
        self.get_logger().info(f"Finished step successfully: {self.step_name}")

    def _finish_error(self, msg: str):
        self._exit_code = 1
        self._finished = True
        self.get_logger().error(msg)

    def _run_once(self):
        if self._started or self._finished:
            return
        self._started = True

        # 1) Wait for IK service
        if not self.ik_client.wait_for_service(timeout_sec=5.0):
            self._finish_error("Service /compute_ik not available. Start MoveIt first.")
            return

        # 2) Convert package convention -> base_link pose
        target_xyz_base, target_rpy_base = self._convert_input_pose_to_base(
            self.target_xyz_m,
            self.target_rpy_rad,
        )

        roll, pitch, yaw = target_rpy_base
        qx, qy, qz, qw = quat_from_rpy_zyx(roll, pitch, yaw)

        pose = PoseStamped()
        pose.header.frame_id = "base_link"
        pose.header.stamp = self.get_clock().now().to_msg()

        pose.pose.position.x = float(target_xyz_base[0])
        pose.pose.position.y = float(target_xyz_base[1])
        pose.pose.position.z = float(target_xyz_base[2])

        pose.pose.orientation.x = qx
        pose.pose.orientation.y = qy
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw

        self.get_logger().info(
            f"Pose goal input ({self.reference_frame} frame): "
            f"xyz_m={self.target_xyz_m}, rpy_rad={self.target_rpy_rad}"
        )
        self.get_logger().info(
            f"Pose goal in base_link: "
            f"xyz={target_xyz_base}, rpy={target_rpy_base}, "
            f"quat_xyzw={[round(qx, 5), round(qy, 5), round(qz, 5), round(qw, 5)]}"
        )

        # 3) Build IK request
        req = GetPositionIK.Request()
        req.ik_request.group_name = self.group_name
        req.ik_request.ik_link_name = self.ik_link_name
        req.ik_request.pose_stamped = pose
        req.ik_request.timeout.sec = int(self.ik_timeout)
        req.ik_request.timeout.nanosec = int((self.ik_timeout - int(self.ik_timeout)) * 1e9)

        # 4) Choose IK seed
        seed_positions = list(self.seed_joints_rad)

        if self.seed_from_joint_states and self._last_js is not None:
            name_to_pos = dict(zip(self._last_js.name, self._last_js.position))
            if all(j in name_to_pos for j in UR5E_JOINTS):
                seed_positions = [float(name_to_pos[j]) for j in UR5E_JOINTS]
                if self.print_joints:
                    self.get_logger().info("Using /joint_states as IK seed (UR5e order).")
            else:
                self.get_logger().warn(
                    "/joint_states received but does not contain all UR5e joints. "
                    "Falling back to configured seed."
                )
        elif not self.use_seed_joints:
            self.get_logger().warn(
                "use_seed_joints:=false but no alternative seed mechanism is defined. "
                "Using configured seed_joints_deg anyway."
            )

        seed = JointState()
        seed.header = pose.header
        seed.name = UR5E_JOINTS
        seed.position = seed_positions
        req.ik_request.robot_state.joint_state = seed

        # 5) Call IK asynchronously
        future = self.ik_client.call_async(req)
        future.add_done_callback(self._on_ik)

    def _on_ik(self, future):
        if self._finished:
            return

        try:
            res = future.result()
        except Exception as e:
            self._finish_error(f"IK service call failed: {e}")
            return

        if res.error_code.val != res.error_code.SUCCESS:
            self._finish_error(f"IK failed, error code: {res.error_code.val}")
            return

        sol = res.solution.joint_state
        name_to_pos = {n: p for n, p in zip(sol.name, sol.position)}

        try:
            joint_goal = [float(name_to_pos[j]) for j in UR5E_JOINTS]
        except KeyError as e:
            self._finish_error(f"IK solution missing expected joint: {e}")
            return

        if self.print_joints:
            self.get_logger().info("IK joint goal (UR5e order):")
            for n, v in zip(UR5E_JOINTS, joint_goal):
                self.get_logger().info(f"  {n}: {v:.4f} rad")

        if not self.execute_motion:
            self.get_logger().info("execute:=false -> exiting without motion.")
            self._finish_success()
            return

        try:
            self.get_logger().info("Executing IK joint goal via MoveIt2...")
            self.moveit2.move_to_configuration(joint_goal)
            self.moveit2.wait_until_executed()
            self.get_logger().info("Execution finished.")
            self._finish_success()
        except Exception as e:
            self._finish_error(f"MoveIt execution failed: {e}")


def run_pose_motion(config: Dict[str, Any]) -> int:
    """
    Reusable function to execute one social pose motion from Python code.
    Returns the process-style exit code: 0 success, 1 error, 130 interrupted.
    """
    node = UR5eMoveToPoseViaIK(config=config)

    try:
        while rclpy.ok() and not node._finished:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        node.get_logger().warn("Interrupted by user.")
        node._exit_code = 130
    finally:
        node.destroy_node()

    return node._exit_code


def main():
    rclpy.init()
    exit_code = 1

    try:
        node = UR5eMoveToPoseViaIK()
        while rclpy.ok() and not node._finished:
            rclpy.spin_once(node, timeout_sec=0.1)
        exit_code = node._exit_code
        node.destroy_node()
    except KeyboardInterrupt:
        exit_code = 130
    finally:
        if rclpy.ok():
            rclpy.shutdown()

    sys.exit(exit_code)


if __name__ == "__main__":
    main()