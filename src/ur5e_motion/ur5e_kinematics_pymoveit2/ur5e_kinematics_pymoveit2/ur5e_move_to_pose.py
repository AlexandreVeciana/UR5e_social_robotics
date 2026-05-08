#!/usr/bin/env python3
import math
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

    Seed strategy:
      - If seed_from_joint_states:=true and /joint_states is available,
        use the current robot joint state as IK seed.
      - Otherwise use seed_joints parameter.

    Pose convention:
      - pose_input_frame = "base":
          target_xyz and target_rpy are already expressed in ROS base_link frame.
      - pose_input_frame = "table":
          target_xyz and target_rpy are expressed in an intuitive table/workspace
          frame, assuming the robot base frame is rotated 180 deg around Z with
          respect to that table frame.

          Conversion applied:
            x_base = -x_table
            y_base = -y_table
            z_base =  z_table
            yaw_base = yaw_table + table_frame_yaw_offset_deg
    """

    def __init__(self):
        super().__init__("ur5e_move_to_pose")

        # --- Pose target
        self.declare_parameter("target_xyz", [0.35, 0.00, 0.45])
        self.declare_parameter("target_rpy", [0.0, 0.0, 0.0])  # roll,pitch,yaw [rad]

        # --- Pose input convention
        self.declare_parameter("pose_input_frame", "base")   # "base" or "table"
        self.declare_parameter("table_frame_yaw_offset_deg", 180.0)

        # --- IK settings
        self.declare_parameter("group_name", "ur_manipulator")
        self.declare_parameter("ik_link", "tool0")
        self.declare_parameter(
            "seed_joints",
            [0.0, -math.pi / 2.0, math.pi / 2.0, 0.0, math.pi / 2.0, 0.0],
        )
        self.declare_parameter("ik_timeout_sec", 0.2)

        # --- Motion settings
        self.declare_parameter("max_velocity", 0.3)
        self.declare_parameter("max_acceleration", 0.3)
        self.declare_parameter("execute", True)

        # --- Debug
        self.declare_parameter("print_joints", False)

        # --- Seed source
        self.declare_parameter("seed_from_joint_states", True)

        # Read params
        self.target_xyz = [float(x) for x in self.get_parameter("target_xyz").value]
        self.target_rpy = [float(x) for x in self.get_parameter("target_rpy").value]

        self.pose_input_frame = str(self.get_parameter("pose_input_frame").value).strip().lower()
        self.table_frame_yaw_offset_deg = float(
            self.get_parameter("table_frame_yaw_offset_deg").value
        )

        self.group_name = str(self.get_parameter("group_name").value)
        self.ik_link = str(self.get_parameter("ik_link").value)
        self.seed_joints = [float(x) for x in self.get_parameter("seed_joints").value]
        self.ik_timeout = float(self.get_parameter("ik_timeout_sec").value)

        self.max_velocity = float(self.get_parameter("max_velocity").value)
        self.max_acceleration = float(self.get_parameter("max_acceleration").value)
        self.execute_motion = bool(self.get_parameter("execute").value)
        self.print_joints = bool(self.get_parameter("print_joints").value)

        self.seed_from_joint_states = bool(self.get_parameter("seed_from_joint_states").value)

        # Basic validation
        if len(self.target_xyz) != 3:
            raise ValueError("Parameter 'target_xyz' must contain exactly 3 values.")
        if len(self.target_rpy) != 3:
            raise ValueError("Parameter 'target_rpy' must contain exactly 3 values.")
        if len(self.seed_joints) != 6:
            raise ValueError("Parameter 'seed_joints' must contain exactly 6 values.")
        if self.pose_input_frame not in ("base", "table"):
            raise ValueError(
                "Parameter 'pose_input_frame' must be either 'base' or 'table'."
            )

        # Cache the latest joint state
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
            end_effector_name=self.ik_link,
            group_name=self.group_name,
        )
        self.moveit2.max_velocity = self.max_velocity
        self.moveit2.max_acceleration = self.max_acceleration

        self._done = False
        self.create_timer(0.1, self._run_once)

    def _js_cb(self, msg: JointState):
        self._last_js = msg

    def _convert_input_pose_to_base(self, xyz, rpy):
        """
        Convert input pose convention to ROS base_link convention.

        Supported input conventions:
          - "base": pose is already expressed in base_link
          - "table": pose is expressed in an intuitive table frame where the
                     UR base frame is rotated around Z with respect to the table

        For "table":
          x_base = -x_table
          y_base = -y_table
          z_base =  z_table
          yaw_base = yaw_table + offset
        """
        if self.pose_input_frame == "base":
            return list(xyz), list(rpy)

        # table -> base_link
        x, y, z = xyz
        roll, pitch, yaw = rpy

        yaw_offset = math.radians(self.table_frame_yaw_offset_deg)

        xyz_base = [-x, -y, z]
        rpy_base = [roll, pitch, yaw + yaw_offset]

        return xyz_base, rpy_base

    def _run_once(self):
        if self._done:
            return
        self._done = True

        # 1) Wait for IK service
        if not self.ik_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("Service /compute_ik not available. Start MoveIt first.")
            rclpy.shutdown()
            return

        # 2) Convert input pose convention -> base_link pose
        target_xyz_base, target_rpy_base = self._convert_input_pose_to_base(
            self.target_xyz,
            self.target_rpy,
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
            f"Pose goal input ({self.pose_input_frame} frame): "
            f"xyz={self.target_xyz}, rpy={self.target_rpy}"
        )
        self.get_logger().info(
            f"Pose goal in base_link: "
            f"xyz={target_xyz_base}, rpy={target_rpy_base}, "
            f"quat_xyzw={[qx, qy, qz, qw]}"
        )

        # 3) Build IK request
        req = GetPositionIK.Request()
        req.ik_request.group_name = self.group_name
        req.ik_request.ik_link_name = self.ik_link
        req.ik_request.pose_stamped = pose
        req.ik_request.timeout.sec = int(self.ik_timeout)
        req.ik_request.timeout.nanosec = int(
            (self.ik_timeout - int(self.ik_timeout)) * 1e9
        )

        # Choose seed: joint_states (preferred) or configured seed_joints
        seed_positions = list(self.seed_joints)

        if self.seed_from_joint_states and self._last_js is not None:
            name_to_pos = dict(zip(self._last_js.name, self._last_js.position))
            if all(j in name_to_pos for j in UR5E_JOINTS):
                seed_positions = [float(name_to_pos[j]) for j in UR5E_JOINTS]
                if self.print_joints:
                    self.get_logger().info("Using /joint_states as IK seed (UR5e order).")
            else:
                self.get_logger().warn(
                    "/joint_states received but does not contain all UR5e joints. "
                    "Falling back to seed_joints."
                )

        seed = JointState()
        seed.header = pose.header
        seed.name = UR5E_JOINTS
        seed.position = seed_positions
        req.ik_request.robot_state.joint_state = seed

        # 4) Call IK asynchronously
        future = self.ik_client.call_async(req)
        future.add_done_callback(self._on_ik)

    def _on_ik(self, future):
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
        name_to_pos = {n: p for n, p in zip(sol.name, sol.position)}

        try:
            joint_goal = [float(name_to_pos[j]) for j in UR5E_JOINTS]
        except KeyError as e:
            self.get_logger().error(f"IK solution missing expected joint: {e}")
            rclpy.shutdown()
            return

        if self.print_joints:
            self.get_logger().info("IK joint goal (UR5e order):")
            for n, v in zip(UR5E_JOINTS, joint_goal):
                self.get_logger().info(f"  {n}: {v:.4f} rad")

        if not self.execute_motion:
            self.get_logger().info("execute:=false -> exiting without motion.")
            rclpy.shutdown()
            return

        self.get_logger().info("Executing IK joint goal via MoveIt2...")
        self.moveit2.move_to_configuration(joint_goal)
        self.moveit2.wait_until_executed()
        self.get_logger().info("Execution finished.")

        rclpy.shutdown()


def main():
    rclpy.init()
    node = UR5eMoveToPoseViaIK()
    rclpy.spin(node)


if __name__ == "__main__":
    main()