#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration

from sensor_msgs.msg import JointState
from geometry_msgs.msg import TransformStamped, PoseStamped

import tf2_ros

from pymoveit2 import MoveIt2
from moveit_msgs.srv import GetPositionFK


def euler_from_quaternion_xyzw(qx, qy, qz, qw):
    # roll (x-axis rotation)
    sinr_cosp = 2.0 * (qw * qx + qy * qz)
    cosr_cosp = 1.0 - 2.0 * (qx * qx + qy * qy)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    # pitch (y-axis rotation)
    sinp = 2.0 * (qw * qy - qz * qx)
    if abs(sinp) >= 1.0:
        pitch = math.copysign(math.pi / 2.0, sinp)
    else:
        pitch = math.asin(sinp)

    # yaw (z-axis rotation)
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw


def wrap_to_pi(a):
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


class UR5eMoveJoints(Node):
    def __init__(self):
        super().__init__("ur5e_move_joints")

        # Parameters
        self.declare_parameter("joints", [0.0, -1.57, 1.57, 0.0, 1.57, 0.0])
        self.declare_parameter("group_name", "ur_manipulator")
        self.declare_parameter("execute", True)
        self.declare_parameter("max_velocity", 0.3)
        self.declare_parameter("max_acceleration", 0.3)
        self.declare_parameter(
            "follow_joint_traj_action",
            "/joint_trajectory_controller/follow_joint_trajectory",
        )
        self.declare_parameter("wait_joint_states_sec", 10.0)

        # TF parameters
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("ee_frame", "tool0")
        self.declare_parameter("wait_tf_sec", 8.0)

        # FK parameters
        self.declare_parameter("fk_service", "/compute_fk")
        self.declare_parameter("wait_fk_service_sec", 8.0)

        self.joints = [float(v) for v in self.get_parameter("joints").value]
        self.group_name = str(self.get_parameter("group_name").value)
        self.execute = bool(self.get_parameter("execute").value)
        self.max_velocity = float(self.get_parameter("max_velocity").value)
        self.max_acceleration = float(self.get_parameter("max_acceleration").value)
        self.fjt_action = str(self.get_parameter("follow_joint_traj_action").value)
        self.wait_js = float(self.get_parameter("wait_joint_states_sec").value)

        self.base_frame = str(self.get_parameter("base_frame").value)
        self.ee_frame = str(self.get_parameter("ee_frame").value)
        self.wait_tf_sec = float(self.get_parameter("wait_tf_sec").value)

        self.fk_service_name = str(self.get_parameter("fk_service").value)
        self.wait_fk_service_sec = float(self.get_parameter("wait_fk_service_sec").value)

        # Joint states gate
        self._have_js = False
        self.create_subscription(JointState, "/joint_states", self._js_cb, 10)

        # TF listener
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        # MoveIt2 interface (execution)
        self.joint_names = [
            "shoulder_pan_joint",
            "shoulder_lift_joint",
            "elbow_joint",
            "wrist_1_joint",
            "wrist_2_joint",
            "wrist_3_joint",
        ]

        self.moveit2 = MoveIt2(
            node=self,
            joint_names=self.joint_names,
            base_link_name=self.base_frame,
            end_effector_name=self.ee_frame,
            group_name=self.group_name,
        )
        self.moveit2.max_velocity = self.max_velocity
        self.moveit2.max_acceleration = self.max_acceleration

        # FK service client
        self._fk_client = self.create_client(GetPositionFK, self.fk_service_name)

    def _js_cb(self, _msg: JointState):
        self._have_js = True

    # -------- TF obtained pose (after motion) --------
    def _get_ee_pose_tf(self):
        """
        Returns:
          p = (x, y, z)
          q_xyzw = (qx, qy, qz, qw)
          rpy = (roll, pitch, yaw) [rad]
        """
        # Let TF callbacks populate the buffer
        rclpy.spin_once(self, timeout_sec=0.05)

        tf: TransformStamped = self._tf_buffer.lookup_transform(
            self.base_frame,                 # target
            self.ee_frame,                   # source
            rclpy.time.Time(),               # latest
            timeout=Duration(seconds=self.wait_tf_sec),
        )

        tr = tf.transform.translation
        rot = tf.transform.rotation

        p = (float(tr.x), float(tr.y), float(tr.z))
        q_xyzw = (float(rot.x), float(rot.y), float(rot.z), float(rot.w))
        rpy = euler_from_quaternion_xyzw(*q_xyzw)
        return p, q_xyzw, (float(rpy[0]), float(rpy[1]), float(rpy[2]))

    # -------- MoveIt FK (model pose from joints) --------
    def _get_fk_pose_model(self, joint_positions):
        """
        Calls /compute_fk with the provided joint positions (ordered like self.joint_names).
        Returns:
          p = (x, y, z)
          q_xyzw = (qx, qy, qz, qw)
          rpy = (roll, pitch, yaw) [rad]
        """
        if not self._fk_client.wait_for_service(timeout_sec=self.wait_fk_service_sec):
            raise TimeoutError(f"FK service not available: {self.fk_service_name}")

        req = GetPositionFK.Request()
        req.header.frame_id = self.base_frame
        req.fk_link_names = [self.ee_frame]  # compute pose of this link

        js = JointState()
        js.name = list(self.joint_names)
        js.position = [float(x) for x in joint_positions]
        req.robot_state.joint_state = js

        future = self._fk_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=self.wait_fk_service_sec)

        if not future.done():
            raise TimeoutError("FK service call timed out")

        res = future.result()
        if res is None:
            raise RuntimeError("FK service returned None")

        if res.error_code.val != res.error_code.SUCCESS:
            raise RuntimeError(f"FK failed, error code: {res.error_code.val}")

        if not res.pose_stamped:
            raise RuntimeError("FK returned empty pose_stamped list")

        pose: PoseStamped = res.pose_stamped[0]
        p = (
            float(pose.pose.position.x),
            float(pose.pose.position.y),
            float(pose.pose.position.z),
        )
        q_xyzw = (
            float(pose.pose.orientation.x),
            float(pose.pose.orientation.y),
            float(pose.pose.orientation.z),
            float(pose.pose.orientation.w),
        )
        rpy = euler_from_quaternion_xyzw(*q_xyzw)
        return p, q_xyzw, (float(rpy[0]), float(rpy[1]), float(rpy[2]))

    def _log_pose(self, label, p, q, rpy):
        self.get_logger().info(
            f"{label}: p=[{p[0]:.4f}, {p[1]:.4f}, {p[2]:.4f}] m, "
            f"q(xyzw)=[{q[0]:.5f}, {q[1]:.5f}, {q[2]:.5f}, {q[3]:.5f}], "
            f"rpy=[{rpy[0]:.4f}, {rpy[1]:.4f}, {rpy[2]:.4f}] rad"
        )

    def _log_compare(self, label, model_pose, obtained_pose):
        (pm, _, rpym) = model_pose
        (po, _, rpyo) = obtained_pose

        dx = po[0] - pm[0]
        dy = po[1] - pm[1]
        dz = po[2] - pm[2]
        dpos = math.sqrt(dx * dx + dy * dy + dz * dz)

        dr = wrap_to_pi(rpyo[0] - rpym[0])
        dp = wrap_to_pi(rpyo[1] - rpym[1])
        dyaw = wrap_to_pi(rpyo[2] - rpym[2])

        self.get_logger().info(
            f"{label} ERROR (obtained - model): "
            f"dp=[{dx:.4f}, {dy:.4f}, {dz:.4f}] m (|dp|={dpos:.4f} m), "
            f"drpy=[{dr:.4f}, {dp:.4f}, {dyaw:.4f}] rad"
        )

    def run_once(self):
        self.get_logger().info(f"Target joints (rad): {self.joints}")

        # 1) Wait for joint states
        t0 = self.get_clock().now()
        while rclpy.ok() and not self._have_js:
            if (self.get_clock().now() - t0).nanoseconds * 1e-9 > self.wait_js:
                self.get_logger().error("Timed out waiting for /joint_states.")
                return 1
            rclpy.spin_once(self, timeout_sec=0.1)
        self.get_logger().info("Joint states are available now.")

        # 2) Model FK for target joints
        try:
            fk_model_target = self._get_fk_pose_model(self.joints)
            self._log_pose("MODEL FK (target joints)", *fk_model_target)
        except Exception as e:
            self.get_logger().warn(f"Could not compute MODEL FK: {e}")
            fk_model_target = None

        # 3) Obtained pose BEFORE (TF)
        try:
            tf_before = self._get_ee_pose_tf()
            self._log_pose("OBTAINED TF (BEFORE)", *tf_before)
            if fk_model_target is not None:
                self._log_compare("BEFORE vs MODEL", fk_model_target, tf_before)
        except Exception as e:
            self.get_logger().warn(f"BEFORE Could not read EE pose from TF: {e}")

        if not self.execute:
            self.get_logger().info("execute:=false -> exiting without motion.")
            return 0

        # 4) Execute motion
        self.get_logger().info("Sending joint configuration to MoveIt2...")
        try:
            self.moveit2.move_to_configuration(self.joints)
            self.moveit2.wait_until_executed()
        except KeyboardInterrupt:
            self.get_logger().warn("Interrupted by user (Ctrl+C).")
            return 130

        self.get_logger().info("Motion finished.")

        # 5) Obtained pose AFTER (TF) + compare to MODEL FK
        try:
            tf_after = self._get_ee_pose_tf()
            self._log_pose("OBTAINED TF (AFTER)", *tf_after)
            if fk_model_target is not None:
                self._log_compare("AFTER vs MODEL", fk_model_target, tf_after)
        except Exception as e:
            self.get_logger().warn(f"AFTER Could not read EE pose from TF: {e}")

        return 0


def main(args=None):
    rclpy.init(args=args)
    node = UR5eMoveJoints()
    try:
        rc = node.run_once()
    finally:
        node.destroy_node()
        rclpy.shutdown()
    raise SystemExit(rc)


if __name__ == "__main__":
    main()
