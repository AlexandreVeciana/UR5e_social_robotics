#!/usr/bin/env python3
import math
import time
import yaml

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.duration import Duration
from rclpy.action import ActionClient

from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint
from control_msgs.action import FollowJointTrajectory
from moveit_msgs.srv import GetPositionIK

from tf2_ros import Buffer, TransformListener
import tf2_geometry_msgs


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


def normalize_angle_near_reference(angle, reference):
    """
    Return an equivalent angle angle + 2*pi*k that is closest to reference.
    """
    return reference + math.atan2(
        math.sin(angle - reference),
        math.cos(angle - reference),
    )


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


class UR5ePoseSequenceSimple(Node):

    def __init__(self):
        super().__init__("ur5e_pose_sequence")

        self.declare_parameter("sequence_file", "")
        self.declare_parameter(
            "controller_action",
            "/scaled_joint_trajectory_controller/follow_joint_trajectory",
        )

        self.sequence_file = str(self.get_parameter("sequence_file").value)
        self.controller_action = str(self.get_parameter("controller_action").value)

        if not self.sequence_file:
            raise ValueError("Parameter 'sequence_file' is empty.")

        with open(self.sequence_file, "r") as f:
            self.data = yaml.safe_load(f)

        self.common = self.data.get("common", {})
        self.steps = self.data.get("steps", [])

        if not self.steps:
            raise ValueError("YAML file must contain a non-empty 'steps' list.")

        self.group_name = self.common.get("group_name", "ur_manipulator")
        self.ik_link = self.common.get("ik_link", "tool0")
        self.target_frame = self.common.get("target_frame", "table")
        self.planning_frame = self.common.get("planning_frame", "base_link")
        self.ik_timeout = float(self.common.get("ik_timeout_sec", 3.0))
        self.execute = bool(self.common.get("execute", True))
        self.print_joints = bool(self.common.get("print_joints", True))

        self.default_seed_from_joint_states = bool(
            self.common.get("seed_from_joint_states", True)
        )

        self.default_seed_joints = deg_to_rad_list(
            self.common.get(
                "seed_joints",
                [0.0, -90.0, 90.0, 0.0, 90.0, 0.0],
            )
        )

        self._last_js = None

        self.create_subscription(
            JointState,
            "/joint_states",
            self._js_cb,
            qos_profile_sensor_data,
        )

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.ik_client = self.create_client(GetPositionIK, "/compute_ik")

        self.traj_client = ActionClient(
            self,
            FollowJointTrajectory,
            self.controller_action,
        )

        self.step_index = 0
        self.waiting = False

        self.get_logger().info(f"Loaded sequence file: {self.sequence_file}")
        self.get_logger().info(f"Number of steps: {len(self.steps)}")
        self.get_logger().info(f"Controller action: {self.controller_action}")
        self.get_logger().info(f"Target frame: {self.target_frame}")
        self.get_logger().info(f"Planning frame: {self.planning_frame}")
        self.get_logger().info(f"IK link: {self.ik_link}")

        self.create_timer(0.5, self._timer_cb)

    def _js_cb(self, msg):
        self._last_js = msg

    def _timer_cb(self):
        if self.waiting:
            return

        if self.step_index >= len(self.steps):
            self.get_logger().info("Sequence finished.")
            rclpy.shutdown()
            return

        if not self.ik_client.service_is_ready():
            self.get_logger().warn("Waiting for /compute_ik...")
            self.ik_client.wait_for_service(timeout_sec=1.0)
            return

        if not self.traj_client.server_is_ready():
            self.get_logger().warn(
                f"Waiting for action server {self.controller_action}..."
            )
            self.traj_client.wait_for_server(timeout_sec=1.0)
            return

        self._start_step(self.step_index)

    def _start_step(self, index):
        self.waiting = True
        step = self.steps[index]
        name = step.get("name", f"step_{index + 1}")

        self.get_logger().info("=" * 60)
        self.get_logger().info(f"Step {index + 1}/{len(self.steps)}: {name}")

        xyz_m = mm_to_m_list(step["target_xyz"])
        rpy_rad = deg_to_rad_list(step["target_rpy"])

        self.get_logger().info(
            f"Pose input ({self.target_frame} frame): "
            f"xyz_mm={step['target_xyz']}, rpy_deg={step['target_rpy']}"
        )

        pose_target = self._build_pose(xyz_m, rpy_rad)
        pose_base = self._transform_pose_to_planning_frame(pose_target)

        if pose_base is None:
            self.get_logger().error(f"Stopping sequence at step: {name}")
            rclpy.shutdown()
            return

        seed_positions = self._get_seed_positions(step)

        req = self._build_ik_request(pose_base, seed_positions)

        future = self.ik_client.call_async(req)
        future.add_done_callback(
            lambda fut, step=step, name=name: self._on_ik_result(fut, step, name)
        )

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

    def _get_seed_positions(self, step):
        seed_from_joint_states = bool(
            step.get("seed_from_joint_states", self.default_seed_from_joint_states)
        )

        if "seed_joints" in step:
            fallback_seed = deg_to_rad_list(step["seed_joints"])
        else:
            fallback_seed = list(self.default_seed_joints)

        if seed_from_joint_states and self._last_js is not None:
            name_to_pos = dict(zip(self._last_js.name, self._last_js.position))
            if all(j in name_to_pos for j in UR5E_JOINTS):
                self.get_logger().info("Using /joint_states as IK seed.")
                return [float(name_to_pos[j]) for j in UR5E_JOINTS]

        self.get_logger().info("Using fallback/YAML seed_joints as IK seed.")
        return fallback_seed

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

    def _normalize_joint_goal_near_current_state(self, joint_goal):
        if self._last_js is None:
            self.get_logger().warn(
                "No /joint_states available for joint normalization. "
                "Using raw IK solution."
            )
            return joint_goal

        name_to_pos = dict(zip(self._last_js.name, self._last_js.position))

        if not all(j in name_to_pos for j in UR5E_JOINTS):
            self.get_logger().warn(
                "/joint_states does not contain all UR5e joints. "
                "Using raw IK solution."
            )
            return joint_goal

        current_joints = [float(name_to_pos[j]) for j in UR5E_JOINTS]

        normalized_goal = [
            normalize_angle_near_reference(goal, current)
            for goal, current in zip(joint_goal, current_joints)
        ]

        self.get_logger().info("Normalized IK joint goal near current state:")
        for joint_name, raw, norm, current in zip(
            UR5E_JOINTS,
            joint_goal,
            normalized_goal,
            current_joints,
        ):
            self.get_logger().info(
                f"  {joint_name}: raw={raw:.4f}, "
                f"normalized={norm:.4f}, "
                f"current={current:.4f}"
            )

        return normalized_goal

    def _on_ik_result(self, future, step, name):
        try:
            res = future.result()
        except Exception as e:
            self.get_logger().error(f"IK service call failed at step '{name}': {e}")
            rclpy.shutdown()
            return

        if res.error_code.val != res.error_code.SUCCESS:
            self.get_logger().error(
                f"IK failed at step '{name}', error code: {res.error_code.val}"
            )
            rclpy.shutdown()
            return

        sol = res.solution.joint_state
        name_to_pos = dict(zip(sol.name, sol.position))

        try:
            joint_goal = [float(name_to_pos[j]) for j in UR5E_JOINTS]
        except KeyError as e:
            self.get_logger().error(f"IK solution missing joint: {e}")
            rclpy.shutdown()
            return

        joint_goal = self._normalize_joint_goal_near_current_state(joint_goal)

        if self.print_joints:
            self.get_logger().info("Final IK joint goal:")
            for joint_name, value in zip(UR5E_JOINTS, joint_goal):
                self.get_logger().info(f"  {joint_name}: {value:.4f} rad")

        if not self.execute:
            self.get_logger().info("execute:=false -> not sending trajectory.")
            self._finish_step(step)
            return

        duration = float(step.get("duration", self.common.get("duration", 3.0)))
        self._send_trajectory(joint_goal, duration, step)

    def _send_trajectory(self, joint_goal, duration, step):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = UR5E_JOINTS
        goal.trajectory.header.stamp = self.get_clock().now().to_msg()

        point = JointTrajectoryPoint()
        point.positions = joint_goal
        point.velocities = [0.0] * len(UR5E_JOINTS)
        point.time_from_start.sec = int(duration)
        point.time_from_start.nanosec = int((duration - int(duration)) * 1e9)

        goal.trajectory.points.append(point)

        self.get_logger().info(f"Sending trajectory, duration={duration:.2f} s")

        future = self.traj_client.send_goal_async(goal)
        future.add_done_callback(
            lambda fut, step=step: self._on_goal_response(fut, step)
        )

    def _on_goal_response(self, future, step):
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().error("Trajectory goal rejected.")
            rclpy.shutdown()
            return

        self.get_logger().info("Trajectory goal accepted.")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            lambda fut, step=step: self._on_trajectory_done(fut, step)
        )

    def _on_trajectory_done(self, future, step):
        result = future.result().result
        error_code = result.error_code

        if error_code != FollowJointTrajectory.Result.SUCCESSFUL:
            self.get_logger().error(
                f"Trajectory execution failed. error_code={error_code}"
            )
            rclpy.shutdown()
            return

        self.get_logger().info("Trajectory execution finished.")
        self._finish_step(step)

    def _finish_step(self, step):
        sleep_after = float(step.get("sleep_after", 0.0))

        if sleep_after > 0.0:
            self.get_logger().info(f"Sleeping {sleep_after:.2f} s.")
            time.sleep(sleep_after)

        self.step_index += 1
        self.waiting = False


def main():
    rclpy.init()
    node = UR5ePoseSequenceSimple()
    rclpy.spin(node)


if __name__ == "__main__":
    main()