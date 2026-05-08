#!/usr/bin/env python3
import os
import yaml
import math

from launch import LaunchDescription
from launch.actions import OpaqueFunction, RegisterEventHandler, TimerAction, LogInfo
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory


def seed_description(step_dict):
    """
    Describe how IK seed is provided for this step.
    """
    has_seed_joints = "seed_joints" in step_dict
    seed_from_js = step_dict.get("seed_from_joint_states", True)

    if has_seed_joints and (seed_from_js is False):
        return "manual seed_joints"
    if has_seed_joints and (seed_from_js is True):
        return "joint_states preferred (seed_joints also provided)"
    if (not has_seed_joints) and seed_from_js:
        return "joint_states"
    return "no seed"


def build_sequence(context, *args, **kwargs):
    pkg_share = get_package_share_directory("ur5e_kinematics_pymoveit2")
    cfg_path = os.path.join(pkg_share, "config", "ur5e_hand_shake.yaml")

    with open(cfg_path, "r") as f:
        data = yaml.safe_load(f)

    common = data["common"]
    steps = data["steps"]

    pose_input_frame = common.get("pose_input_frame", "base")

    nodes = []

    for step in steps:
        step_name = step["name"]

        # YAML units:
        #   target_xyz -> mm
        #   target_rpy -> deg
        xyz = [float(v) / 1000.0 for v in step["target_xyz"]]
        rpy = [math.radians(float(v)) for v in step["target_rpy"]]

        params = {
            **common,
            "target_xyz": xyz,
            "target_rpy": rpy,
        }

        if "seed_from_joint_states" in step:
            params["seed_from_joint_states"] = step["seed_from_joint_states"]

        if "seed_joints" in step:
            params["seed_joints"] = [math.radians(float(v)) for v in step["seed_joints"]]

        node = Node(
            package="ur5e_kinematics_pymoveit2",
            executable="ur5e_move_to_pose_exe",
            name=f"ur5e_move_{step_name}",
            output="screen",
            parameters=[params],
        )

        nodes.append(
            (
                step_name,
                step,
                node,
                float(step.get("sleep_after", 0.5)),
            )
        )

    actions = []

    first_name, first_step, first_node, _ = nodes[0]
    actions.append(
        LogInfo(
            msg=(
                f"\n=== STEP: {first_name} | "
                f"frame={pose_input_frame} | "
                f"seed={seed_description(first_step)} ==="
            )
        )
    )
    actions.append(first_node)

    for (cur_name, cur_step, cur_node, delay), (next_name, next_step, next_node, _) in zip(
        nodes[:-1], nodes[1:]
    ):
        actions.append(
            RegisterEventHandler(
                OnProcessExit(
                    target_action=cur_node,
                    on_exit=[
                        TimerAction(
                            period=delay,
                            actions=[
                                LogInfo(
                                    msg=(
                                        f"\n=== STEP: {next_name} | "
                                        f"frame={pose_input_frame} | "
                                        f"seed={seed_description(next_step)} ==="
                                    )
                                ),
                                next_node,
                            ],
                        )
                    ],
                )
            )
        )

    return actions


def generate_launch_description():
    return LaunchDescription([
        OpaqueFunction(function=build_sequence)
    ])