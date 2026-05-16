#!/usr/bin/env python3

import os
import yaml

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, RegisterEventHandler, LogInfo
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _load_yaml_file(yaml_file):
    with open(yaml_file, "r") as f:
        return yaml.safe_load(f)


def _build_step_node(step, index):
    step_name = step.get("step_name", f"step_{index}")
    target_xyz_mm = step.get("target_xyz_mm", [0.0, 0.0, 0.0])
    target_rpy_deg = step.get("target_rpy_deg", [0.0, 0.0, 0.0])

    seed_joints_deg = step.get("seed_joints_deg", [0.0, -90.0, 90.0, -90.0, -90.0, 0.0])
    use_seed_joints = step.get("use_seed_joints", True)
    seed_from_joint_states = step.get("seed_from_joint_states", True)

    execute = step.get("execute", True)
    max_velocity_scale = step.get("max_velocity_scale", 0.15)
    max_acceleration_scale = step.get("max_acceleration_scale", 0.15)
    print_joints = step.get("print_joints", True)

    return Node(
        package="ur5e_social_motion",
        executable="ur5e_move_to_pose_exe",
        name=f"ur5e_move_to_pose_{index}",
        output="screen",
        parameters=[
            {
                "step_name": step_name,
                "reference_frame": "table",
                "table_frame_yaw_offset_deg": 180.0,
                "target_xyz_mm": target_xyz_mm,
                "target_rpy_deg": target_rpy_deg,
                "seed_joints_deg": seed_joints_deg,
                "use_seed_joints": use_seed_joints,
                "seed_from_joint_states": seed_from_joint_states,
                "execute": execute,
                "group_name": "ur_manipulator",
                "ik_link_name": "tool0",
                "ik_timeout_sec": 0.2,
                "max_velocity_scale": max_velocity_scale,
                "max_acceleration_scale": max_acceleration_scale,
                "print_joints": print_joints,
            }
        ],
    )


def launch_setup(context, *args, **kwargs):
    yaml_file = LaunchConfiguration("motion_config").perform(context)

    if not os.path.isfile(yaml_file):
        raise FileNotFoundError(f"YAML config file not found: {yaml_file}")

    config = _load_yaml_file(yaml_file)
    steps = config.get("sequence", [])

    actions = []
    actions.append(LogInfo(msg=f"Loading motion sequence from: {yaml_file}"))
    actions.append(LogInfo(msg=f"Number of steps: {len(steps)}"))

    if not steps:
        actions.append(LogInfo(msg="No steps found in YAML file."))
        return actions

    nodes = [_build_step_node(step, i) for i, step in enumerate(steps)]

    for i, step in enumerate(steps):
        step_name = step.get("step_name", f"step_{i}")
        xyz_mm = step.get("target_xyz_mm", [0.0, 0.0, 0.0])
        rpy_deg = step.get("target_rpy_deg", [0.0, 0.0, 0.0])

        actions.append(
            LogInfo(
                msg=(
                    f"Step {i + 1}: {step_name} | "
                    f"XYZ [mm]={xyz_mm} | RPY [deg]={rpy_deg}"
                )
            )
        )

    actions.append(nodes[0])

    for i in range(len(nodes) - 1):
        current_node = nodes[i]
        next_node = nodes[i + 1]
        next_name = steps[i + 1].get("step_name", f"step_{i+1}")

        actions.append(
            RegisterEventHandler(
                OnProcessExit(
                    target_action=current_node,
                    on_exit=[
                        LogInfo(msg=f"Previous step finished. Launching next step: {next_name}"),
                        next_node,
                    ],
                )
            )
        )

    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "motion_config",
            default_value="",
            description="Absolute path to the YAML motion sequence file",
        ),
        OpaqueFunction(function=launch_setup),
    ])