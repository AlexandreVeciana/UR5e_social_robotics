#!/usr/bin/env python3
import os
import yaml
import math

from launch import LaunchDescription
from launch.actions import OpaqueFunction, RegisterEventHandler, TimerAction, LogInfo
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory


def _fmt_list(vals, nd=3):
    """Pretty-print numeric lists (xyz, rpy, etc.)."""
    return "[" + ", ".join(f"{float(v):.{nd}f}" for v in vals) + "]"


def _seed_desc(step_dict):
    """
    Determine how the seed will be provided for this step:
      - manual: seed_joints present
      - from joint_states: seed_from_joint_states True (or default)
      - none/unknown: explicitly disabled and no seed_joints
    """
    has_seed_joints = "seed_joints" in step_dict

    # Default behavior if key missing: assume True (matches node default)
    seed_from_js = step_dict.get("seed_from_joint_states", True)

    if has_seed_joints and (seed_from_js is False):
        return f"manual seed_joints(deg)={_fmt_list(step_dict['seed_joints'], nd=2)}"

    if has_seed_joints and (seed_from_js is True):
        return "seed_from_joint_states=True (seed_joints also provided)"

    if (not has_seed_joints) and seed_from_js:
        return "seed_from_joint_states=True"

    return "seed disabled (no seed_joints, seed_from_joint_states=False)"


def _step_header(step_name, xyz_mm, rpy_deg, step_dict, pose_input_frame):
    return (
        "\n"
        "============================================================\n"
        f"STEP: {step_name}\n"
        f"  input frame: {pose_input_frame}\n"
        f"  xyz_mm: {_fmt_list(xyz_mm, nd=1)}\n"
        f"  rpy_deg: {_fmt_list(rpy_deg, nd=1)}\n"
        f"  seed: {_seed_desc(step_dict)}\n"
        "============================================================"
    )


def _build(context, *args, **kwargs):
    pkg_share = get_package_share_directory("ur5e_kinematics_pymoveit2")
    cfg_path = os.path.join(pkg_share, "config", "ur5e_pick_place.yaml")

    with open(cfg_path, "r") as f:
        data = yaml.safe_load(f)

    common = data["common"]
    steps = data["steps"]

    pose_input_frame = common.get("pose_input_frame", "base")

    nodes = []
    for s in steps:
        step_name = s["name"]

        # YAML convention:
        #   target_xyz  -> millimeters
        #   target_rpy  -> degrees
        xyz_mm = s["target_xyz"]
        rpy_deg = s["target_rpy"]

        # Convert only units here.
        # Frame conversion (table -> base_link) is handled INSIDE the node.
        xyz = [float(v) / 1000.0 for v in xyz_mm]          # mm -> m
        rpy = [math.radians(float(v)) for v in rpy_deg]    # deg -> rad

        params = {
            **common,
            "target_xyz": xyz,
            "target_rpy": rpy,
        }

        # Per-step optional params
        if "seed_from_joint_states" in s:
            params["seed_from_joint_states"] = s["seed_from_joint_states"]

        if "seed_joints" in s:
            # seed_joints are defined in YAML in degrees -> convert to radians
            params["seed_joints"] = [math.radians(float(v)) for v in s["seed_joints"]]

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
                xyz_mm,
                rpy_deg,
                s,
                node,
                float(s.get("sleep_after", 0.5)),
            )
        )

    actions = []

    # Start first step immediately
    step_name0, xyz0, rpy0, s0, node0, _ = nodes[0]
    actions.append(LogInfo(msg=_step_header(step_name0, xyz0, rpy0, s0, pose_input_frame)))
    actions.append(node0)

    # Chain the rest
    for (cur_name, cur_xyz, cur_rpy, cur_s, cur_node, dt), (
        nxt_name,
        nxt_xyz,
        nxt_rpy,
        nxt_s,
        nxt_node,
        _,
    ) in zip(nodes[:-1], nodes[1:]):
        actions.append(
            RegisterEventHandler(
                OnProcessExit(
                    target_action=cur_node,
                    on_exit=[
                        TimerAction(
                            period=dt,
                            actions=[
                                LogInfo(
                                    msg=_step_header(
                                        nxt_name, nxt_xyz, nxt_rpy, nxt_s, pose_input_frame
                                    )
                                ),
                                nxt_node,
                            ],
                        )
                    ],
                )
            )
        )

    return actions


def generate_launch_description():
    return LaunchDescription([OpaqueFunction(function=_build)])