#!/usr/bin/env python3
"""
sequence_runner.py
──────────────────
Thin wrapper that runs a gesture YAML as a trajectory on the UR5e.

The previous version of this file dispatched steps one-by-one through
MoveIt2 (IK or joint-space via pymoveit2).  This version uses the same
direct JointTrajectory publisher mechanism as ur5e_kinematics_control,
which is simpler, more reliable, and does not require MoveIt2 to be running.

Public API
──────────
    run_motion_sequence(yaml_path, controller_topic, logger) -> int

YAML format (same as ur5e_kinematics_control):
──────────────────────────────────────────────
targets:
  - joints_deg: [-90.0, -90.0, -90.0, -180.0, -90.0, 90.0]
    time_sec: 3.0
  - joints_deg: [-45.0, -90.0, -90.0, -160.0, -90.0, 90.0]
    time_sec: 6.0
"""

from pathlib import Path
from typing import Optional

from ur5e_video_classification.ur5e_trajectory_runner import (
    run_trajectory,
    CONTROLLER_TOPIC_DEFAULT,
)


def run_motion_sequence(
    yaml_path: Path,
    controller_topic: str = CONTROLLER_TOPIC_DEFAULT,
    logger=None,
) -> int:
    """
    Execute the gesture defined by *yaml_path* on the UR5e.

    Publishes the full joint trajectory in a single message and blocks
    until the controller is expected to have finished.

    Parameters
    ----------
    yaml_path        : Path to the gesture YAML file (targets format)
    controller_topic : JointTrajectoryController topic to publish to
    logger           : optional ROS logger (node.get_logger()) for info/error

    Returns
    -------
    0 on success, non-zero on failure.
    """
    if logger:
        logger.info(f"[sequence_runner] Starting: {yaml_path.name}")

    exit_code = run_trajectory(yaml_path, controller_topic)

    if exit_code == 0:
        if logger:
            logger.info(f"[sequence_runner] Finished: {yaml_path.name}")
    else:
        if logger:
            logger.error(
                f"[sequence_runner] Failed: {yaml_path.name} "
                f"(exit code {exit_code})"
            )

    return exit_code