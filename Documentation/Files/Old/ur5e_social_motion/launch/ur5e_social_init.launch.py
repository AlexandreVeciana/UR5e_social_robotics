#!/usr/bin/env python3

import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = get_package_share_directory("ur5e_social_motion")

    motion_yaml = os.path.join(pkg_share, "config", "ur5e_social_init.yaml")
    sequence_launch = os.path.join(pkg_share, "launch", "ur5e_pose_sequence.launch.py")

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(sequence_launch),
            launch_arguments={
                "motion_config": motion_yaml
            }.items()
        )
    ])