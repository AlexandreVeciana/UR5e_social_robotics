from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    sequence_file = LaunchConfiguration("sequence_file")

    sequence_path = PathJoinSubstitution([
        FindPackageShare("ur5e_kinematics_pymoveit2"),
        "config",
        sequence_file,
    ])

    sequence_arg = DeclareLaunchArgument(
        "sequence_file",
        default_value="handshake.yaml",
        description="YAML sequence file",
    )

    static_table_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_base_link_to_table",
        output="screen",
        arguments=[
            "0.0", "0.0", "0.0",
            "0.0", "0.0", "1.0", "0.0",
            "base_link",
            "table",
        ],
    )

    sequence_node = Node(
        package="ur5e_kinematics_pymoveit2",
        executable="ur5e_pose_sequence_simple_exe",
        name="ur5e_pose_sequence",
        output="screen",
        parameters=[{
            "sequence_file": sequence_path,
        }],
    )

    return LaunchDescription([
        sequence_arg,
        static_table_tf,
        sequence_node,
    ])