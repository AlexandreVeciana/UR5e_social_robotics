from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    trajectory_file = PathJoinSubstitution([
        FindPackageShare("ur5e_kinematics_control"),
        "config",
        LaunchConfiguration("trajectory_file")
    ])

    return LaunchDescription([

        DeclareLaunchArgument(
            "trajectory_file",
            default_value="trajectory.yaml",
            description="YAML filename inside config folder"
        ),

        DeclareLaunchArgument(
            "controller_topic",
            default_value="/joint_trajectory_controller/joint_trajectory"
        ),

        Node(
            package="ur5e_kinematics_control",
            executable="ur5e_joint_targets_exec",
            output="screen",
            parameters=[{
                "trajectory_file": trajectory_file,
                "controller_topic": LaunchConfiguration("controller_topic"),
            }],
        )
    ])