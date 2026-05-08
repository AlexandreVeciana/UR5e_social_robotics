from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    target_xyz = DeclareLaunchArgument("target_xyz", default_value="[0.0, -0.4, 0.5]")
    target_rpy = DeclareLaunchArgument("target_rpy", default_value="[1.57, 0.0, 0.0]")
    seed_joints = DeclareLaunchArgument(
        "seed_joints",
        default_value="[-1.5, -1.7, 2.2, 1.5, -1.0, -3.14]",
        description="IK seed joint configuration [rad] in UR5e order",
    )

    group_name = DeclareLaunchArgument("group_name", default_value="ur_manipulator")
    ik_link = DeclareLaunchArgument("ik_link", default_value="tool0")
    execute = DeclareLaunchArgument("execute", default_value="false")
    use_sim_time = DeclareLaunchArgument("use_sim_time", default_value="false")

    max_velocity = DeclareLaunchArgument("max_velocity", default_value="0.1")
    max_acceleration = DeclareLaunchArgument("max_acceleration", default_value="0.1")

    node = Node(
        package="ur5e_kinematics_pymoveit2",
        executable="ur5e_inverse_kinematics_exe",
        name="ur5e_inverse_kinematics_node",
        output="screen",
        parameters=[
            {
                "target_xyz": LaunchConfiguration("target_xyz"),
                "target_rpy": LaunchConfiguration("target_rpy"),
                "seed_joints": LaunchConfiguration("seed_joints"),
                "group_name": LaunchConfiguration("group_name"),
                "ik_link": LaunchConfiguration("ik_link"),
                "execute": LaunchConfiguration("execute"),
                "use_sim_time": LaunchConfiguration("use_sim_time"),
                "max_velocity": LaunchConfiguration("max_velocity"),
                "max_acceleration": LaunchConfiguration("max_acceleration"),
            }
        ],
    )

    return LaunchDescription(
        [
            target_xyz,
            target_rpy,
            seed_joints,
            group_name,
            ik_link,
            execute,
            use_sim_time,
            max_velocity,
            max_acceleration,
            node,
        ]
    )