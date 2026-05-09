from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():

    joints = DeclareLaunchArgument(
        "joints",
        default_value="[0.0, -90.0, 90.0, -90.0, -90.0, 0.0]",
        description="Target joint configuration [deg] in UR5e order",
    )

    # ------------------------------------------------------------
    # Convert degrees -> radians
    # ------------------------------------------------------------
    joints_rad = PythonExpression([
        "[x*3.141592653589793/180.0 for x in ",
        LaunchConfiguration("joints"),
        "]"
    ])

    group_name = DeclareLaunchArgument(
        "group_name",
        default_value="ur_manipulator",
    )

    base_link = DeclareLaunchArgument(
        "base_link",
        default_value="base_link",
    )

    ee_link = DeclareLaunchArgument(
        "ee_link",
        default_value="tool0",
    )

    execute = DeclareLaunchArgument(
        "execute",
        default_value="true",
    )

    use_sim_time = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
    )

    max_velocity = DeclareLaunchArgument(
        "max_velocity",
        default_value="0.1",
    )

    max_acceleration = DeclareLaunchArgument(
        "max_acceleration",
        default_value="0.1",
    )

    node = Node(
        package="ur5e_kinematics_pymoveit2",
        executable="ur5e_forward_kinematics_exe",
        name="ur5e_move_joints",
        output="screen",
        parameters=[
            {
                "joints": joints_rad,
                "group_name": LaunchConfiguration("group_name"),
                "base_link": LaunchConfiguration("base_link"),
                "ee_link": LaunchConfiguration("ee_link"),
                "execute": LaunchConfiguration("execute"),
                "use_sim_time": LaunchConfiguration("use_sim_time"),
                "max_velocity": LaunchConfiguration("max_velocity"),
                "max_acceleration": LaunchConfiguration("max_acceleration"),
            }
        ],
    )

    return LaunchDescription(
        [
            joints,
            group_name,
            base_link,
            ee_link,
            execute,
            use_sim_time,
            max_velocity,
            max_acceleration,
            node,
        ]
    )