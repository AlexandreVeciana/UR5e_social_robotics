from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():

    # Frames / group
    group_name = DeclareLaunchArgument("group_name", default_value="ur_manipulator")
    base_frame = DeclareLaunchArgument("base_frame", default_value="base_link")
    ee_frame = DeclareLaunchArgument("ee_frame", default_value="tool0")

    # Target position
    target_xyz_mm = DeclareLaunchArgument(
        "target_xyz",
        default_value="[400.0, 0.0, 300.0]",
        description="Target position [mm] as [x,y,z]",
    )

    target_xyz_m = PythonExpression(
        [
            "[x/1000.0 for x in ",
            LaunchConfiguration("target_xyz"),
            "]"
        ]
    )
    # Target orientation (NOW IN DEGREES)
    target_rpy_deg = DeclareLaunchArgument(
        "target_rpy",
        default_value="[0.0, 180.0, 0.0]",
        description="Target orientation [deg] as [roll,pitch,yaw]",
    )

    # Convert deg → rad
    target_rpy_rad = PythonExpression(
        [
            "[x*3.141592653589793/180.0 for x in ",
            LaunchConfiguration("target_rpy"),
            "]"
        ]
    )

    # Quaternion mode (optional)
    use_quat = DeclareLaunchArgument("use_quat", default_value="false")

    target_quat_xyzw = DeclareLaunchArgument(
        "target_quat_xyzw",
        default_value="[0.0, 0.0, 0.0, 1.0]",
        description="Quaternion [qx,qy,qz,qw]",
    )

    # Seed joints (IN DEGREES)
    seed_joints_deg = DeclareLaunchArgument(
        "seed_joints",
        default_value="[0.0, -90.0, 90.0, 0.0, 90.0, 0.0]",
        description="IK seed joints [deg]",
    )

    seed_joints_rad = PythonExpression(
        [
            "[x*3.141592653589793/180.0 for x in ",
            LaunchConfiguration("seed_joints"),
            "]"
        ]
    )

    # Motion / runtime
    execute = DeclareLaunchArgument("execute", default_value="true")
    use_sim_time = DeclareLaunchArgument("use_sim_time", default_value="false")
    max_velocity = DeclareLaunchArgument("max_velocity", default_value="0.1")
    max_acceleration = DeclareLaunchArgument("max_acceleration", default_value="0.1")

    seed_from_joint_states = DeclareLaunchArgument(
        "seed_from_joint_states",
        default_value="true"
    )

    node = Node(
        package="ur5e_kinematics_pymoveit2",
        executable="ur5e_move_to_pose_exe",
        name="ur5e_move_to_pose",
        output="screen",
        parameters=[
            {
                "group_name": LaunchConfiguration("group_name"),
                "base_frame": LaunchConfiguration("base_frame"),
                "ee_frame": LaunchConfiguration("ee_frame"),
                "target_xyz": target_xyz_m,
                "target_rpy": target_rpy_rad,
                "use_quat": LaunchConfiguration("use_quat"),
                "target_quat_xyzw": LaunchConfiguration("target_quat_xyzw"),
                "seed_joints": seed_joints_rad,
                "seed_from_joint_states": LaunchConfiguration("seed_from_joint_states"),
                "execute": LaunchConfiguration("execute"),
                "use_sim_time": LaunchConfiguration("use_sim_time"),
                "max_velocity": LaunchConfiguration("max_velocity"),
                "max_acceleration": LaunchConfiguration("max_acceleration"),
            }
        ],
    )

    return LaunchDescription(
        [
            group_name,
            base_frame,
            ee_frame,
            target_xyz_mm,
            target_rpy_deg,
            use_quat,
            target_quat_xyzw,
            seed_joints_deg,
            seed_from_joint_states,
            execute,
            use_sim_time,
            max_velocity,
            max_acceleration,
            node,
        ]
    )