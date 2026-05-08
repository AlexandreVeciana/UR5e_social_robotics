from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():

    group_name = DeclareLaunchArgument(
        "group_name",
        default_value="ur_manipulator"
    )

    ik_link = DeclareLaunchArgument(
        "ik_link",
        default_value="tool0",
        description="IK link/end-effector link, e.g. tool0 or 2fg7_tcp"
    )

    target_xyz = DeclareLaunchArgument(
        "target_xyz",
        default_value="[400.0, 0.0, 300.0]",
        description="Target position in table frame [mm] as [x,y,z]"
    )

    target_xyz_m = PythonExpression([
        "[x/1000.0 for x in ",
        LaunchConfiguration("target_xyz"),
        "]"
    ])

    target_rpy = DeclareLaunchArgument(
        "target_rpy",
        default_value="[0.0, 180.0, 0.0]",
        description="Target orientation in table frame [deg] as [roll,pitch,yaw]"
    )

    target_rpy_rad = PythonExpression([
        "[x*3.141592653589793/180.0 for x in ",
        LaunchConfiguration("target_rpy"),
        "]"
    ])

    seed_joints = DeclareLaunchArgument(
        "seed_joints",
        default_value="[0.0, -90.0, 90.0, 0.0, 90.0, 0.0]",
        description="Fallback IK seed joints [deg]"
    )

    seed_joints_rad = PythonExpression([
        "[x*3.141592653589793/180.0 for x in ",
        LaunchConfiguration("seed_joints"),
        "]"
    ])

    seed_from_joint_states = DeclareLaunchArgument(
        "seed_from_joint_states",
        default_value="true",
        description="Use current /joint_states as IK seed when available"
    )

    execute = DeclareLaunchArgument("execute", default_value="true")
    use_sim_time = DeclareLaunchArgument("use_sim_time", default_value="false")
    max_velocity = DeclareLaunchArgument("max_velocity", default_value="0.1")
    max_acceleration = DeclareLaunchArgument("max_acceleration", default_value="0.1")
    ik_timeout_sec = DeclareLaunchArgument("ik_timeout_sec", default_value="0.2")
    print_joints = DeclareLaunchArgument("print_joints", default_value="true")

    # Static TF:
    #
    # base_link -> table
    #
    # Translation: zero
    # Rotation: 180 deg around Z
    #
    # Quaternion for yaw = pi:
    # qx = 0
    # qy = 0
    # qz = 1
    # qw = 0
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

    move_to_pose_node = Node(
        package="ur5e_kinematics_pymoveit2",
        executable="ur5e_move_to_pose_table_exe",
        name="ur5e_move_to_pose",
        output="screen",
        parameters=[{
            "group_name": LaunchConfiguration("group_name"),
            "ik_link": LaunchConfiguration("ik_link"),

            "target_frame": "table",
            "planning_frame": "base_link",

            "target_xyz": target_xyz_m,
            "target_rpy": target_rpy_rad,

            "seed_joints": seed_joints_rad,
            "seed_from_joint_states": LaunchConfiguration("seed_from_joint_states"),

            "execute": LaunchConfiguration("execute"),
            "use_sim_time": LaunchConfiguration("use_sim_time"),
            "max_velocity": LaunchConfiguration("max_velocity"),
            "max_acceleration": LaunchConfiguration("max_acceleration"),
            "ik_timeout_sec": LaunchConfiguration("ik_timeout_sec"),
            "print_joints": LaunchConfiguration("print_joints"),
        }],
    )

    return LaunchDescription([
        group_name,
        ik_link,

        target_xyz,
        target_rpy,

        seed_joints,
        seed_from_joint_states,

        execute,
        use_sim_time,
        max_velocity,
        max_acceleration,
        ik_timeout_sec,
        print_joints,

        static_table_tf,
        move_to_pose_node,
    ])