from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
    LogInfo,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    # ── Mode arguments ────────────────────────────────────────────────────────

    use_fake_hardware_arg = DeclareLaunchArgument(
        'use_fake_hardware', default_value='false',
        description='Use simulated hardware instead of real robot')

    launch_rviz_arg = DeclareLaunchArgument(
        'launch_rviz', default_value='false',
        description='Launch RViz with MoveIt')

    # ── Robot arguments ───────────────────────────────────────────────────────

    robot_ip_arg = DeclareLaunchArgument(
        'robot_ip', default_value='192.168.1.4',
        description='IP address of the UR5e robot')

    ur_type_arg = DeclareLaunchArgument(
        'ur_type', default_value='ur5e',
        description='UR robot type')

    # ── Camera arguments ──────────────────────────────────────────────────────

    camera_topic_arg = DeclareLaunchArgument(
        'camera_topic', default_value='/camera/camera/color/image_raw',
        description='Camera topic the classification node subscribes to')

    # ── Classification node arguments ─────────────────────────────────────────

    confidence_threshold_arg = DeclareLaunchArgument(
        'confidence_threshold', default_value='0.9',
        description='Minimum CNN confidence to publish a prediction (0.0-1.0)')

    prediction_rate_arg = DeclareLaunchArgument(
        'prediction_rate', default_value='1.0',
        description='Inference rate in Hz')

    # ── Command node arguments ────────────────────────────────────────────────

    command_threshold_arg = DeclareLaunchArgument(
        'command_threshold', default_value='0.9',
        description='Minimum confidence for robot_command_node to dispatch a motion (0.0-1.0)')

    stability_frames_arg = DeclareLaunchArgument(
        'stability_frames', default_value='5',
        description='Consecutive identical predictions required before acting')

    command_cooldown_arg = DeclareLaunchArgument(
        'command_cooldown', default_value='3.0',
        description='Minimum seconds between two consecutive commands')

    ignore_no_action_arg = DeclareLaunchArgument(
        'ignore_no_action', default_value='true',
        description='Skip idle sequence when NoAction is predicted')

    # ── UR Driver (real hardware only) ────────────────────────────────────────

    ur_control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('ur_robot_driver'),
                'launch', 'ur_control.launch.py'
            ])
        ]),
        launch_arguments={
            'ur_type':            LaunchConfiguration('ur_type'),
            'robot_ip':           LaunchConfiguration('robot_ip'),
            'launch_rviz':        'false',
        }.items(),
        condition=UnlessCondition(LaunchConfiguration('use_fake_hardware')),
    )

    # ── MoveIt2 ───────────────────────────────────────────────────────────────
    # Delayed 5s on real hardware to give the driver time to connect.
    # No delay needed in simulation.

    moveit_real = TimerAction(
        period=5.0,
        actions=[
            LogInfo(msg='Starting MoveIt2...'),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    PathJoinSubstitution([
                        FindPackageShare('ur_moveit_config'),
                        'launch', 'ur_moveit.launch.py'
                    ])
                ]),
                launch_arguments={
                    'ur_type':            LaunchConfiguration('ur_type'),
                    'use_fake_hardware':  'false',
                    'launch_rviz':        LaunchConfiguration('launch_rviz'),
                }.items(),
            ),
        ],
        condition=UnlessCondition(LaunchConfiguration('use_fake_hardware')),
    )

    moveit_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('ur_moveit_config'),
                'launch', 'ur_moveit.launch.py'
            ])
        ]),
        launch_arguments={
            'ur_type':            LaunchConfiguration('ur_type'),
            'use_fake_hardware':  'true',
            'launch_rviz':        LaunchConfiguration('launch_rviz'),
        }.items(),
        condition=IfCondition(LaunchConfiguration('use_fake_hardware')),
    )

    # ── Camera ────────────────────────────────────────────────────────────────

    camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('realsense2_camera'),
                'launch', 'rs_launch.py'
            ])
        ]),
        launch_arguments={
            'enable_color': 'true',
            'enable_depth': 'true',
        }.items(),
    )

    # ── Classification node ───────────────────────────────────────────────────
    # Delayed 15s to allow MoveIt to fully load before predictions start
    # flowing and potentially triggering motion commands.

    classification_node = TimerAction(
        period=15.0,
        actions=[
            LogInfo(msg='Starting video classification node...'),
            Node(
                package='ur5e_video_classification',
                executable='video_classification_node',
                name='video_classification_node',
                output='screen',
                parameters=[{
                    'camera_topic':         LaunchConfiguration('camera_topic'),
                    'confidence_threshold': LaunchConfiguration('confidence_threshold'),
                    'prediction_rate':      LaunchConfiguration('prediction_rate'),
                }],
            ),
        ],
    )

    # ── Robot command node ────────────────────────────────────────────────────
    # Same 15s delay — must not start accepting predictions before
    # MoveIt's /compute_ik service is available.

    robot_command_node = TimerAction(
        period=15.0,
        actions=[
            LogInfo(msg='Starting robot command node...'),
            Node(
                package='ur5e_video_classification',
                executable='robot_command_node',
                name='robot_command_node',
                output='screen',
                parameters=[{
                    'confidence_threshold': LaunchConfiguration('command_threshold'),
                    'stability_frames':     LaunchConfiguration('stability_frames'),
                    'command_cooldown':     LaunchConfiguration('command_cooldown'),
                    'ignore_no_action':     LaunchConfiguration('ignore_no_action'),
                }],
            ),
        ],
    )

    return LaunchDescription([
        # Arguments
        use_fake_hardware_arg,
        launch_rviz_arg,
        robot_ip_arg,
        ur_type_arg,
        camera_topic_arg,
        confidence_threshold_arg,
        prediction_rate_arg,
        command_threshold_arg,
        stability_frames_arg,
        command_cooldown_arg,
        ignore_no_action_arg,
        # Infrastructure
        ur_control,
        moveit_real,
        moveit_sim,
        camera,
        # Application (delayed)
        classification_node,
        robot_command_node,
    ])