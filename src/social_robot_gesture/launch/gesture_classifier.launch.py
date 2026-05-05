from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    input_topic = LaunchConfiguration('input_topic')
    output_topic = LaunchConfiguration('output_topic')

    declare_input_topic = DeclareLaunchArgument(
        'input_topic',
        default_value='/social_robot_perception/keypoints_json',
        description='Input topic with decoded body keypoints in JSON format.',
    )

    declare_output_topic = DeclareLaunchArgument(
        'output_topic',
        default_value='/social_robot_gesture/gesture',
        description='Output topic with final gesture label.',
    )

    config_file = PathJoinSubstitution(
        [FindPackageShare('social_robot_gesture'), 'config', 'gesture_thresholds.yaml']
    )

    gesture_classifier_node = Node(
        package='social_robot_gesture',
        executable='gesture_classifier_node',
        name='gesture_classifier_node',
        output='screen',
        parameters=[
            config_file,
            {
                'input_topic': input_topic,
                'output_topic': output_topic,
            },
        ],
    )

    return LaunchDescription([
        declare_input_topic,
        declare_output_topic,
        gesture_classifier_node,
    ])
