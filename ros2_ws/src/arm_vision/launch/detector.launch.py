"""Run the neural detector against the live camera.

    ros2 launch arm_vision detector.launch.py
    ros2 launch arm_vision detector.launch.py target_class:=cup

Needs arm_poc running for /camera/image_raw, and the model files in ~/models
(tools/fetch_detection_model.sh). Publishes /vision/object_point, which
vision_pick can be pointed at with its target_topic parameter.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('target_class', default_value='',
                              description='COCO class to hunt for, e.g. cup'),
        DeclareLaunchArgument('confidence', default_value='0.45'),
        DeclareLaunchArgument('publish_annotated', default_value='true'),
        Node(
            package='arm_vision', executable='detector', output='screen',
            parameters=[{
                'target_class': LaunchConfiguration('target_class'),
                'confidence': LaunchConfiguration('confidence'),
                'publish_annotated': LaunchConfiguration('publish_annotated'),
            }]),
    ])
