"""Bring up the YDLIDAR X2/X2L, publishing /scan.

    ros2 launch arm_lidar x2.launch.py

Requires tools/fetch_ydlidar_driver.sh to have been run on this machine and
tools/99-ydlidar.rules installed (see docs/LIDAR.md), or /dev/ydlidar will
not exist and the node will fault on startup.
"""
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    params = get_package_share_directory('arm_lidar') + '/config/x2.yaml'
    return LaunchDescription([
        Node(
            package='ydlidar_ros2_driver',
            executable='ydlidar_ros2_driver_node',
            name='ydlidar_ros2_driver_node',
            output='screen',
            emulate_tty=True,
            parameters=[params],
        ),
    ])
