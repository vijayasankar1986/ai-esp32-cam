"""Bring up move_group, the state publisher and the execution bridge.

What this does NOT start is arm_poc. That node owns the serial port and the
camera, it is managed by its own service, and restarting it commands the arm
home. Start it the way you always have; this launch attaches to it.

    ros2 launch arm_moveit_config arm_moveit.launch.py            # plan only
    ros2 launch arm_moveit_config arm_moveit.launch.py execute:=true
    ros2 launch arm_moveit_config arm_moveit.launch.py rviz:=true

execute defaults to false on purpose. Without the bridge, MoveIt plans and
displays trajectories and nothing reaches a servo. With it, a successful plan
followed by Execute moves the arm.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    description_pkg = get_package_share_directory('arm_description')
    config_pkg = get_package_share_directory('arm_moveit_config')
    xacro_file = os.path.join(description_pkg, 'urdf', 'arm.urdf.xacro')

    moveit_config = (
        MoveItConfigsBuilder('arm', package_name='arm_moveit_config')
        .robot_description(file_path=xacro_file)
        .robot_description_semantic(file_path='config/arm.srdf')
        .robot_description_kinematics(file_path='config/kinematics.yaml')
        .joint_limits(file_path='config/joint_limits.yaml')
        .trajectory_execution(file_path='config/moveit_controllers.yaml')
        .planning_pipelines(pipelines=['ompl'])
        .to_moveit_configs()
    )

    robot_description = {
        'robot_description': ParameterValue(
            Command([FindExecutable(name='xacro'), ' ', xacro_file]),
            value_type=str)
    }

    return LaunchDescription([
        DeclareLaunchArgument(
            'execute', default_value='false',
            description='Start the bridge, so Execute moves the real arm'),
        DeclareLaunchArgument(
            'rviz', default_value='false',
            description='Start RViz with the motion planning panel'),
        DeclareLaunchArgument(
            'vision', default_value='false',
            description='Plan to whatever the camera detects'),
        DeclareLaunchArgument(
            'vision_execute', default_value='false',
            description='Let a detection move the arm, not just plan'),

        Node(
            package='robot_state_publisher', executable='robot_state_publisher',
            output='screen', parameters=[robot_description]),

        Node(
            package='moveit_ros_move_group', executable='move_group',
            output='screen', parameters=[moveit_config.to_dict()]),

        Node(
            package='arm_moveit_bridge', executable='bridge',
            output='screen',
            condition=IfCondition(LaunchConfiguration('execute'))),

        # Without the bridge nothing publishes /joint_states, and MoveIt needs
        # a start state to plan from. This stands in, driven by the model's
        # own zero pose rather than by anything the hardware reports.
        Node(
            package='joint_state_publisher', executable='joint_state_publisher',
            output='screen', parameters=[robot_description],
            condition=IfCondition(
                PythonNot(LaunchConfiguration('execute')))),

        # Camera-driven planning. Two switches, not one: starting it is not
        # the same as letting a detection move the arm, and the geometry it
        # depends on is unmeasured.
        Node(
            package='arm_moveit_bridge', executable='vision_pick',
            output='screen',
            parameters=[{'plan_only': PythonNot(
                LaunchConfiguration('vision_execute'))}],
            condition=IfCondition(LaunchConfiguration('vision'))),

        Node(
            package='rviz2', executable='rviz2', output='screen',
            arguments=['-d', os.path.join(config_pkg, 'config', 'moveit.rviz')],
            condition=IfCondition(LaunchConfiguration('rviz')),
            parameters=[
                moveit_config.robot_description,
                moveit_config.robot_description_semantic,
                moveit_config.robot_description_kinematics,
                moveit_config.planning_pipelines,
                moveit_config.joint_limits,
            ]),
    ])


def PythonNot(condition):
    """'not' for a launch substitution, which has no boolean operators."""
    from launch.substitutions import PythonExpression
    return PythonExpression(["'false' if '", condition, "' == 'true' else 'true'"])
