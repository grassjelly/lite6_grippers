#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    robot_ip = LaunchConfiguration('robot_ip')
    hw_ns = LaunchConfiguration('hw_ns', default='ufactory')

    moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('xarm_moveit_config'),
            'launch', '_robot_moveit_realmove.launch.py',
        ])),
        launch_arguments={
            'robot_ip': robot_ip,
            'dof': '6',
            'robot_type': 'lite',
            'hw_ns': hw_ns,
            'add_custom_gripper': 'true',
        }.items(),
    )

    gripper_node = Node(
        package='parallel_gripper',
        executable='gripper_driver',
        name='parallel_gripper_driver',
        output='screen',
        parameters=[{
            'robot_ip': robot_ip,
            'joint_state_publish_rate': 10.0,
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument('robot_ip', description='xArm Lite6 IP address'),
        DeclareLaunchArgument('hw_ns', default_value='ufactory',
                              description='Hardware namespace'),
        moveit_launch,
        gripper_node,
    ])
