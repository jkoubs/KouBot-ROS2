#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import Command
from launch_ros.parameter_descriptions import ParameterValue

import os

def generate_launch_description():
    description_pkg = get_package_share_directory('robot_description')
    xacro_file = os.path.join(description_pkg, 'urdf', 'base', 'koubot_base.xacro')

    rplidar_pkg = get_package_share_directory('rplidar_ros')
    rplidar_launch = os.path.join(rplidar_pkg, 'launch', 'rplidar_a1_launch.py')

    return LaunchDescription([
        # Robot State Publisher using xacro
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': ParameterValue(
                    Command(['xacro ', xacro_file]),
                    value_type=str
                )
            }]
        ),
        # Joint State Publisher
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher_node',
            output="screen"
        ),
        # RPLIDAR Node 
        Node(
            package='rplidar_ros',
            executable='rplidar_node',
            name='rplidar',
            output='screen',
            parameters=[
                {'serial_port': '/dev/ttyUSB0'},
                {'frame_id': 'lidar_link'},
                {'serial_baudrate': 115200}
            ]
        ),
    ])
