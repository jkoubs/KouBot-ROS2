from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.actions import TimerAction
from launch_ros.actions import Node
import os

def generate_launch_description():

    pkg_path = get_package_share_directory('koubot_nav')
    map_file = os.path.join(pkg_path, 'maps', 'my_map.yaml')
    amcl_params = os.path.join(pkg_path, 'config', 'amcl.yaml')


    laser_scan_matcher_node = Node(
        package='ros2_laser_scan_matcher',
        executable='laser_scan_matcher',
        name='laser_scan_matcher',
        output='screen',
        parameters=[{
            'base_frame': 'base_footprint',
            'odom_frame': 'odom',
            'laser_frame': 'lidar_link',
            'publish_odom': '/odom',
            'publish_tf': True,
            'qos_overrides./tf.publisher.durability': 'transient_local',
            'qos_overrides./tf.publisher.history': 'keep_last',
            'qos_overrides./tf.publisher.depth': 10,
        }]
    )

    amcl_node = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[amcl_params]
    )

    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[
            {'yaml_filename': map_file},
            {'use_sim_time': False},
            {'use_transient_local': True}
        ]
    )

    lifecycle_manager_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[
            {'use_sim_time': False},
            {'autostart': True},
            {'node_names': ['map_server', 'amcl']}
        ]
    )

    return LaunchDescription([
        laser_scan_matcher_node,
        TimerAction(
            period=5.0,  # wait 5 seconds
            actions=[
                map_server_node,
                amcl_node,
                lifecycle_manager_node
            ]
        )
    ])
