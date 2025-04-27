from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    pkg_koubot_nav = get_package_share_directory('koubot_nav')

    # Path to navigation parameters
    nav2_params = os.path.join(pkg_koubot_nav, 'config', 'navigation.yaml')

    # Core Nav2 Nodes
    planner_server_node = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[nav2_params]
    )

    controller_server_node = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[nav2_params]
    )

    recoveries_server_node = Node(
        package='nav2_recoveries',
        executable='recoveries_server',
        name='recoveries_server',
        output='screen',
        parameters=[nav2_params]
    )

    bt_navigator_node = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[nav2_params]
    )

    # Lifecycle Manager
    lifecycle_manager_navigation_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[
            {'use_sim_time': False},
            {'autostart': True},
            {'node_names': [
                'planner_server',
                'controller_server',
                'recoveries_server',
                'bt_navigator'
            ]}
        ]
    )

    nav2_nodes = [
        planner_server_node,
        controller_server_node,
        recoveries_server_node,
        bt_navigator_node,
        lifecycle_manager_navigation_node
    ]

    return LaunchDescription(nav2_nodes)