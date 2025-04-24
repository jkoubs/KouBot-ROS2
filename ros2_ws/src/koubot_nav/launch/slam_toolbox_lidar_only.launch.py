from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # SLAM Toolbox Node
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[
                {'use_sim_time': False},
                {'slam_params_file': '/ros2_ws/src/mecanum_interface/config/mapper_params_online_async.yaml'}
            ],
            arguments=['--ros-args', '--param', 'queue_size:=50']
        ),
    ])