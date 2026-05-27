from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='sentinel_fusion',
            executable='point_lio_wrapper_node',
            name='point_lio_wrapper',
            output='screen',
        ),
    ])
