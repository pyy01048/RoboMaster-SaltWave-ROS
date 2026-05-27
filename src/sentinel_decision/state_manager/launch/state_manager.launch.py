from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='sentinel_decision',
            executable='state_manager_node',
            name='state_manager',
            output='screen',
        ),
    ])
