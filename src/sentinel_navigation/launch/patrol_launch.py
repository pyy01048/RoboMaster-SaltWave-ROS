from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    config_file = os.path.join(
        get_package_share_directory('sentinel_navigation'),
        'config',
        'patrol_waypoints.yaml'
    )
    
    return LaunchDescription([
        Node(
            package='sentinel_navigation',
            executable='patrol_waypoints',
            name='patrol_manager',
            output='screen',
            parameters=[config_file]
        )
    ])
