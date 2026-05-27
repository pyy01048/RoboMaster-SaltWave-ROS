import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    config_path = get_package_share_directory('sentinel_terrain_analysis')
    default_config = os.path.join(config_path, 'config', 'terrain_config.yaml')
    return LaunchDescription([
        Node(
            package='sentinel_terrain_analysis',
            executable='terrain_analysis_node',
            name='terrain_analysis',
            output='screen',
            parameters=[default_config]
        )
    ])
