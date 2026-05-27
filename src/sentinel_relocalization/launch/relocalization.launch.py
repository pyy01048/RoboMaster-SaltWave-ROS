import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    config_path = get_package_share_directory('sentinel_relocalization')
    default_config = os.path.join(config_path, 'config', 'small_gicp_config.yaml')
    return LaunchDescription([
        Node(
            package='sentinel_relocalization',
            executable='small_gicp_reloc_node',
            name='relocalization',
            output='screen',
            parameters=[default_config]
        )
    ])
