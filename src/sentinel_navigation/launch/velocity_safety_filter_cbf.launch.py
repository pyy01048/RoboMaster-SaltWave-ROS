import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('sentinel_bringup'),
        'config',
        'sentinel_navigation.yaml'
    )
    return LaunchDescription([
        Node(
            package='sentinel_navigation',
            executable='velocity_safety_filter_cbf',
            parameters=[config],
        ),
    ])
