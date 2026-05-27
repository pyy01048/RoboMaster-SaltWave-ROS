from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    config_file = os.path.join(
        get_package_share_directory('vision_aiming'),
        'config',
        'aiming_params.yaml'
    )
    
    return LaunchDescription([
        Node(
            package='vision_aiming',
            executable='detector_node',
            name='detector_node',
            output='screen',
            parameters=[config_file]
        ),
        Node(
            package='vision_aiming',
            executable='tracker_node',
            name='tracker_node',
            output='screen',
            parameters=[config_file]
        ),
        Node(
            package='vision_aiming',
            executable='ballistics_node',
            name='ballistics_node',
            output='screen',
            parameters=[config_file]
        ),
        Node(
            package='vision_aiming',
            executable='servo_controller_node',
            name='servo_controller_node',
            output='screen',
            parameters=[config_file]
        ),
    ])
