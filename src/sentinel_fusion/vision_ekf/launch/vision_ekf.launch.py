from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='sentinel_fusion',
            executable='vision_ekf_node',
            name='vision_ekf',
            output='screen',
            parameters=['/home/pyy/Documents/sentiel/install/sentinel_fusion/share/sentinel_fusion/config/vision_ekf_params.yaml'],
        ),
    ])
