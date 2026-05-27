from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='sentinel_aiming',
            executable='ballistics_node',
            name='ballistics',
            output='screen',
            parameters=['/home/pyy/Documents/sentiel/install/sentinel_aiming/share/sentinel_aiming/config/ballistics_params.yaml'],
        ),
    ])
