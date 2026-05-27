from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='sentinel_aiming',
            executable='servo_controller_node',
            name='servo_controller',
            output='screen',
            parameters=['/home/pyy/Documents/sentiel/install/sentinel_aiming/share/sentinel_aiming/config/servo_params.yaml'],
        ),
    ])
