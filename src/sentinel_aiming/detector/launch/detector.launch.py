from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='sentinel_aiming',
            executable='yolov8_detector_node',
            name='yolov8_detector',
            output='screen',
            parameters=['/home/pyy/Documents/sentiel/install/sentinel_aiming/share/sentinel_aiming/config/detector_params.yaml'],
        ),
    ])
