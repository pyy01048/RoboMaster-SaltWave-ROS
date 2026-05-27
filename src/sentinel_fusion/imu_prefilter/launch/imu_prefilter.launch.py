from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='sentinel_fusion',
            executable='imu_prefilter_node',
            name='imu_prefilter',
            output='screen',
        ),
    ])
