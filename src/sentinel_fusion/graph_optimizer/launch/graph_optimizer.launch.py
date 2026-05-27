from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='sentinel_fusion',
            executable='graph_optimizer_node',
            name='graph_optimizer',
            output='screen',
            parameters=['/home/pyy/Documents/sentiel/install/sentinel_fusion/share/sentinel_fusion/config/graph_optimizer_params.yaml'],
        ),
    ])
