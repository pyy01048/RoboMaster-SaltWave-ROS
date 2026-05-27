from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    tree_file = os.path.join(
        get_package_share_directory('decision_making'),
        'behavior_trees',
        'tactical.xml'
    )
    
    return LaunchDescription([
        Node(
            package='decision_making',
            executable='decision_node',
            name='decision_node',
            output='screen',
            parameters=[{'tree_file': tree_file}]
        )
    ])
