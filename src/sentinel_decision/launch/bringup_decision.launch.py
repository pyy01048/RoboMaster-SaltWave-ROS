from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(
                get_package_share_directory('sentinel_decision'), 'state_manager', 'launch',
                'state_manager.launch.py')]),
        ),

        Node(
            package='behaviortree_cpp_v3',
            executable='bt_recorder',
            arguments=['-z', '1666'],
            output='screen',
        ),
    ])
