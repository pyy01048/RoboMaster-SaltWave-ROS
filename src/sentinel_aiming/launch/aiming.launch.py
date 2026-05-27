from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(
                get_package_share_directory('sentinel_aiming'), 'detector', 'launch',
                'detector.launch.py')]),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(
                get_package_share_directory('sentinel_aiming'), 'tracker', 'launch',
                'tracker.launch.py')]),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(
                get_package_share_directory('sentinel_aiming'), 'ballistics', 'launch',
                'ballistics.launch.py')]),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(
                get_package_share_directory('sentinel_aiming'), 'servo_controller', 'launch',
                'servo_controller.launch.py')]),
        ),
    ])
