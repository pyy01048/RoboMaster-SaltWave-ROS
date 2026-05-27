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
                get_package_share_directory('sentinel_fusion'), 'imu_prefilter', 'launch',
                'imu_prefilter.launch.py')]),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(
                get_package_share_directory('sentinel_fusion'), 'lidar_ekf', 'launch',
                'lidar_ekf.launch.py')]),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(
                get_package_share_directory('sentinel_fusion'), 'vision_ekf', 'launch',
                'vision_ekf.launch.py')]),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(
                get_package_share_directory('sentinel_fusion'), 'graph_optimizer', 'launch',
                'graph_optimizer.launch.py')]),
        ),
    ])
