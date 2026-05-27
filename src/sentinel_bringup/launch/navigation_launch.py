from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    sentinel_bringup_dir = get_package_share_directory('sentinel_bringup')
    
    nav2_params_file = os.path.join(sentinel_bringup_dir, 'config', 'nav2_params.yaml')
    
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py')),
            launch_arguments={
                'params_file': nav2_params_file,
                'use_sim_time': 'false'
            }.items()
        )
    ])
