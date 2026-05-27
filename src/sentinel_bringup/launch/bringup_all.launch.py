from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    sensor_fusion_dir = get_package_share_directory('sensor_fusion')
    sentinel_navigation_dir = get_package_share_directory('sentinel_navigation')
    vision_aiming_dir = get_package_share_directory('vision_aiming')
    decision_making_dir = get_package_share_directory('decision_making')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time if true'
        ),
        
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(sensor_fusion_dir, 'launch', 'fusion_launch.py')),
            launch_arguments={'use_sim_time': use_sim_time}.items()
        ),
        
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(sentinel_navigation_dir, 'launch', 'cbf_launch.py')),
            launch_arguments={'use_sim_time': use_sim_time}.items()
        ),
        
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(vision_aiming_dir, 'launch', 'aiming_pipeline_launch.py')),
            launch_arguments={'use_sim_time': use_sim_time}.items()
        ),
        
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(decision_making_dir, 'launch', 'decision_launch.py')),
            launch_arguments={'use_sim_time': use_sim_time}.items()
        ),
    ])
