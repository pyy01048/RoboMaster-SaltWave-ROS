import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    model = LaunchConfiguration('model')

    pkg_dir = get_package_share_directory('sentinel_description')
    default_model = os.path.join(pkg_dir, 'urdf', 'sentinel.urdf.xacro')

    robot_description = {
        'robot_description': Command(['xacro ', model]),
        'use_sim_time': use_sim_time,
    }

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation clock if true',
        ),
        DeclareLaunchArgument(
            'model',
            default_value=default_model,
            description='Path to sentinel URDF/Xacro file',
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[robot_description],
        ),
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
        ),
    ])
