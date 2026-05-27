import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node


def generate_launch_description():
    config_path = get_package_share_directory('sentinel_point_lio')
    default_config = os.path.join(config_path, 'config', 'point_lio_config.yaml')
    rviz_path = os.path.join(config_path, 'rviz', 'pointlio.rviz')

    lidar_to_IMU = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='lidar2IMU',
        arguments=['0', '0', '0', '1', '0', '0', '0', 'livox_frame', 'camera_init']
    )

    mapping = Node(
        package='pointlio_mapping',
        executable='pointlio_mapping_ros',
        name='pointlio_mapping',
        output='screen',
        parameters=[default_config]
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_path]
    )
    
    data_recorder = Node(
        package='pointlio_mapping',
        executable='data_recorder',
        name='data_recorder',
        output='screen',
        parameters=[default_config]
    )

    return LaunchDescription([
        lidar_to_IMU,
        mapping,
        rviz,
        data_recorder
    ])
