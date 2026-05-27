#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
FAST-LIO启动文件
基于Livox MID-360 LiDAR的高精度里程计估计
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():

    sentinel_fast_lio_dir = get_package_share_directory('sentinel_fast_lio')
    default_fast_lio_config_file = os.path.join(
        sentinel_fast_lio_dir, 'config', 'fast_lio_config.yaml')

    default_rviz_config_file = os.path.join(
        sentinel_fast_lio_dir, 'rviz', 'fast_lio.rviz')

    # 声明launch参数
    use_rviz_arg = DeclareLaunchArgument('use_rviz', default_value='False',
                                         description='Use rviz to visualize the result')
    rviz_arg = DeclareLaunchArgument('rviz', default_value=default_rviz_config_file,
                                     description='Rviz config file')
    config_file_arg = DeclareLaunchArgument('config_file', default_value=default_fast_lio_config_file,
                                            description='FAST-LIO config file')

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', LaunchConfiguration('rviz')],
        condition=LaunchConfiguration('use_rviz'),
    )
    
    fast_lio_node = Node(
        package='fast_lio',
        executable='fastlio_mapping',
        parameters=[LaunchConfiguration('config_file')],
        remappings=[
            ('/livox/lidar', '/livox/lidar'),
            ('/livox/imu', '/livox/imu'),
            ('/Laser_map', '/laser_map')
        ],
        output='screen'
    )

    # 坐标系转换
    livox_to_mid360_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0.13', '0', '0', '0', 'livox_frame', 'livox_base_link']
    )

    return LaunchDescription([
        use_rviz_arg,
        rviz_arg,
        config_file_arg,
        livox_to_mid360_node,
        fast_lio_node,
        rviz_node
    ])
