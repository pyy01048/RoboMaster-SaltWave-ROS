#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
EKF融合启动文件
融合IMU和FAST-LIO里程计数据，提供高精度定位
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():

    pkg_sentinel_ekf_fusion = get_package_share_directory('sentinel_ekf_fusion')
    ekf_config_path = os.path.join(pkg_sentinel_ekf_fusion, 'config', 'ekf.yaml')

    # 声明launch参数
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='False',
        description='Use simulation/Gazebo clock')
    ekf_config_arg = DeclareLaunchArgument(
        'ekf_config',
        default_value=ekf_config_path,
        description='Path to the EKF configuration file')

    ekf_filter_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[LaunchConfiguration('ekf_config'), {
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'base_link_frame': 'base_link',
            'odom_frame': 'odom',
            'world_frame': 'odom'
        }]
    )

    ekf_fusion_node = Node(
        package='sentinel_ekf_fusion',
        executable='ekf_fusion_node',
        name='ekf_fusion_node',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'imu_topic': '/livox/imu',
            'fast_lio_odom_topic': '/Odometry',
            'fused_odom_topic': '/ekf_fusion_node/odom',
            'publish_rate': 50.0,
            'use_imu': True,
            'use_fast_lio': True
        }]
    )

    return LaunchDescription([
        use_sim_time_arg,
        ekf_config_arg,
        ekf_filter_node,
        ekf_fusion_node
    ])
