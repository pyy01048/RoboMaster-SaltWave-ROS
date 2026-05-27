from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os


def generate_launch_description():
    bringup_dir = get_package_share_directory('sentinel_bringup')
    nav2_dir = get_package_share_directory('nav2_bringup')
    description_dir = get_package_share_directory('sentinel_description')
    fast_lio_dir = get_package_share_directory('sentinel_fast_lio')
    ekf_fusion_dir = get_package_share_directory('sentinel_ekf_fusion')

    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    slam = LaunchConfiguration('slam')
    rviz = LaunchConfiguration('rviz')
    params_file = LaunchConfiguration('params_file')
    map_file = LaunchConfiguration('map')
    use_fast_lio = LaunchConfiguration('use_fast_lio')
    use_ekf_fusion = LaunchConfiguration('use_ekf_fusion')

    default_map = os.path.join(bringup_dir, 'maps', 'empty_rm_field.yaml')
    default_params = os.path.join(bringup_dir, 'config', 'nav2_params.yaml')
    default_rviz = os.path.join(bringup_dir, 'rviz', 'sentinel_nav.rviz')
    default_fast_lio_config = os.path.join(fast_lio_dir, 'config', 'fast_lio_config.yaml')
    default_ekf_config = os.path.join(ekf_fusion_dir, 'config', 'ekf.yaml')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('autostart', default_value='true'),
        DeclareLaunchArgument('slam', default_value='false'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('map', default_value=default_map),
        DeclareLaunchArgument('params_file', default_value=default_params),
        DeclareLaunchArgument('use_fast_lio', default_value='true'),
        DeclareLaunchArgument('use_ekf_fusion', default_value='true'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(description_dir, 'launch', 'description.launch.py')
            ),
            launch_arguments={'use_sim_time': use_sim_time}.items(),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(fast_lio_dir, 'launch', 'fast_lio.launch.py')
            ),
            condition=IfCondition(use_fast_lio),
            launch_arguments={
                'use_rviz': 'false',
                'config_file': default_fast_lio_config,
            }.items(),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(ekf_fusion_dir, 'launch', 'ekf_fusion.launch.py')
            ),
            condition=IfCondition(use_ekf_fusion),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'ekf_config': default_ekf_config,
            }.items(),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(nav2_dir, 'launch', 'bringup_launch.py')),
            launch_arguments={
                'slam': slam,
                'map': map_file,
                'use_sim_time': use_sim_time,
                'params_file': params_file,
                'autostart': autostart,
            }.items(),
        ),

        Node(
            package='sentinel_navigation',
            executable='velocity_safety_filter',
            name='velocity_safety_filter',
            output='screen',
            parameters=[os.path.join(bringup_dir, 'config', 'sentinel_navigation.yaml')],
        ),
        Node(
            package='sentinel_navigation',
            executable='sentinel_commander',
            name='sentinel_commander',
            output='screen',
            parameters=[os.path.join(bringup_dir, 'config', 'sentinel_navigation.yaml')],
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', default_rviz],
            parameters=[{'use_sim_time': use_sim_time}],
            condition=IfCondition(rviz),
        ),
    ])
