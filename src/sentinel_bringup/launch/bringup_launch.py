import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    bringup_dir = get_package_share_directory('sentinel_bringup')
    nav2_dir = get_package_share_directory('nav2_bringup')
    description_dir = get_package_share_directory('sentinel_description')
    point_lio_dir = get_package_share_directory('sentinel_point_lio')
    relocalization_dir = get_package_share_directory('sentinel_relocalization')
    terrain_analysis_dir = get_package_share_directory('sentinel_terrain_analysis')
    ekf_fusion_dir = get_package_share_directory('sentinel_ekf_fusion')
    fusion_dir = get_package_share_directory('sentinel_fusion')
    decision_dir = get_package_share_directory('sentinel_decision')
    aiming_dir = get_package_share_directory('sentinel_aiming')

    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    slam = LaunchConfiguration('slam')
    rviz = LaunchConfiguration('rviz')
    map_file = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')
    use_point_lio = LaunchConfiguration('use_point_lio')
    use_relocalization = LaunchConfiguration('use_relocalization')
    use_terrain_analysis = LaunchConfiguration('use_terrain_analysis')
    use_ekf_fusion = LaunchConfiguration('use_ekf_fusion')
    use_fusion = LaunchConfiguration('use_fusion')
    use_decision = LaunchConfiguration('use_decision')
    use_aiming = LaunchConfiguration('use_aiming')

    default_map = os.path.join(bringup_dir, 'maps', 'empty_rm_field.yaml')
    default_params = os.path.join(bringup_dir, 'config', 'nav2_params.yaml')
    default_rviz = os.path.join(bringup_dir, 'rviz', 'sentinel_nav.rviz')
    default_point_lio_config = os.path.join(point_lio_dir, 'config', 'point_lio_config.yaml')
    default_relocalization_config = os.path.join(relocalization_dir, 'config', 'small_gicp_config.yaml')
    default_terrain_config = os.path.join(terrain_analysis_dir, 'config', 'terrain_config.yaml')
    default_ekf_config = os.path.join(ekf_fusion_dir, 'config', 'ekf.yaml')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('autostart', default_value='true'),
        DeclareLaunchArgument('slam', default_value='false'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('map', default_value=default_map),
        DeclareLaunchArgument('params_file', default_value=default_params),
        DeclareLaunchArgument('use_point_lio', default_value='true'),
        DeclareLaunchArgument('use_relocalization', default_value='true'),
        DeclareLaunchArgument('use_terrain_analysis', default_value='true'),
        DeclareLaunchArgument('use_ekf_fusion', default_value='true'),
        DeclareLaunchArgument('use_fusion', default_value='true'),
        DeclareLaunchArgument('use_decision', default_value='true'),
        DeclareLaunchArgument('use_aiming', default_value='true'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(description_dir, 'launch', 'description.launch.py')
            ),
            launch_arguments={'use_sim_time': use_sim_time}.items(),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(point_lio_dir, 'launch', 'point_lio.launch.py')
            ),
            condition=IfCondition(use_point_lio),
            launch_arguments={
                'use_rviz': 'false',
                'config_file': default_point_lio_config,
            }.items(),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(terrain_analysis_dir, 'launch', 'terrain_analysis.launch.py')
            ),
            condition=IfCondition(use_terrain_analysis),
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
            PythonLaunchDescriptionSource(
                os.path.join(fusion_dir, 'launch', 'fusion.launch.py')
            ),
            condition=IfCondition(use_fusion),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(decision_dir, 'launch', 'bringup_decision.launch.py')
            ),
            condition=IfCondition(use_decision),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(aiming_dir, 'launch', 'aiming.launch.py')
            ),
            condition=IfCondition(use_aiming),
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
            executable='velocity_safety_filter_cbf',
            name='velocity_safety_filter_cbf',
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
