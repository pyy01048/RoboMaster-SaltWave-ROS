from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    config_file = os.path.join(
        get_package_share_directory('sensor_fusion'),
        'config',
        'sensor_fusion_params.yaml'
    )
    
    return LaunchDescription([
        Node(
            package='sensor_fusion',
            executable='sensor_fusion_node',
            name='sensor_fusion_node',
            output='screen',
            parameters=[config_file],
            remappings=[
                ('/imu/data', '/imu/data'),
                ('/fastlio/odom', '/fastlio/odom'),
                ('/odom_filtered', '/odom_filtered')
            ]
        )
    ])
