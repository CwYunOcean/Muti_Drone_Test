"""Start one real-vehicle controller using the canonical drone namespace."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory('distribute_control')
    config_file = os.path.join(package_share, 'config', 'params.yaml')
    drone_id = LaunchConfiguration('drone_id')
    target_system = LaunchConfiguration('target_system')
    return LaunchDescription(
        [
            DeclareLaunchArgument('drone_id', default_value='1'),
            DeclareLaunchArgument('target_system', default_value='1'),
            Node(
                package='distribute_control',
                executable='offboard_node',
                name='control_node',
                parameters=[
                    config_file,
                    {
                        'system.uav_id': drone_id,
                        'system.total_uavs': 1,
                        'system.is_sim': False,
                        'system.target_system': target_system,
                        'control.auto_arm': False,
                    },
                ],
                output='screen',
            ),
            Node(
                package='distribute_control',
                executable='visualizer_node',
                name='viz_node',
                parameters=[
                    config_file,
                    {
                        'system.uav_id': drone_id,
                        'system.total_uavs': 1,
                        'system.is_sim': False,
                    },
                ],
                output='screen',
            ),
        ]
    )
