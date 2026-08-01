"""Start only the formation visualizer/control command topics."""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package='rviz2',
                executable='rviz2',
                name='formation_rviz',
                output='screen',
            )
        ]
    )
