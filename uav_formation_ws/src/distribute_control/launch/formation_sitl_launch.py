"""Launch a namespaced multi-PX4 SITL formation and its ROS nodes."""

import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from formation_launch_common import drone_prefix, formation_grid_origin
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    GroupAction,
    OpaqueFunction,
    TimerAction,
)
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace
import yaml


def _load_defaults(config_file: str) -> tuple[int, float]:
    try:
        with open(config_file, 'r', encoding='utf-8') as stream:
            params = yaml.safe_load(stream)['/**']['ros__parameters']
        return (
            int(params.get('system', {}).get('total_uavs', 3)),
            float(params.get('mission', {}).get('formation_spacing', 3.0)),
        )
    except (OSError, KeyError, TypeError, ValueError):
        return 3, 3.0


def _as_bool(value: str) -> bool:
    return value.lower() in {'1', 'true', 'yes', 'on'}


def _build_actions(context, controller_executable: str, visualizer_executable: str):
    package_name = 'distribute_control'
    package_share = get_package_share_directory(package_name)
    config_file = os.path.join(package_share, 'config', 'params.yaml')
    default_total, spacing = _load_defaults(config_file)
    total_uavs = int(LaunchConfiguration('total_uavs').perform(context) or default_total)
    px4_dir = os.path.expanduser(LaunchConfiguration('px4_dir').perform(context))
    start_px4 = _as_bool(LaunchConfiguration('start_px4').perform(context))
    start_rviz = _as_bool(LaunchConfiguration('start_rviz').perform(context))
    model_name = LaunchConfiguration('model').perform(context)
    actions = []

    if start_rviz:
        actions.append(
            Node(package='rviz2', executable='rviz2', name='formation_rviz', output='screen')
        )

    for drone_id in range(1, total_uavs + 1):
        prefix = drone_prefix(drone_id)
        offset_x, offset_y = formation_grid_origin(drone_id, total_uavs, spacing)
        model_pose = f'{offset_x},{offset_y},0,0,0,0'
        if start_px4:
            px4_bin = str(Path(px4_dir) / 'build' / 'px4_sitl_default' / 'bin' / 'px4')
            actions.append(
                TimerAction(
                    period=2.0 * (drone_id - 1),
                    actions=[
                        ExecuteProcess(
                            cmd=[px4_bin, '-i', str(drone_id)],
                            cwd=px4_dir,
                            additional_env={
                                'PX4_SYS_AUTOSTART': '4001',
                                'PX4_SIM_MODEL': f'gz_{model_name}',
                                'PX4_GZ_MODEL_POSE': model_pose,
                                'PX4_UXRCE_DDS_NS': f'drone_{drone_id}',
                            },
                            output='screen',
                        )
                    ],
                )
            )

        actions.append(
            GroupAction(
                [
                    PushRosNamespace(prefix.lstrip('/')),
                    Node(
                        package=package_name,
                        executable=controller_executable,
                        name=f'control_node_{drone_id}',
                        parameters=[
                            config_file,
                            {
                                'system.uav_id': drone_id,
                                'system.total_uavs': total_uavs,
                                'system.is_sim': True,
                                'offset_x': offset_x,
                                'offset_y': offset_y,
                                'control.auto_arm': False,
                            },
                        ],
                        output='screen',
                        emulate_tty=True,
                    ),
                    Node(
                        package=package_name,
                        executable=visualizer_executable,
                        name=f'viz_node_{drone_id}',
                        parameters=[
                            config_file,
                            {
                                'system.uav_id': drone_id,
                                'system.total_uavs': total_uavs,
                                'system.is_sim': True,
                                'offset_x': offset_x,
                                'offset_y': offset_y,
                            },
                        ],
                        output='screen',
                        emulate_tty=True,
                    ),
                ]
            )
        )
    return actions


def generate_sitl_launch_description(
    controller_executable='offboard_node', visualizer_executable='visualizer_node'
):
    package_share = get_package_share_directory('distribute_control')
    config_file = os.path.join(package_share, 'config', 'params.yaml')
    default_total, _ = _load_defaults(config_file)
    return LaunchDescription(
        [
            DeclareLaunchArgument('total_uavs', default_value=str(default_total)),
            DeclareLaunchArgument(
                'px4_dir', default_value=os.path.expanduser('~/PX4/PX4-Autopilot')
            ),
            DeclareLaunchArgument('model', default_value='x500'),
            DeclareLaunchArgument('start_px4', default_value='true'),
            DeclareLaunchArgument('start_rviz', default_value='false'),
            OpaqueFunction(
                function=lambda context: _build_actions(
                    context, controller_executable, visualizer_executable
                )
            ),
        ]
    )


def generate_launch_description():
    return generate_sitl_launch_description()
