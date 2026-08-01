#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    bringup_share = get_package_share_directory("fast_lio2_mid360_bringup")

    default_fast_lio_params = os.path.join(
        bringup_share, "config", "fast_lio2_mid360.yaml"
    )
    default_livox_config = os.path.join(
        bringup_share, "config", "MID360_config_drone_1.json"
    )

    livox_config_file = LaunchConfiguration("livox_config_file")
    fast_lio_params_file = LaunchConfiguration("fast_lio_params_file")
    publish_freq = LaunchConfiguration("publish_freq")
    xfer_format = LaunchConfiguration("xfer_format")
    multi_topic = LaunchConfiguration("multi_topic")
    frame_id = LaunchConfiguration("frame_id")
    odom_topic = LaunchConfiguration("odom_topic")
    drone_id = LaunchConfiguration("drone_id")

    drone_prefix = ["/drone_", drone_id]
    livox_lidar_topic = drone_prefix + ["/livox/lidar"]
    livox_imu_topic = drone_prefix + ["/livox/imu"]

    return LaunchDescription([
        DeclareLaunchArgument(
            "drone_id",
            default_value="1",
            description="Drone index used to prefix per-drone topics",
        ),
        DeclareLaunchArgument(
            "livox_config_file",
            default_value=default_livox_config,
            description="Path to the MID360 livox_ros_driver2 JSON config",
        ),
        DeclareLaunchArgument(
            "fast_lio_params_file",
            default_value=default_fast_lio_params,
            description="Project-owned FAST-LIO2 parameter file",
        ),
        DeclareLaunchArgument(
            "publish_freq",
            default_value="10.0",
            description="Livox publish frequency in Hz",
        ),
        DeclareLaunchArgument(
            "xfer_format",
            default_value="1",
            description="Livox transfer format: 1 means CustomMsg",
        ),
        DeclareLaunchArgument(
            "multi_topic",
            default_value="0",
            description="Livox shared-topic mode; keep 0 for /livox/lidar and /livox/imu",
        ),
        DeclareLaunchArgument(
            "frame_id",
            default_value="livox_frame",
            description=(
                "Frame ID for Livox point cloud/custom outputs only; "
                "upstream IMU stays on livox_frame"
            ),
        ),
        DeclareLaunchArgument(
            "odom_topic",
            default_value=["/drone_", drone_id, "/aft_mapped_to_init"],
            description="Compatibility odometry topic consumed by downstream planner/bridge nodes",
        ),
        Node(
            package="livox_ros_driver2",
            executable="livox_ros_driver2_node",
            name="livox_lidar_publisher",
            output="screen",
            parameters=[
                {"xfer_format": xfer_format},
                {"multi_topic": multi_topic},
                {"data_src": 0},
                {"publish_freq": publish_freq},
                {"output_data_type": 0},
                {"frame_id": frame_id},
                {"user_config_path": livox_config_file},
                {"cmdline_input_bd_code": "livox0000000001"},
            ],
            remappings=[
                ("/livox/lidar", livox_lidar_topic),
                ("/livox/imu", livox_imu_topic),
            ],
        ),
        Node(
            package="fast_lio",
            executable="fastlio_mapping",
            name="fastlio_mapping",
            output="screen",
            parameters=[
                fast_lio_params_file,
                {"common.lid_topic": livox_lidar_topic},
                {"common.imu_topic": livox_imu_topic},
            ],
            remappings=[
                ("/Odometry", odom_topic),
                ("/cloud_registered", drone_prefix + ["/cloud_registered"]),
                ("/cloud_registered_body", drone_prefix + ["/cloud_registered_body"]),
                ("/path", drone_prefix + ["/path"]),
            ],
        ),
    ])
