#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    bringup_share = get_package_share_directory("fast_livo2_mid360_bringup")
    default_livo_params = os.path.join(
        bringup_share, "config", "fast_livo2_mid360_lio.yaml"
    )
    default_camera_params = os.path.join(
        bringup_share, "config", "camera_startup_pinhole.yaml"
    )
    default_livox_config = os.path.join(
        bringup_share, "config", "MID360_config_drone_1.json"
    )

    livox_config_file = LaunchConfiguration("livox_config_file")
    livo_params_file = LaunchConfiguration("livo_params_file")
    camera_params_file = LaunchConfiguration("camera_params_file")
    publish_freq = LaunchConfiguration("publish_freq")
    xfer_format = LaunchConfiguration("xfer_format")
    multi_topic = LaunchConfiguration("multi_topic")
    frame_id = LaunchConfiguration("frame_id")
    base_link_pitch_rad = LaunchConfiguration("base_link_pitch_rad")

    return LaunchDescription([
        DeclareLaunchArgument(
            "livox_config_file",
            default_value=default_livox_config,
            description="Path to the MID360 livox_ros_driver2 JSON config",
        ),
        DeclareLaunchArgument(
            "livo_params_file",
            default_value=default_livo_params,
            description="Project-owned FAST-LIVO2 pure-LIO parameter file",
        ),
        DeclareLaunchArgument(
            "camera_params_file",
            default_value=default_camera_params,
            description="Startup-only camera model file required by FAST-LIVO2",
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
                "upstream IMU stays on livox_frame, so do not change casually"
            ),
        ),
        DeclareLaunchArgument(
            "base_link_pitch_rad",
            default_value="-0.519",
            description=(
                "Static aft_mapped->base_link pitch compensation in radians; "
                "use this to remove the fixed sensor installation pitch only"
            ),
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
        ),
        Node(
            package="fast_livo",
            executable="fastlivo_mapping",
            name="laserMapping",
            output="screen",
            parameters=[
                livo_params_file,
                camera_params_file,
            ],
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="world_to_camera_init",
            output="screen",
            arguments=[
                "--x", "0",
                "--y", "0",
                "--z", "0",
                "--roll", "0",
                "--pitch", "0",
                "--yaw", "0",
                "--frame-id", "world",
                "--child-frame-id", "camera_init",
            ],
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="aft_mapped_to_base_link",
            output="screen",
            arguments=[
                "--x", "0",
                "--y", "0",
                "--z", "0",
                "--roll", "0",
                "--pitch", base_link_pitch_rad,
                "--yaw", "0",
                "--frame-id", "aft_mapped",
                "--child-frame-id", "base_link",
            ],
        ),
    ])
