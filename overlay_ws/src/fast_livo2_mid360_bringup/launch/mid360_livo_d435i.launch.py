#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    bringup_share = get_package_share_directory("fast_livo2_mid360_bringup")
    default_livo_params = os.path.join(
        bringup_share, "config", "fast_livo2_mid360_livo_d435i.yaml"
    )
    default_camera_params = os.path.join(
        bringup_share, "config", "d435i_infra1_pinhole.yaml"
    )
    default_livox_config = os.path.join(
        bringup_share, "config", "MID360_config_drone_1.json"
    )

    livox_config_file = LaunchConfiguration("livox_config_file")
    livo_params_file = LaunchConfiguration("livo_params_file")
    camera_params_file = LaunchConfiguration("camera_params_file")
    realsense_serial_no = LaunchConfiguration("realsense_serial_no")
    publish_freq = LaunchConfiguration("publish_freq")
    xfer_format = LaunchConfiguration("xfer_format")
    multi_topic = LaunchConfiguration("multi_topic")
    frame_id = LaunchConfiguration("frame_id")
    infra_profile = LaunchConfiguration("infra_profile")
    base_link_pitch_rad = LaunchConfiguration("base_link_pitch_rad")
    img_time_offset = LaunchConfiguration("img_time_offset")

    return LaunchDescription([
        DeclareLaunchArgument(
            "livox_config_file",
            default_value=default_livox_config,
            description="Path to the MID360 livox_ros_driver2 JSON config",
        ),
        DeclareLaunchArgument(
            "livo_params_file",
            default_value=default_livo_params,
            description="Project-owned FAST-LIVO2 LiVO runtime parameter file",
        ),
        DeclareLaunchArgument(
            "camera_params_file",
            default_value=default_camera_params,
            description="FAST-LIVO2 camera model file for D435i infra1",
        ),
        DeclareLaunchArgument(
            "realsense_serial_no",
            default_value="",
            description="Intel RealSense serial number under /camera namespace",
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
            "infra_profile",
            default_value="1280x720x15",
            description="RealSense infra1 profile widthxheightxfps",
        ),
        DeclareLaunchArgument(
            "base_link_pitch_rad",
            default_value="-0.519",
            description=(
                "Static aft_mapped->base_link pitch compensation in radians; "
                "use this to remove the fixed sensor installation pitch only"
            ),
        ),
        DeclareLaunchArgument(
            "img_time_offset",
            default_value="0.0",
            description="Override for FAST-LIVO2 time_offset.img_time_offset in seconds",
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
                {"publish_freq": ParameterValue(publish_freq, value_type=float)},
                {"output_data_type": 0},
                {"frame_id": frame_id},
                {"user_config_path": livox_config_file},
                {"cmdline_input_bd_code": "livox0000000001"},
            ],
        ),
        Node(
            package="realsense2_camera",
            executable="realsense2_camera_node",
            namespace="camera",
            name="camera",
            output="screen",
            parameters=[
                {"camera_name": "camera"},
                {"serial_no": realsense_serial_no},
                {"enable_sync": False},
                {"enable_infra1": True},
                {"enable_color": False},
                {"enable_depth": False},
                {"enable_infra2": False},
                {"enable_gyro": False},
                {"enable_accel": False},
                {"depth_module.infra_profile": infra_profile},
                {"depth_module.infra1_format": "Y8"},
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
                {
                    "time_offset.img_time_offset": ParameterValue(
                        img_time_offset, value_type=float
                    )
                },
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
