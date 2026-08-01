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
    default_livox_config = os.path.join(
        bringup_share, "config", "MID360_config_drone_1.json"
    )

    livox_config_file = LaunchConfiguration("livox_config_file")
    publish_freq = LaunchConfiguration("publish_freq")
    realsense_serial_no = LaunchConfiguration("realsense_serial_no")
    cmdline_input_bd_code = LaunchConfiguration("cmdline_input_bd_code")
    infra_profile = LaunchConfiguration("infra_profile")

    return LaunchDescription([
        DeclareLaunchArgument(
            "livox_config_file",
            default_value=default_livox_config,
            description="Path to the MID360 livox_ros_driver2 JSON config",
        ),
        DeclareLaunchArgument(
            "publish_freq",
            default_value="10.0",
            description="Livox publish frequency in Hz",
        ),
        DeclareLaunchArgument(
            "realsense_serial_no",
            default_value="",
            description="Intel RealSense serial number for /camera capture",
        ),
        DeclareLaunchArgument(
            "cmdline_input_bd_code",
            default_value="livox0000000001",
            description="Livox broadcast code for direct hardware capture",
        ),
        DeclareLaunchArgument(
            "infra_profile",
            default_value="1280x720x15",
            description="RealSense infrared stream profile widthxheightxfps",
        ),
        Node(
            package="livox_ros_driver2",
            executable="livox_ros_driver2_node",
            name="livox_lidar_publisher",
            output="screen",
            parameters=[
                {"xfer_format": 0},
                {"multi_topic": 0},
                {"data_src": 0},
                {"publish_freq": ParameterValue(publish_freq, value_type=float)},
                {"output_data_type": 0},
                {"frame_id": "livox_frame"},
                {"user_config_path": livox_config_file},
                {"cmdline_input_bd_code": cmdline_input_bd_code},
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
    ])
