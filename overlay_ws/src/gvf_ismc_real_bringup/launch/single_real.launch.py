from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    gvf_ismc_config = Path(
        get_package_share_directory("gvf_ismc_path_following")
    ) / "config" / "gvf_ismc_path_following.yaml"
    fastlio2_leveling_config = Path(
        get_package_share_directory("fastlio2_to_ego_swarm_leveling")
    ) / "config" / "fastlio2_to_ego_swarm_leveling.yaml"
    fastlio2_px4_config = Path(
        get_package_share_directory("fastlio2_to_px4_odometry")
    ) / "config" / "fastlio2_to_px4_odometry.yaml"
    position_cmd_px4_config = Path(
        get_package_share_directory("position_cmd_to_px4_bridge")
    ) / "config" / "position_cmd_to_px4_bridge.yaml"

    use_acceleration_feedforward = LaunchConfiguration(
        "use_acceleration_feedforward"
    )
    drone_id = LaunchConfiguration("drone_id")
    target_system = LaunchConfiguration("target_system")
    odom_topic = LaunchConfiguration("odom_topic")
    leveled_odom_topic = LaunchConfiguration("leveled_odom_topic")

    planning_prefix = ["/drone_", drone_id, "_planning"]
    pos_cmd_topic = planning_prefix + ["/pos_cmd"]
    topic_prefix = ["/drone_", drone_id]
    fmu_in_prefix = topic_prefix + ["/fmu/in"]
    fmu_out_prefix = topic_prefix + ["/fmu/out"]

    return LaunchDescription([
        DeclareLaunchArgument("drone_id", default_value="1"),
        DeclareLaunchArgument("target_system", default_value="2"),
        DeclareLaunchArgument(
            "odom_topic",
            default_value=["/drone_", drone_id, "/aft_mapped_to_init"],
        ),
        DeclareLaunchArgument(
            "leveled_odom_topic",
            default_value=["/drone_", drone_id, "/aft_mapped_to_init_level"],
        ),
        DeclareLaunchArgument(
            "use_acceleration_feedforward",
            default_value="false",
        ),
        Node(
            package="fastlio2_to_px4_odometry",
            executable="fastlio2_to_px4_odometry_node",
            name="fastlio2_to_px4_odometry",
            output="screen",
            parameters=[
                str(fastlio2_px4_config),
                {"input_topic": leveled_odom_topic},
                {"output_topic": fmu_in_prefix + ["/vehicle_visual_odometry"]},
            ],
        ),
        Node(
            package="fastlio2_to_ego_swarm_leveling",
            executable="fastlio2_to_ego_swarm_leveling_node",
            name="fastlio2_to_ego_swarm_leveling",
            output="screen",
            parameters=[
                str(fastlio2_leveling_config),
                {"input_odom_topic": odom_topic},
                {"output_odom_topic": leveled_odom_topic},
            ],
        ),
        Node(
            package="gvf_ismc_path_following",
            executable="gvf_reference_node",
            name="gvf_reference_node",
            output="screen",
            parameters=[
                str(gvf_ismc_config),
                {"odom_topic": leveled_odom_topic},
            ],
        ),
        Node(
            package="gvf_ismc_path_following",
            executable="ismc_velocity_tracker_node",
            name="ismc_velocity_tracker_node",
            output="screen",
            parameters=[
                str(gvf_ismc_config),
                {"odom_topic": leveled_odom_topic},
                {"command_topic": pos_cmd_topic},
            ],
        ),
        Node(
            package="position_cmd_to_px4_bridge",
            executable="position_cmd_to_px4_bridge_node",
            name="position_cmd_to_px4_bridge",
            output="screen",
            parameters=[
                str(position_cmd_px4_config),
                {
                    "command_topic": pos_cmd_topic,
                    "offboard_mode_topic": fmu_in_prefix + ["/offboard_control_mode"],
                    "trajectory_setpoint_topic": fmu_in_prefix + ["/trajectory_setpoint"],
                    "vehicle_command_topic": fmu_in_prefix + ["/vehicle_command"],
                    "vehicle_odometry_topic": fmu_out_prefix + ["/vehicle_odometry"],
                    "fastlivo_odom_topic": leveled_odom_topic,
                    "position_control_enabled": False,
                    "velocity_control_enabled": True,
                    "auto_request_offboard_and_arm": False,
                    "target_system": ParameterValue(target_system, value_type=int),
                    "use_acceleration_feedforward": use_acceleration_feedforward,
                },
            ],
        ),
    ])
