from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import PythonExpression
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    planner_config = Path(
        get_package_share_directory("ego_swarm_real_bringup")
    ) / "config" / "ego_planner_real.yaml"
    fastlivo_px4_config = Path(
        get_package_share_directory("fastlivo_to_px4_odometry")
    ) / "config" / "fastlivo_to_px4_odometry.yaml"
    fastlio2_px4_config = Path(
        get_package_share_directory("fastlio2_to_px4_odometry")
    ) / "config" / "fastlio2_to_px4_odometry.yaml"
    fastlio2_leveling_config = Path(
        get_package_share_directory("fastlio2_to_ego_swarm_leveling")
    ) / "config" / "fastlio2_to_ego_swarm_leveling.yaml"
    position_cmd_px4_config = Path(
        get_package_share_directory("position_cmd_to_px4_bridge")
    ) / "config" / "position_cmd_to_px4_bridge.yaml"
    default_robot_params = Path(
        get_package_share_directory("ego_swarm_real_bringup")
    ) / "config" / "robot_overrides.yaml"

    odom_topic = LaunchConfiguration("odom_topic")
    cloud_topic = LaunchConfiguration("cloud_topic")
    odom_bridge_type = LaunchConfiguration("odom_bridge_type")
    planner_input_mode = LaunchConfiguration("planner_input_mode")
    leveled_odom_topic = LaunchConfiguration("leveled_odom_topic")
    leveled_cloud_topic = LaunchConfiguration("leveled_cloud_topic")
    leveled_frame_id = LaunchConfiguration("leveled_frame_id")
    drone_id = LaunchConfiguration("drone_id")
    target_system = LaunchConfiguration("target_system")
    robot_params_file = LaunchConfiguration("robot_params_file")

    drone_prefix = ["drone_", drone_id]
    planning_prefix = ["/drone_", drone_id, "_planning"]
    plan_vis_prefix = ["/drone_", drone_id, "_plan_vis"]
    pos_cmd_topic = planning_prefix + ["/pos_cmd"]
    topic_prefix = ["/drone_", drone_id]
    fmu_in_prefix = topic_prefix + ["/fmu/in"]
    fmu_out_prefix = topic_prefix + ["/fmu/out"]

    planner_odom_topic = PythonExpression([
        "'",
        leveled_odom_topic,
        "' if '",
        planner_input_mode,
        "' == 'fastlio2_leveled' else '",
        odom_topic,
        "'",
    ])
    planner_cloud_topic = PythonExpression([
        "'",
        leveled_cloud_topic,
        "' if '",
        planner_input_mode,
        "' == 'fastlio2_leveled' else '",
        cloud_topic,
        "'",
    ])
    planner_frame_id = PythonExpression([
        "'",
        leveled_frame_id,
        "' if '",
        planner_input_mode,
        "' == 'fastlio2_leveled' else 'camera_init'",
    ])
    px4_fastlio2_odom_topic = PythonExpression([
        "'",
        leveled_odom_topic,
        "' if '",
        planner_input_mode,
        "' == 'fastlio2_leveled' else '",
        odom_topic,
        "'",
    ])

    return LaunchDescription([
        DeclareLaunchArgument("drone_id", default_value="1"),
        DeclareLaunchArgument(
            "odom_topic",
            default_value=["/drone_", drone_id, "/aft_mapped_to_init"],
        ),
        DeclareLaunchArgument(
            "cloud_topic",
            default_value=["/drone_", drone_id, "/cloud_registered"],
        ),
        DeclareLaunchArgument("odom_bridge_type", default_value="fastlio2"),
        DeclareLaunchArgument("planner_input_mode", default_value="fastlio2_leveled"),
        DeclareLaunchArgument(
            "leveled_odom_topic",
            default_value=["/drone_", drone_id, "/aft_mapped_to_init_level"],
        ),
        DeclareLaunchArgument(
            "leveled_cloud_topic",
            default_value=["/drone_", drone_id, "/cloud_registered_level"],
        ),
        DeclareLaunchArgument(
            "leveled_frame_id", default_value="camera_init_level"
        ),
        DeclareLaunchArgument("target_system", default_value="2"),
        DeclareLaunchArgument(
            "robot_params_file", default_value=str(default_robot_params)
        ),
        Node(
            package="ego_planner",
            executable="ego_planner_node",
            name=drone_prefix + ["_ego_planner_node"],
            output="screen",
            parameters=[
                str(planner_config),
                {"grid_map/frame_id": planner_frame_id},
                {"manager/drone_id": ParameterValue(drone_id, value_type=int)},
            ],
            remappings=[
                ("odom_world", planner_odom_topic),
                ("planning/bspline", planning_prefix + ["/bspline"]),
                ("planning/data_display", planning_prefix + ["/data_display"]),
                ("planning/broadcast_bspline_from_planner", "/broadcast_bspline"),
                ("planning/broadcast_bspline_to_planner", "/broadcast_bspline"),
                ("/move_base_simple/goal", topic_prefix + ["/move_base_simple/goal"]),
                ("goal_point", plan_vis_prefix + ["/goal_point"]),
                ("global_list", plan_vis_prefix + ["/global_list"]),
                ("init_list", plan_vis_prefix + ["/init_list"]),
                ("optimal_list", plan_vis_prefix + ["/optimal_list"]),
                ("a_star_list", plan_vis_prefix + ["/a_star_list"]),
                ("grid_map/odom", planner_odom_topic),
                ("grid_map/cloud", planner_cloud_topic),
                (
                    "grid_map/occupancy_inflate",
                    ["/drone_", drone_id, "_grid/grid_map/occupancy_inflate"],
                ),
            ],
        ),
        Node(
            package="ego_planner",
            executable="traj_server",
            name=drone_prefix + ["_traj_server"],
            output="screen",
            parameters=[str(planner_config)],
            remappings=[
                ("planning/bspline", planning_prefix + ["/bspline"]),
                ("position_cmd", pos_cmd_topic),
            ],
        ),
        Node(
            package="fastlivo_to_px4_odometry",
            executable="fastlivo_to_px4_odometry_node",
            name="fastlivo_to_px4_odometry",
            output="screen",
            parameters=[str(fastlivo_px4_config)],
            condition=IfCondition(
                PythonExpression(["'", odom_bridge_type, "' == 'fastlivo'"])
            ),
        ),
        Node(
            package="fastlio2_to_px4_odometry",
            executable="fastlio2_to_px4_odometry_node",
            name="fastlio2_to_px4_odometry",
            output="screen",
            parameters=[
                str(fastlio2_px4_config),
                {"input_topic": px4_fastlio2_odom_topic},
                {"output_topic": fmu_in_prefix + ["/vehicle_visual_odometry"]},
            ],
            condition=IfCondition(
                PythonExpression(["'", odom_bridge_type, "' == 'fastlio2'"])
            ),
        ),
        Node(
            package="fastlio2_to_ego_swarm_leveling",
            executable="fastlio2_to_ego_swarm_leveling_node",
            name="fastlio2_to_ego_swarm_leveling",
            output="screen",
            parameters=[
                str(fastlio2_leveling_config),
                robot_params_file,
                {"input_odom_topic": odom_topic},
                {"input_cloud_topic": cloud_topic},
                {"output_odom_topic": leveled_odom_topic},
                {"output_cloud_topic": leveled_cloud_topic},
                {"output_child_frame_id": ["body_level_", drone_id]},
            ],
            condition=IfCondition(
                PythonExpression(["'", planner_input_mode, "' == 'fastlio2_leveled'"])
            ),
        ),
        Node(
            package="position_cmd_to_px4_bridge",
            executable="position_cmd_to_px4_bridge_node",
            name="position_cmd_to_px4_bridge",
            output="screen",
            parameters=[
                str(position_cmd_px4_config),
                robot_params_file,
                {"command_topic": pos_cmd_topic},
                {"offboard_mode_topic": fmu_in_prefix + ["/offboard_control_mode"]},
                {"trajectory_setpoint_topic": fmu_in_prefix + ["/trajectory_setpoint"]},
                {"vehicle_command_topic": fmu_in_prefix + ["/vehicle_command"]},
                {"vehicle_odometry_topic": fmu_out_prefix + ["/vehicle_odometry"]},
                {"fastlivo_odom_topic": px4_fastlio2_odom_topic},
                {"target_system": ParameterValue(target_system, value_type=int)},
            ],
        ),
    ])
