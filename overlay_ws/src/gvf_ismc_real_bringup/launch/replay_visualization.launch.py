from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _find_repo_script(script_name: str) -> Path:
    package_share = Path(
        get_package_share_directory("gvf_ismc_real_bringup")
    ).resolve()
    for candidate in [package_share, *package_share.parents]:
        script_path = candidate / "scripts" / script_name
        if script_path.exists():
            return script_path
    raise FileNotFoundError(f"Unable to locate scripts/{script_name} from {package_share}")


def generate_launch_description() -> LaunchDescription:
    gvf_ismc_config = Path(
        get_package_share_directory("gvf_ismc_path_following")
    ) / "config" / "gvf_ismc_path_following.yaml"
    fastlio2_leveling_config = Path(
        get_package_share_directory("fastlio2_to_ego_swarm_leveling")
    ) / "config" / "fastlio2_to_ego_swarm_leveling.yaml"
    default_rviz_config = _find_repo_script("gvf_ismc_bag_replay.rviz")

    use_sim_time = LaunchConfiguration("use_sim_time")
    rviz_config_file = LaunchConfiguration("rviz_config_file")
    drone_id = LaunchConfiguration("drone_id")
    leveled_odom_topic = LaunchConfiguration("leveled_odom_topic")

    return LaunchDescription([
        DeclareLaunchArgument("drone_id", default_value="1"),
        DeclareLaunchArgument(
            "leveled_odom_topic",
            default_value=["/drone_", drone_id, "/aft_mapped_to_init_level"],
        ),
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true",
        ),
        DeclareLaunchArgument(
            "rviz_config_file",
            default_value=str(default_rviz_config),
        ),
        Node(
            package="fastlio2_to_ego_swarm_leveling",
            executable="fastlio2_to_ego_swarm_leveling_node",
            name="fastlio2_to_ego_swarm_leveling",
            output="screen",
            parameters=[
                str(fastlio2_leveling_config),
                {"use_sim_time": use_sim_time},
            ],
        ),
        Node(
            package="gvf_ismc_path_following",
            executable="gvf_reference_node",
            name="gvf_reference_node",
            output="screen",
            parameters=[
                str(gvf_ismc_config),
                {
                    "use_sim_time": use_sim_time,
                    "odom_topic": leveled_odom_topic,
                    "reference_topic": "/gvf/reference_recomputed",
                },
            ],
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="gvf_ismc_replay_rviz",
            output="screen",
            arguments=["-d", rviz_config_file],
            parameters=[{"use_sim_time": use_sim_time}],
        ),
    ])
