from importlib.util import module_from_spec
from importlib.util import spec_from_file_location
from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from ament_index_python.packages import PackageNotFoundError
from launch import LaunchContext
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.utilities import perform_substitutions
from launch_ros.actions import Node


PACKAGE_NAME = "ego_swarm_real_bringup"
CONFIG_PATH = Path(__file__).resolve().parents[1] / "config" / "ego_planner_real.yaml"
LAUNCH_PATH = Path(__file__).resolve().parents[1] / "launch" / "single_real.launch.py"
EXPECTED_PACKAGES = {
    ("ego_planner", "ego_planner_node"),
    ("ego_planner", "traj_server"),
    ("fastlivo_to_px4_odometry", "fastlivo_to_px4_odometry_node"),
    ("fastlio2_to_px4_odometry", "fastlio2_to_px4_odometry_node"),
    ("fastlio2_to_ego_swarm_leveling", "fastlio2_to_ego_swarm_leveling_node"),
    ("position_cmd_to_px4_bridge", "position_cmd_to_px4_bridge_node"),
}


def _load_launch_description() -> LaunchDescription:
    spec = spec_from_file_location("single_real_launch", LAUNCH_PATH)
    assert spec is not None and spec.loader is not None
    module = module_from_spec(spec)
    spec.loader.exec_module(module)

    original_get_package_share_directory = module.get_package_share_directory

    def _get_package_share_directory_for_test(package_name: str) -> str:
        if package_name == "ego_swarm_real_bringup":
            return str(Path(__file__).resolve().parents[1])
        if package_name == "fastlivo_to_px4_odometry":
            return str(
                Path(__file__).resolve().parents[2] /
                "fastlivo_to_px4_odometry"
            )
        if package_name == "fastlio2_to_px4_odometry":
            return str(
                Path(__file__).resolve().parents[2] /
                "fastlio2_to_px4_odometry"
            )
        if package_name == "fastlio2_to_ego_swarm_leveling":
            return str(
                Path(__file__).resolve().parents[2] /
                "fastlio2_to_ego_swarm_leveling"
            )
        if package_name == "position_cmd_to_px4_bridge":
            return str(
                Path(__file__).resolve().parents[2] /
                "position_cmd_to_px4_bridge"
            )
        return original_get_package_share_directory(package_name)

    for package_name in (
        "ego_swarm_real_bringup",
        "fastlivo_to_px4_odometry",
        "fastlio2_to_px4_odometry",
        "fastlio2_to_ego_swarm_leveling",
        "position_cmd_to_px4_bridge",
    ):
        try:
            get_package_share_directory(package_name)
        except PackageNotFoundError:
            module.get_package_share_directory = _get_package_share_directory_for_test
            break

    return module.generate_launch_description()


def _to_text(substitutions) -> str:
    context = LaunchContext()
    if isinstance(substitutions, (list, tuple)):
        items = list(substitutions)
    else:
        items = [substitutions]
    return perform_substitutions(context, items)


def _perform_with_launch_configurations(substitutions, launch_configurations: dict) -> str:
    from launch.utilities import normalize_to_list_of_substitutions
    from launch_ros.parameter_descriptions import ParameterValue

    context = LaunchContext()
    configurations = {
        "drone_id": "1",
        "odom_topic": "/drone_1/aft_mapped_to_init",
        "cloud_topic": "/drone_1/cloud_registered",
        "planner_input_mode": "fastlio2_leveled",
        "leveled_odom_topic": "/drone_1/aft_mapped_to_init_level",
        "leveled_cloud_topic": "/drone_1/cloud_registered_level",
        "leveled_frame_id": "camera_init_level",
        **launch_configurations,
    }
    for name, value in configurations.items():
        context.launch_configurations[name] = value
    if isinstance(substitutions, ParameterValue):
        substitutions = substitutions.value
    return perform_substitutions(
        context, normalize_to_list_of_substitutions(substitutions)
    )


def _node_parameter_overrides(node: Node) -> dict:
    params = {}
    for entry in node._Node__parameters:
        if not isinstance(entry, dict):
            continue
        assert len(entry) == 1
        key, value = next(iter(entry.items()))
        params[_to_text(key)] = value
    return params


def test_config_exists() -> None:
    assert CONFIG_PATH.exists(), f"Missing config file: {CONFIG_PATH}"


def test_launch_exists() -> None:
    assert LAUNCH_PATH.exists(), f"Missing launch file: {LAUNCH_PATH}"


def test_config_contract_has_manual_goal_and_expected_topics() -> None:
    config = yaml.safe_load(CONFIG_PATH.read_text())
    assert "/**" in config, "config must use /** wildcard so any drone_id node matches"
    params = config["/**"]["ros__parameters"]

    assert params["fsm/flight_type"] == 1
    assert params["fsm/realworld_experiment"] is False
    assert params["grid_map/frame_id"] == "camera_init"
    assert params["traj_server/time_forward"] == 1.0


def _launch_nodes() -> list:
    launch_description = _load_launch_description()
    return [
        entity for entity in launch_description.entities
        if isinstance(entity, Node)
    ]


def _planner_node(nodes: list) -> Node:
    return next(
        node for node in nodes
        if (node.node_package, node.node_executable) == ("ego_planner", "ego_planner_node")
    )


def _performed_remappings(node: Node, launch_configurations: dict) -> dict:
    configurations = {
        "drone_id": "1",
        "odom_topic": "/drone_1/aft_mapped_to_init",
        "cloud_topic": "/drone_1/cloud_registered",
        "planner_input_mode": "fastlio2_leveled",
        "leveled_odom_topic": "/drone_1/aft_mapped_to_init_level",
        "leveled_cloud_topic": "/drone_1/cloud_registered_level",
        "leveled_frame_id": "camera_init_level",
        **launch_configurations,
    }
    remappings = {}
    for src, dst in node._Node__remappings:
        remappings[
            _perform_with_launch_configurations(src, configurations)
        ] = _perform_with_launch_configurations(dst, configurations)
    return remappings


def test_launch_declares_drone_id_argument_default_drone1() -> None:
    launch_description = _load_launch_description()
    arguments = [
        entity for entity in launch_description.entities
        if isinstance(entity, DeclareLaunchArgument)
    ]
    drone_id_argument = next(
        argument for argument in arguments if argument.name == "drone_id"
    )
    assert _to_text(drone_id_argument.default_value) == "1"


def test_default_topics_and_names_are_for_drone1() -> None:
    nodes = _launch_nodes()

    planner = _planner_node(nodes)
    assert _perform_with_launch_configurations(
        planner._Node__node_name, {}
    ) == "drone_1_ego_planner_node"
    planner_remappings = _performed_remappings(planner, {})
    assert planner_remappings["odom_world"] == "/drone_1/aft_mapped_to_init_level"
    assert planner_remappings["planning/bspline"] == "/drone_1_planning/bspline"
    assert planner_remappings["/move_base_simple/goal"] == "/drone_1/move_base_simple/goal"

    bridge = next(
        node for node in nodes
        if (node.node_package, node.node_executable)
        == ("position_cmd_to_px4_bridge", "position_cmd_to_px4_bridge_node")
    )
    params = _node_parameter_overrides(bridge)
    assert _perform_with_launch_configurations(
        params["command_topic"], {}
    ) == "/drone_1_planning/pos_cmd"
    assert _perform_with_launch_configurations(
        params["offboard_mode_topic"], {}
    ) == "/drone_1/fmu/in/offboard_control_mode"
    assert _perform_with_launch_configurations(
        params["trajectory_setpoint_topic"], {}
    ) == "/drone_1/fmu/in/trajectory_setpoint"
    assert _perform_with_launch_configurations(
        params["vehicle_command_topic"], {}
    ) == "/drone_1/fmu/in/vehicle_command"
    assert _perform_with_launch_configurations(
        params["vehicle_odometry_topic"], {}
    ) == "/drone_1/fmu/out/vehicle_odometry"


def test_planner_broadcast_bspline_remapped_to_shared_topic() -> None:
    planner = _planner_node(_launch_nodes())
    remappings = _performed_remappings(planner, {"drone_id": "1"})
    assert remappings["planning/broadcast_bspline_from_planner"] == "/broadcast_bspline"
    assert remappings["planning/broadcast_bspline_to_planner"] == "/broadcast_bspline"


def test_launch_topics_and_names_follow_drone_id() -> None:
    nodes = _launch_nodes()
    launch_configurations = {"drone_id": "1"}

    planner = _planner_node(nodes)
    assert _perform_with_launch_configurations(
        planner._Node__node_name, launch_configurations
    ) == "drone_1_ego_planner_node"
    planner_remappings = _performed_remappings(planner, launch_configurations)
    assert planner_remappings["planning/bspline"] == "/drone_1_planning/bspline"

    traj_server = next(
        node for node in nodes
        if (node.node_package, node.node_executable) == ("ego_planner", "traj_server")
    )
    assert _perform_with_launch_configurations(
        traj_server._Node__node_name, launch_configurations
    ) == "drone_1_traj_server"
    traj_server_remappings = _performed_remappings(traj_server, launch_configurations)
    assert traj_server_remappings["position_cmd"] == "/drone_1_planning/pos_cmd"
    assert traj_server_remappings["planning/bspline"] == "/drone_1_planning/bspline"


def test_planner_drone_id_parameter_follows_launch_argument() -> None:
    planner = _planner_node(_launch_nodes())
    params = _node_parameter_overrides(planner)
    assert "manager/drone_id" in params
    assert _perform_with_launch_configurations(
        params["manager/drone_id"], {"drone_id": "1"}
    ) == "1"


def test_planner_config_defaults_to_drone1_for_direct_node_runs() -> None:
    config = yaml.safe_load(CONFIG_PATH.read_text())
    params = config["/**"]["ros__parameters"]

    assert params["manager/drone_id"] == 1


def test_px4_bridge_command_topic_follows_drone_id() -> None:
    nodes = _launch_nodes()
    bridge = next(
        node for node in nodes
        if (node.node_package, node.node_executable)
        == ("position_cmd_to_px4_bridge", "position_cmd_to_px4_bridge_node")
    )
    params = _node_parameter_overrides(bridge)
    assert "command_topic" in params
    assert _perform_with_launch_configurations(
        params["command_topic"], {"drone_id": "1"}
    ) == "/drone_1_planning/pos_cmd"


def test_launch_starts_expected_nodes() -> None:
    launch_description = _load_launch_description()
    nodes = [
        entity for entity in launch_description.entities
        if isinstance(entity, Node)
    ]
    assert len(nodes) == 6
    actual = {(node.node_package, node.node_executable) for node in nodes}
    assert actual == EXPECTED_PACKAGES


def test_launch_declares_bridge_type_argument_with_fastlio2_default() -> None:
    launch_description = _load_launch_description()
    arguments = [
        entity for entity in launch_description.entities
        if isinstance(entity, DeclareLaunchArgument)
    ]

    actual_arguments = {argument.name for argument in arguments}
    assert "odom_bridge_type" in actual_arguments

    bridge_argument = next(
        argument for argument in arguments
        if argument.name == "odom_bridge_type"
    )
    assert _to_text(bridge_argument.default_value) == "fastlio2"


def test_launch_declares_planner_input_mode_argument_with_leveled_default() -> None:
    launch_description = _load_launch_description()
    arguments = [
        entity for entity in launch_description.entities
        if isinstance(entity, DeclareLaunchArgument)
    ]

    planner_mode_argument = next(
        argument for argument in arguments
        if argument.name == "planner_input_mode"
    )
    assert _to_text(planner_mode_argument.default_value) == "fastlio2_leveled"


def test_fastlio2_px4_bridge_uses_leveled_odometry_when_planner_is_leveled() -> None:
    launch_description = _load_launch_description()
    nodes = [
        entity for entity in launch_description.entities
        if isinstance(entity, Node)
    ]
    bridge_node = next(
        node for node in nodes
        if (
            node.node_package,
            node.node_executable,
        ) == ("fastlio2_to_px4_odometry", "fastlio2_to_px4_odometry_node")
    )

    params = _node_parameter_overrides(bridge_node)

    assert "input_topic" in params
    assert _perform_with_launch_configurations(
        params["input_topic"],
        {
            "odom_topic": "/aft_mapped_to_init",
            "leveled_odom_topic": "/aft_mapped_to_init_level",
            "planner_input_mode": "fastlio2_leveled",
        },
    ) == "/aft_mapped_to_init_level"
    assert _perform_with_launch_configurations(
        params["input_topic"],
        {
            "odom_topic": "/aft_mapped_to_init",
            "leveled_odom_topic": "/aft_mapped_to_init_level",
            "planner_input_mode": "raw",
        },
    ) == "/aft_mapped_to_init"


def _launch_arguments() -> list:
    launch_description = _load_launch_description()
    return [
        entity for entity in launch_description.entities
        if isinstance(entity, DeclareLaunchArgument)
    ]


def _argument_default(name: str):
    return next(
        argument for argument in _launch_arguments() if argument.name == name
    ).default_value


def test_slam_topic_defaults_are_drone_prefixed() -> None:
    launch_configurations = {"drone_id": "1"}
    assert _perform_with_launch_configurations(
        _argument_default("odom_topic"), launch_configurations
    ) == "/drone_1/aft_mapped_to_init"
    assert _perform_with_launch_configurations(
        _argument_default("cloud_topic"), launch_configurations
    ) == "/drone_1/cloud_registered"
    assert _perform_with_launch_configurations(
        _argument_default("leveled_odom_topic"), launch_configurations
    ) == "/drone_1/aft_mapped_to_init_level"
    assert _perform_with_launch_configurations(
        _argument_default("leveled_cloud_topic"), launch_configurations
    ) == "/drone_1/cloud_registered_level"


def _node_by_name(package: str, executable: str) -> Node:
    return next(
        node for node in _launch_nodes()
        if (node.node_package, node.node_executable) == (package, executable)
    )


def test_px4_bridge_fmu_topics_are_drone_prefixed() -> None:
    bridge = _node_by_name(
        "position_cmd_to_px4_bridge", "position_cmd_to_px4_bridge_node"
    )
    params = _node_parameter_overrides(bridge)
    launch_configurations = {"drone_id": "1"}
    assert _perform_with_launch_configurations(
        params["offboard_mode_topic"], launch_configurations
    ) == "/drone_1/fmu/in/offboard_control_mode"
    assert _perform_with_launch_configurations(
        params["trajectory_setpoint_topic"], launch_configurations
    ) == "/drone_1/fmu/in/trajectory_setpoint"
    assert _perform_with_launch_configurations(
        params["vehicle_command_topic"], launch_configurations
    ) == "/drone_1/fmu/in/vehicle_command"
    assert _perform_with_launch_configurations(
        params["vehicle_odometry_topic"], launch_configurations
    ) == "/drone_1/fmu/out/vehicle_odometry"


def test_px4_target_system_is_launch_parameter() -> None:
    assert _perform_with_launch_configurations(
        _argument_default("target_system"), {}
    ) == "2"

    bridge = _node_by_name(
        "position_cmd_to_px4_bridge", "position_cmd_to_px4_bridge_node"
    )
    params = _node_parameter_overrides(bridge)
    assert _perform_with_launch_configurations(
        params["target_system"], {"target_system": "12"}
    ) == "12"


def test_launch_accepts_local_robot_parameter_file() -> None:
    from launch_ros.parameter_descriptions import ParameterFile

    default_path = _to_text(_argument_default("robot_params_file"))
    assert default_path.endswith("config/robot_overrides.yaml")

    for package, executable in (
        ("fastlio2_to_ego_swarm_leveling", "fastlio2_to_ego_swarm_leveling_node"),
        ("position_cmd_to_px4_bridge", "position_cmd_to_px4_bridge_node"),
    ):
        node = _node_by_name(package, executable)
        parameter_files = []
        for entry in node._Node__parameters:
            if isinstance(entry, ParameterFile):
                entry = entry._ParameterFile__param_file
            if not isinstance(entry, dict):
                parameter_files.append(
                    _perform_with_launch_configurations(
                        entry, {"robot_params_file": "/tmp/robot.params.yaml"}
                    )
                )
        assert "/tmp/robot.params.yaml" in parameter_files


def test_px4_bridge_slam_health_topic_follows_planner_input() -> None:
    bridge = _node_by_name(
        "position_cmd_to_px4_bridge", "position_cmd_to_px4_bridge_node"
    )
    params = _node_parameter_overrides(bridge)
    assert _perform_with_launch_configurations(
        params["fastlivo_odom_topic"],
        {
            "drone_id": "1",
            "odom_topic": "/drone_1/aft_mapped_to_init",
            "leveled_odom_topic": "/drone_1/aft_mapped_to_init_level",
            "planner_input_mode": "fastlio2_leveled",
        },
    ) == "/drone_1/aft_mapped_to_init_level"


def test_fastlio2_px4_output_topic_is_drone_prefixed() -> None:
    node = _node_by_name(
        "fastlio2_to_px4_odometry", "fastlio2_to_px4_odometry_node"
    )
    params = _node_parameter_overrides(node)
    assert _perform_with_launch_configurations(
        params["output_topic"], {"drone_id": "1"}
    ) == "/drone_1/fmu/in/vehicle_visual_odometry"


def test_leveling_node_topics_follow_launch_arguments() -> None:
    node = _node_by_name(
        "fastlio2_to_ego_swarm_leveling", "fastlio2_to_ego_swarm_leveling_node"
    )
    params = _node_parameter_overrides(node)
    launch_configurations = {
        "odom_topic": "/drone_1/aft_mapped_to_init",
        "cloud_topic": "/drone_1/cloud_registered",
        "leveled_odom_topic": "/drone_1/aft_mapped_to_init_level",
        "leveled_cloud_topic": "/drone_1/cloud_registered_level",
    }
    assert _perform_with_launch_configurations(
        params["input_odom_topic"], launch_configurations
    ) == "/drone_1/aft_mapped_to_init"
    assert _perform_with_launch_configurations(
        params["input_cloud_topic"], launch_configurations
    ) == "/drone_1/cloud_registered"
    assert _perform_with_launch_configurations(
        params["output_odom_topic"], launch_configurations
    ) == "/drone_1/aft_mapped_to_init_level"
    assert _perform_with_launch_configurations(
        params["output_cloud_topic"], launch_configurations
    ) == "/drone_1/cloud_registered_level"


def test_manual_goal_topic_is_drone_prefixed() -> None:
    planner = _planner_node(_launch_nodes())
    remappings = _performed_remappings(planner, {"drone_id": "1"})
    assert remappings["/move_base_simple/goal"] == "/drone_1/move_base_simple/goal"


def test_leveling_child_frame_is_drone_suffixed() -> None:
    node = _node_by_name(
        "fastlio2_to_ego_swarm_leveling", "fastlio2_to_ego_swarm_leveling_node"
    )
    params = _node_parameter_overrides(node)
    assert _perform_with_launch_configurations(
        params["output_child_frame_id"], {"drone_id": "1"}
    ) == "body_level_1"
