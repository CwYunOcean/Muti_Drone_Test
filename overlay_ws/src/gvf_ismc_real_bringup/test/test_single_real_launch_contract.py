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


LAUNCH_PATH = Path(__file__).resolve().parents[1] / "launch" / "single_real.launch.py"
EXPECTED_NODES = {
    ("fastlio2_to_px4_odometry", "fastlio2_to_px4_odometry_node"),
    (
        "fastlio2_to_ego_swarm_leveling",
        "fastlio2_to_ego_swarm_leveling_node",
    ),
    ("gvf_ismc_path_following", "gvf_reference_node"),
    ("gvf_ismc_path_following", "ismc_velocity_tracker_node"),
    ("position_cmd_to_px4_bridge", "position_cmd_to_px4_bridge_node"),
}


def _load_launch_description() -> LaunchDescription:
    spec = spec_from_file_location("single_real_launch", LAUNCH_PATH)
    assert spec is not None and spec.loader is not None
    module = module_from_spec(spec)
    spec.loader.exec_module(module)

    original_get_package_share_directory = module.get_package_share_directory

    def _get_package_share_directory_for_test(package_name: str) -> str:
        if package_name == "gvf_ismc_real_bringup":
            return str(Path(__file__).resolve().parents[1])
        if package_name == "gvf_ismc_path_following":
            return str(Path(__file__).resolve().parents[2] / "gvf_ismc_path_following")
        if package_name == "fastlio2_to_ego_swarm_leveling":
            return str(
                Path(__file__).resolve().parents[2] /
                "fastlio2_to_ego_swarm_leveling"
            )
        if package_name == "fastlio2_to_px4_odometry":
            return str(Path(__file__).resolve().parents[2] / "fastlio2_to_px4_odometry")
        if package_name == "position_cmd_to_px4_bridge":
            return str(Path(__file__).resolve().parents[2] / "position_cmd_to_px4_bridge")
        return original_get_package_share_directory(package_name)

    for package_name in (
        "gvf_ismc_real_bringup",
        "gvf_ismc_path_following",
        "fastlio2_to_ego_swarm_leveling",
        "fastlio2_to_px4_odometry",
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
    items = _flatten_substitutions(substitutions)
    return perform_substitutions(context, items)


def _perform_with_launch_configurations(
    substitutions,
    launch_configurations: dict,
) -> object:
    from launch.utilities import normalize_to_list_of_substitutions
    from launch_ros.parameter_descriptions import ParameterValue

    context = LaunchContext()
    configurations = {
        "drone_id": "1",
        "target_system": "2",
        "odom_topic": "/drone_1/aft_mapped_to_init",
        "leveled_odom_topic": "/drone_1/aft_mapped_to_init_level",
        "use_acceleration_feedforward": "false",
        **launch_configurations,
    }
    for name, value in configurations.items():
        context.launch_configurations[name] = value
    if isinstance(substitutions, ParameterValue):
        substitutions = substitutions.value
    return _load_yaml_scalar(
        perform_substitutions(
            context, normalize_to_list_of_substitutions(substitutions)
        )
    )


def _flatten_substitutions(value):
    if isinstance(value, (list, tuple)):
        items = []
        for item in value:
            items.extend(_flatten_substitutions(item))
        return items
    return [value]


def _resolved_parameter_value(value):
    if isinstance(value, bool):
        return value
    return _load_yaml_scalar(_to_text(value))


def _load_yaml_scalar(text: str) -> object:
    return yaml.safe_load(text)


def _node_parameter_overrides(node: Node) -> dict:
    params = {}
    for entry in node._Node__parameters:
        if not isinstance(entry, dict):
            continue
        for key, value in entry.items():
            params[_to_text(key)] = value
    return params


def _launch_argument_defaults() -> dict:
    launch_description = _load_launch_description()
    defaults = {}
    context = LaunchContext()
    for entity in launch_description.entities:
        if not isinstance(entity, DeclareLaunchArgument):
            continue
        defaults[entity.name] = perform_substitutions(
            context,
            _flatten_substitutions(entity.default_value),
        )
        context.launch_configurations[entity.name] = defaults[entity.name]
    return defaults


def test_launch_exists() -> None:
    assert LAUNCH_PATH.exists(), f"Missing launch file: {LAUNCH_PATH}"


def test_launch_starts_expected_nodes() -> None:
    launch_description = _load_launch_description()
    nodes = [
        entity for entity in launch_description.entities
        if isinstance(entity, Node)
    ]

    assert len(nodes) == 5
    actual = {(node.node_package, node.node_executable) for node in nodes}
    assert actual == EXPECTED_NODES


def test_launch_defaults_to_drone1_and_px4_system2() -> None:
    defaults = _launch_argument_defaults()

    assert defaults["drone_id"] == "1"
    assert defaults["target_system"] == "2"
    assert defaults["odom_topic"] == "/drone_1/aft_mapped_to_init"
    assert defaults["leveled_odom_topic"] == "/drone_1/aft_mapped_to_init_level"


def test_fastlio2_px4_odometry_uses_leveled_input_topic() -> None:
    launch_description = _load_launch_description()
    nodes = [
        entity for entity in launch_description.entities
        if isinstance(entity, Node)
    ]
    px4_odometry_node = next(
        node for node in nodes
        if (
            node.node_package,
            node.node_executable,
        ) == ("fastlio2_to_px4_odometry", "fastlio2_to_px4_odometry_node")
    )

    params = _node_parameter_overrides(px4_odometry_node)

    assert "input_topic" in params
    assert _perform_with_launch_configurations(
        params["input_topic"],
        {"drone_id": "1", "leveled_odom_topic": "/drone_1/aft_mapped_to_init_level"},
    ) == (
        "/drone_1/aft_mapped_to_init_level"
    )
    assert _perform_with_launch_configurations(
        params["output_topic"],
        {"drone_id": "1"},
    ) == (
        "/drone_1/fmu/in/vehicle_visual_odometry"
    )


def test_leveling_and_gvf_nodes_use_drone1_topics() -> None:
    launch_description = _load_launch_description()
    nodes = [
        entity for entity in launch_description.entities
        if isinstance(entity, Node)
    ]
    leveling_node = next(
        node for node in nodes
        if (
            node.node_package,
            node.node_executable,
        ) == ("fastlio2_to_ego_swarm_leveling", "fastlio2_to_ego_swarm_leveling_node")
    )
    gvf_node = next(
        node for node in nodes
        if (
            node.node_package,
            node.node_executable,
        ) == ("gvf_ismc_path_following", "gvf_reference_node")
    )
    ismc_node = next(
        node for node in nodes
        if (
            node.node_package,
            node.node_executable,
        ) == ("gvf_ismc_path_following", "ismc_velocity_tracker_node")
    )

    context = {
        "drone_id": "1",
        "odom_topic": "/drone_1/aft_mapped_to_init",
        "leveled_odom_topic": "/drone_1/aft_mapped_to_init_level",
    }
    leveling_params = _node_parameter_overrides(leveling_node)
    gvf_params = _node_parameter_overrides(gvf_node)
    ismc_params = _node_parameter_overrides(ismc_node)

    assert _perform_with_launch_configurations(
        leveling_params["input_odom_topic"],
        context,
    ) == "/drone_1/aft_mapped_to_init"
    assert _perform_with_launch_configurations(
        leveling_params["output_odom_topic"],
        context,
    ) == "/drone_1/aft_mapped_to_init_level"
    assert _perform_with_launch_configurations(
        gvf_params["odom_topic"],
        context,
    ) == "/drone_1/aft_mapped_to_init_level"
    assert _perform_with_launch_configurations(
        ismc_params["odom_topic"],
        context,
    ) == "/drone_1/aft_mapped_to_init_level"
    assert _perform_with_launch_configurations(
        ismc_params["command_topic"],
        context,
    ) == "/drone_1_planning/pos_cmd"


def test_position_cmd_bridge_uses_real_hardware_overrides() -> None:
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
        ) == ("position_cmd_to_px4_bridge", "position_cmd_to_px4_bridge_node")
    )

    params = _node_parameter_overrides(bridge_node)

    assert _perform_with_launch_configurations(
        params["command_topic"],
        {"drone_id": "1"},
    ) == "/drone_1_planning/pos_cmd"
    assert _perform_with_launch_configurations(
        params["offboard_mode_topic"],
        {"drone_id": "1"},
    ) == "/drone_1/fmu/in/offboard_control_mode"
    assert _perform_with_launch_configurations(
        params["trajectory_setpoint_topic"],
        {"drone_id": "1"},
    ) == "/drone_1/fmu/in/trajectory_setpoint"
    assert _perform_with_launch_configurations(
        params["vehicle_command_topic"],
        {"drone_id": "1"},
    ) == "/drone_1/fmu/in/vehicle_command"
    assert _perform_with_launch_configurations(
        params["vehicle_odometry_topic"],
        {"drone_id": "1"},
    ) == "/drone_1/fmu/out/vehicle_odometry"
    assert _perform_with_launch_configurations(
        params["fastlivo_odom_topic"],
        {"drone_id": "1", "leveled_odom_topic": "/drone_1/aft_mapped_to_init_level"},
    ) == (
        "/drone_1/aft_mapped_to_init_level"
    )
    assert params["position_control_enabled"] is False
    assert params["velocity_control_enabled"] is True
    assert params["auto_request_offboard_and_arm"] is False
    assert _perform_with_launch_configurations(
        params["target_system"],
        {"target_system": "2"},
    ) == 2
    assert _perform_with_launch_configurations(
        params["use_acceleration_feedforward"],
        {"use_acceleration_feedforward": "false"},
    ) is False
    assert _perform_with_launch_configurations(
        params["use_acceleration_feedforward"],
        {"use_acceleration_feedforward": "true"},
    ) is True
