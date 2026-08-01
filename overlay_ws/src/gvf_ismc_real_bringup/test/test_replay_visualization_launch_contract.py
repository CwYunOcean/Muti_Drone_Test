from importlib.util import module_from_spec
from importlib.util import spec_from_file_location
from pathlib import Path

import yaml
from ament_index_python.packages import PackageNotFoundError
from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext
from launch import LaunchDescription
from launch.utilities import perform_substitutions
from launch_ros.actions import Node


LAUNCH_PATH = Path(__file__).resolve().parents[1] / "launch" / "replay_visualization.launch.py"
EXPECTED_NODES = {
    (
        "fastlio2_to_ego_swarm_leveling",
        "fastlio2_to_ego_swarm_leveling_node",
    ),
    ("gvf_ismc_path_following", "gvf_reference_node"),
    ("rviz2", "rviz2"),
}


def _load_launch_description() -> LaunchDescription:
    spec = spec_from_file_location("replay_visualization_launch", LAUNCH_PATH)
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
        return original_get_package_share_directory(package_name)

    for package_name in (
        "gvf_ismc_real_bringup",
        "gvf_ismc_path_following",
        "fastlio2_to_ego_swarm_leveling",
    ):
        try:
            get_package_share_directory(package_name)
        except PackageNotFoundError:
            module.get_package_share_directory = _get_package_share_directory_for_test
            break

    return module.generate_launch_description()


def _flatten_substitutions(value):
    if isinstance(value, (list, tuple)):
        items = []
        for item in value:
            items.extend(_flatten_substitutions(item))
        return items
    return [value]


def _to_text(substitutions) -> str:
    context = LaunchContext()
    items = _flatten_substitutions(substitutions)
    return perform_substitutions(context, items)


def _load_yaml_scalar(text: str) -> object:
    return yaml.safe_load(text)


def _resolved_parameter_value(value):
    if isinstance(value, bool):
        return value
    return _load_yaml_scalar(_to_text(value))


def _perform_with_launch_configurations(
    substitutions,
    launch_configurations: dict,
) -> object:
    context = LaunchContext()
    configurations = {
        "drone_id": "1",
        "leveled_odom_topic": "/drone_1/aft_mapped_to_init_level",
        "use_sim_time": "true",
        **launch_configurations,
    }
    for name, value in configurations.items():
        context.launch_configurations[name] = value
    items = _flatten_substitutions(substitutions)
    return _load_yaml_scalar(perform_substitutions(context, items))


def _node_parameter_overrides(node: Node) -> dict:
    params = {}
    for entry in node._Node__parameters:
        if not isinstance(entry, dict):
            continue
        for key, value in entry.items():
            params[_to_text(key)] = value
    return params


def test_launch_exists() -> None:
    assert LAUNCH_PATH.exists(), f"Missing launch file: {LAUNCH_PATH}"


def test_launch_starts_expected_nodes_only() -> None:
    launch_description = _load_launch_description()
    nodes = [
        entity for entity in launch_description.entities
        if isinstance(entity, Node)
    ]

    assert len(nodes) == 3
    actual = {(node.node_package, node.node_executable) for node in nodes}
    assert actual == EXPECTED_NODES


def test_leveling_and_gvf_nodes_use_sim_time() -> None:
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
        ) == (
            "fastlio2_to_ego_swarm_leveling",
            "fastlio2_to_ego_swarm_leveling_node",
        )
    )
    gvf_node = next(
        node for node in nodes
        if (
            node.node_package,
            node.node_executable,
        ) == ("gvf_ismc_path_following", "gvf_reference_node")
    )

    leveling_params = _node_parameter_overrides(leveling_node)
    gvf_params = _node_parameter_overrides(gvf_node)

    assert "output_odom_topic" not in leveling_params
    assert "output_cloud_topic" not in leveling_params
    assert _perform_with_launch_configurations(
        leveling_params["use_sim_time"],
        {"use_sim_time": "true"},
    ) is True
    assert _perform_with_launch_configurations(
        gvf_params["use_sim_time"],
        {"use_sim_time": "true"},
    ) is True


def test_gvf_node_recomputes_reference_on_separate_topic() -> None:
    launch_description = _load_launch_description()
    nodes = [
        entity for entity in launch_description.entities
        if isinstance(entity, Node)
    ]
    gvf_node = next(
        node for node in nodes
        if (
            node.node_package,
            node.node_executable,
        ) == ("gvf_ismc_path_following", "gvf_reference_node")
    )

    params = _node_parameter_overrides(gvf_node)

    assert _perform_with_launch_configurations(
        params["odom_topic"],
        {"drone_id": "1", "leveled_odom_topic": "/drone_1/aft_mapped_to_init_level"},
    ) == "/drone_1/aft_mapped_to_init_level"
    assert _resolved_parameter_value(params["reference_topic"]) == "/gvf/reference_recomputed"


def test_rviz_node_uses_replay_config_and_sim_time() -> None:
    launch_description = _load_launch_description()
    nodes = [
        entity for entity in launch_description.entities
        if isinstance(entity, Node)
    ]
    rviz_node = next(
        node for node in nodes
        if (node.node_package, node.node_executable) == ("rviz2", "rviz2")
    )

    launch_source = LAUNCH_PATH.read_text(encoding="utf-8")
    assert 'arguments=["-d", rviz_config_file]' in launch_source
    params = _node_parameter_overrides(rviz_node)
    assert _perform_with_launch_configurations(
        params["use_sim_time"],
        {"use_sim_time": "true"},
    ) is True
