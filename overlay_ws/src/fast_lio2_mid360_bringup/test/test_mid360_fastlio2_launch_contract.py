from importlib.util import module_from_spec
from importlib.util import spec_from_file_location
from pathlib import Path

from launch import LaunchContext
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.utilities import perform_substitutions
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


REPO_ROOT = Path(__file__).resolve().parents[4]
PACKAGE_ROOT = REPO_ROOT / "overlay_ws" / "src" / "fast_lio2_mid360_bringup"
LAUNCH_PATH = PACKAGE_ROOT / "launch" / "mid360_fastlio2.launch.py"
EXPECTED_ARGUMENTS = {
    "livox_config_file",
    "fast_lio_params_file",
    "publish_freq",
    "xfer_format",
    "multi_topic",
    "frame_id",
    "odom_topic",
    "drone_id",
}


def _load_launch_module():
    spec = spec_from_file_location("mid360_fastlio2_launch", LAUNCH_PATH)
    assert spec is not None and spec.loader is not None

    module = module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _load_launch_description() -> LaunchDescription:
    module = _load_launch_module()

    def _fake_share_dir(package_name: str) -> str:
        if package_name == "fast_lio2_mid360_bringup":
            return str(PACKAGE_ROOT)
        if package_name == "livox_ros_driver2":
            return "/tmp/livox_ros_driver2"
        raise AssertionError(f"Unexpected package lookup: {package_name}")

    module.get_package_share_directory = _fake_share_dir
    return module.generate_launch_description()


def _to_text(substitutions) -> str:
    context = LaunchContext()
    if isinstance(substitutions, (list, tuple)):
        items = list(substitutions)
    else:
        items = [substitutions]
    return perform_substitutions(context, items)


def test_launch_file_exists() -> None:
    assert LAUNCH_PATH.exists(), f"Missing launch file: {LAUNCH_PATH}"


def test_launch_declares_expected_arguments() -> None:
    launch_description = _load_launch_description()
    arguments = [
        entity for entity in launch_description.entities
        if isinstance(entity, DeclareLaunchArgument)
    ]

    assert {argument.name for argument in arguments} == EXPECTED_ARGUMENTS


def test_launch_defaults_to_project_owned_drone1_livox_config() -> None:
    launch_description = _load_launch_description()
    arguments = [
        entity for entity in launch_description.entities
        if isinstance(entity, DeclareLaunchArgument)
    ]
    livox_argument = next(
        argument for argument in arguments if argument.name == "livox_config_file"
    )

    assert _to_text(livox_argument.default_value) == (
        str(PACKAGE_ROOT / "config" / "MID360_config_drone_1.json")
    )


def test_launch_starts_native_fast_lio_and_livox_nodes() -> None:
    launch_description = _load_launch_description()
    nodes = [
        entity for entity in launch_description.entities
        if isinstance(entity, Node)
    ]

    assert {
        (node.node_package, node.node_executable) for node in nodes
    } == {
        ("livox_ros_driver2", "livox_ros_driver2_node"),
        ("fast_lio", "fastlio_mapping"),
    }


def test_fast_lio_node_remaps_odometry_to_drone1_compat_topic_by_default() -> None:
    launch_description = _load_launch_description()
    nodes = [
        entity for entity in launch_description.entities
        if isinstance(entity, Node)
    ]
    fast_lio_node = next(
        node for node in nodes
        if (node.node_package, node.node_executable) == ("fast_lio", "fastlio_mapping")
    )

    remappings = _performed_remappings(fast_lio_node, {})
    assert remappings["/Odometry"] == "/drone_1/aft_mapped_to_init"


def _perform_with_launch_configurations(substitutions, launch_configurations: dict) -> str:
    from launch.utilities import normalize_to_list_of_substitutions
    from launch_ros.parameter_descriptions import ParameterValue

    context = LaunchContext()
    for name, value in launch_configurations.items():
        context.launch_configurations[name] = value
    if isinstance(substitutions, ParameterValue):
        substitutions = substitutions.value
    return perform_substitutions(
        context, normalize_to_list_of_substitutions(substitutions)
    )


def _performed_remappings(node: Node, launch_configurations: dict) -> dict:
    context_configurations = dict(launch_configurations)
    if "drone_id" not in context_configurations:
        context_configurations["drone_id"] = "1"
    if "odom_topic" not in context_configurations:
        drone_id = context_configurations.get("drone_id", "1")
        context_configurations["odom_topic"] = (
            f"/drone_{drone_id}/aft_mapped_to_init"
        )
    remappings = {}
    for src, dst in node._Node__remappings:
        remappings[
            _perform_with_launch_configurations(src, context_configurations)
        ] = _perform_with_launch_configurations(dst, context_configurations)
    return remappings


def _launch_node(package: str, executable: str) -> Node:
    launch_description = _load_launch_description()
    nodes = [
        entity for entity in launch_description.entities
        if isinstance(entity, Node)
    ]
    return next(
        node for node in nodes
        if (node.node_package, node.node_executable) == (package, executable)
    )


def test_drone_id_argument_defaults_to_drone1() -> None:
    launch_description = _load_launch_description()
    arguments = [
        entity for entity in launch_description.entities
        if isinstance(entity, DeclareLaunchArgument)
    ]
    drone_id_argument = next(
        argument for argument in arguments if argument.name == "drone_id"
    )
    assert _to_text(drone_id_argument.default_value) == "1"


def test_odom_topic_default_is_drone_prefixed() -> None:
    launch_description = _load_launch_description()
    arguments = [
        entity for entity in launch_description.entities
        if isinstance(entity, DeclareLaunchArgument)
    ]
    odom_argument = next(
        argument for argument in arguments if argument.name == "odom_topic"
    )
    assert _perform_with_launch_configurations(
        odom_argument.default_value, {"drone_id": "1"}
    ) == "/drone_1/aft_mapped_to_init"


def test_livox_topics_are_drone_prefixed() -> None:
    livox_node = _launch_node("livox_ros_driver2", "livox_ros_driver2_node")
    remappings = _performed_remappings(livox_node, {"drone_id": "1"})
    assert remappings["/livox/lidar"] == "/drone_1/livox/lidar"
    assert remappings["/livox/imu"] == "/drone_1/livox/imu"


def test_fast_lio_inputs_and_cloud_are_drone_prefixed() -> None:
    fast_lio_node = _launch_node("fast_lio", "fastlio_mapping")

    params = {}
    for entry in fast_lio_node._Node__parameters:
        if isinstance(entry, dict):
            for key, value in entry.items():
                params[_to_text(key)] = value
    launch_configurations = {"drone_id": "1"}
    assert _perform_with_launch_configurations(
        params["common.lid_topic"], launch_configurations
    ) == "/drone_1/livox/lidar"
    assert _perform_with_launch_configurations(
        params["common.imu_topic"], launch_configurations
    ) == "/drone_1/livox/imu"

    remappings = _performed_remappings(fast_lio_node, launch_configurations)
    assert remappings["/cloud_registered"] == "/drone_1/cloud_registered"
