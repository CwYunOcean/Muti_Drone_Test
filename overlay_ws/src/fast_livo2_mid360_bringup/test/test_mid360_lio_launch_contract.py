from importlib.util import module_from_spec
from importlib.util import spec_from_file_location
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from ament_index_python.packages import PackageNotFoundError
from launch import LaunchContext
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.substitutions import TextSubstitution
from launch.utilities import perform_substitutions
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile


PACKAGE_NAME = "fast_livo2_mid360_bringup"
EXPECTED_ARGUMENTS = {
    "livox_config_file",
    "livo_params_file",
    "camera_params_file",
    "publish_freq",
    "xfer_format",
    "multi_topic",
    "frame_id",
    "base_link_pitch_rad",
}
EXPECTED_NODE_WIRING = {
    ("livox_ros_driver2", "livox_ros_driver2_node"),
    ("fast_livo", "fastlivo_mapping"),
    ("tf2_ros", "static_transform_publisher"),
}
EXPECTED_ARGUMENT_DEFAULTS = {
    "livox_config_file": "MID360_config_drone_1.json",
    "livo_params_file": "fast_livo2_mid360_lio.yaml",
    "camera_params_file": "camera_startup_pinhole.yaml",
    "publish_freq": "10.0",
    "xfer_format": "1",
    "multi_topic": "0",
    "frame_id": "livox_frame",
    "base_link_pitch_rad": "-0.519",
}


def _resolve_installed_launch_path() -> Path:
    package_share = Path(get_package_share_directory(PACKAGE_NAME))
    return package_share / "launch" / "mid360_lio.launch.py"


def _load_installed_launch_module():
    launch_path = _resolve_installed_launch_path()
    spec = spec_from_file_location("mid360_lio_launch", launch_path)
    assert spec is not None and spec.loader is not None

    module = module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _load_launch_description() -> LaunchDescription:
    module = _load_installed_launch_module()
    original_get_package_share_directory = module.get_package_share_directory

    def _get_package_share_directory_for_test(package_name: str) -> str:
        if package_name == "livox_ros_driver2":
            return "/tmp/livox_ros_driver2"
        return original_get_package_share_directory(package_name)

    try:
        get_package_share_directory("livox_ros_driver2")
    except PackageNotFoundError:
        module.get_package_share_directory = _get_package_share_directory_for_test

    return module.generate_launch_description()


def _to_text(substitutions) -> str:
    context = LaunchContext()
    if isinstance(substitutions, (list, tuple)):
        items = list(substitutions)
    else:
        items = [substitutions]
    return perform_substitutions(context, items)


def _launch_config_name(value: LaunchConfiguration) -> str:
    return _to_text(value.variable_name)


def _node_parameters_by_name(node: Node) -> dict:
    params = {}
    for raw_entry in node._Node__parameters:
        if not isinstance(raw_entry, dict):
            continue

        assert len(raw_entry) == 1
        raw_key, raw_value = next(iter(raw_entry.items()))
        param_name = _to_text(raw_key)
        params[param_name] = raw_value
    return params


def _expect_single_launch_config(value, name: str) -> None:
    assert isinstance(value, tuple)
    assert len(value) == 1
    assert isinstance(value[0], LaunchConfiguration)
    assert _launch_config_name(value[0]) == name


def test_installed_launch_file_exists() -> None:
    launch_path = _resolve_installed_launch_path()
    assert launch_path.exists(), f"Missing installed launch file: {launch_path}"


def test_launch_description_declares_expected_arguments() -> None:
    launch_description = _load_launch_description()

    arg_actions = [
        entity for entity in launch_description.entities
        if isinstance(entity, DeclareLaunchArgument)
    ]
    assert len(arg_actions) == 8

    actual_arguments = {action.name for action in arg_actions}
    assert actual_arguments == EXPECTED_ARGUMENTS


def test_launch_description_argument_defaults_match_contract() -> None:
    launch_description = _load_launch_description()
    arg_actions = [
        entity for entity in launch_description.entities
        if isinstance(entity, DeclareLaunchArgument)
    ]

    defaults = {
        action.name: _to_text(action.default_value)
        for action in arg_actions
    }

    for name, expected in EXPECTED_ARGUMENT_DEFAULTS.items():
        actual = defaults[name]
        if name in {
            "livox_config_file",
            "livo_params_file",
            "camera_params_file",
        }:
            assert actual.endswith(expected), f"{name} default mismatch: {actual}"
        else:
            assert actual == expected, f"{name} default mismatch: {actual}"


def test_launch_description_starts_expected_nodes() -> None:
    launch_description = _load_launch_description()
    node_actions = [
        entity for entity in launch_description.entities
        if isinstance(entity, Node)
    ]
    assert len(node_actions) == 4

    actual_wiring = {
        (node.node_package, node.node_executable) for node in node_actions
    }
    assert actual_wiring == EXPECTED_NODE_WIRING


def test_launch_description_starts_two_static_transform_publishers() -> None:
    launch_description = _load_launch_description()
    node_actions = [
        entity for entity in launch_description.entities
        if isinstance(entity, Node)
    ]

    static_tf_nodes = [
        node for node in node_actions
        if (node.node_package, node.node_executable) ==
        ("tf2_ros", "static_transform_publisher")
    ]
    assert len(static_tf_nodes) == 2


def test_livox_node_parameter_wiring_matches_contract() -> None:
    launch_description = _load_launch_description()
    nodes = [
        entity for entity in launch_description.entities
        if isinstance(entity, Node)
    ]
    livox_node = next(
        node for node in nodes
        if (node.node_package, node.node_executable) ==
        ("livox_ros_driver2", "livox_ros_driver2_node")
    )

    params = _node_parameters_by_name(livox_node)
    assert set(params.keys()) == {
        "xfer_format",
        "multi_topic",
        "data_src",
        "publish_freq",
        "output_data_type",
        "frame_id",
        "user_config_path",
        "cmdline_input_bd_code",
    }

    _expect_single_launch_config(params["xfer_format"], "xfer_format")
    _expect_single_launch_config(params["multi_topic"], "multi_topic")
    assert params["data_src"] == 0
    _expect_single_launch_config(params["publish_freq"], "publish_freq")
    assert params["output_data_type"] == 0
    _expect_single_launch_config(params["frame_id"], "frame_id")
    _expect_single_launch_config(params["user_config_path"], "livox_config_file")

    assert isinstance(params["cmdline_input_bd_code"], tuple)
    assert len(params["cmdline_input_bd_code"]) == 1
    assert isinstance(params["cmdline_input_bd_code"][0], TextSubstitution)
    text_value = params["cmdline_input_bd_code"][0].text
    assert text_value.splitlines()[0] == "livox0000000001"


def test_fast_livo_node_loads_both_yaml_parameter_files() -> None:
    launch_description = _load_launch_description()
    nodes = [
        entity for entity in launch_description.entities
        if isinstance(entity, Node)
    ]
    fast_livo_node = next(
        node for node in nodes
        if (node.node_package, node.node_executable) ==
        ("fast_livo", "fastlivo_mapping")
    )

    parameter_entries = [
        entry for entry in fast_livo_node._Node__parameters
        if isinstance(entry, ParameterFile)
    ]
    assert len(parameter_entries) == 2

    parameter_launch_configs = []
    for entry in parameter_entries:
        assert len(entry.param_file) == 1
        launch_config = entry.param_file[0]
        assert isinstance(launch_config, LaunchConfiguration)
        parameter_launch_configs.append(_launch_config_name(launch_config))

    assert set(parameter_launch_configs) == {
        "livo_params_file",
        "camera_params_file",
    }


def test_static_base_link_transform_node_matches_contract() -> None:
    launch_description = _load_launch_description()
    nodes = [
        entity for entity in launch_description.entities
        if isinstance(entity, Node)
    ]
    static_tf_node = next(
        node for node in nodes
        if (node.node_package, node.node_executable) ==
        ("tf2_ros", "static_transform_publisher") and
        list(node._Node__arguments[-4:]) == [
            "--frame-id", "aft_mapped",
            "--child-frame-id", "base_link",
        ]
    )

    args = static_tf_node._Node__arguments
    assert args is not None
    assert len(args) == 16
    assert list(args[:8]) == [
        "--x", "0",
        "--y", "0",
        "--z", "0",
        "--roll", "0",
    ]
    assert args[8] == "--pitch"
    assert isinstance(args[9], LaunchConfiguration)
    assert _launch_config_name(args[9]) == "base_link_pitch_rad"
    assert list(args[10:]) == [
        "--yaw", "0",
        "--frame-id", "aft_mapped",
        "--child-frame-id", "base_link",
    ]


def test_static_world_to_camera_init_transform_node_matches_contract() -> None:
    launch_description = _load_launch_description()
    nodes = [
        entity for entity in launch_description.entities
        if isinstance(entity, Node)
    ]
    static_tf_nodes = [
        node for node in nodes
        if (node.node_package, node.node_executable) ==
        ("tf2_ros", "static_transform_publisher")
    ]

    world_to_camera_node = next(
        node for node in static_tf_nodes
        if list(node._Node__arguments[-4:]) == [
            "--frame-id", "world",
            "--child-frame-id", "camera_init",
        ]
    )

    args = world_to_camera_node._Node__arguments
    assert args is not None
    assert list(args) == [
        "--x", "0",
        "--y", "0",
        "--z", "0",
        "--roll", "0",
        "--pitch", "0",
        "--yaw", "0",
        "--frame-id", "world",
        "--child-frame-id", "camera_init",
    ]
