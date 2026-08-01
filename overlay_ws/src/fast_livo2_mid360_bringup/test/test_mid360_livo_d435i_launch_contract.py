from importlib.util import module_from_spec
from importlib.util import spec_from_file_location
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from ament_index_python.packages import PackageNotFoundError
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node


PACKAGE_NAME = "fast_livo2_mid360_bringup"
RUNTIME_LAUNCH_PATH = "mid360_livo_d435i.launch.py"
CALIB_LAUNCH_PATH = "mid360_d435i_calib_capture.launch.py"
EXPECTED_RUNTIME_ARGUMENTS = {
    "livox_config_file",
    "livo_params_file",
    "camera_params_file",
    "realsense_serial_no",
    "base_link_pitch_rad",
    "img_time_offset",
}
EXPECTED_RUNTIME_NODE_WIRING = {
    ("livox_ros_driver2", "livox_ros_driver2_node"),
    ("realsense2_camera", "realsense2_camera_node"),
    ("fast_livo", "fastlivo_mapping"),
    ("tf2_ros", "static_transform_publisher"),
}
EXPECTED_CALIB_NODE_WIRING = {
    ("livox_ros_driver2", "livox_ros_driver2_node"),
    ("realsense2_camera", "realsense2_camera_node"),
}


def _installed_launch_path(filename: str) -> Path:
    return Path(get_package_share_directory(PACKAGE_NAME)) / "launch" / filename


def _load_launch_description(filename: str, module_name: str):
    launch_path = _installed_launch_path(filename)
    assert launch_path.exists(), f"Missing installed launch file: {launch_path}"

    spec = spec_from_file_location(module_name, launch_path)
    assert spec is not None and spec.loader is not None

    module = module_from_spec(spec)
    spec.loader.exec_module(module)

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


def test_mid360_livo_d435i_runtime_launch_file_exists() -> None:
    launch_path = _installed_launch_path(RUNTIME_LAUNCH_PATH)
    assert launch_path.exists(), f"Missing installed launch file: {launch_path}"


def test_mid360_d435i_calib_capture_launch_file_exists() -> None:
    launch_path = _installed_launch_path(CALIB_LAUNCH_PATH)
    assert launch_path.exists(), f"Missing installed launch file: {launch_path}"


def test_runtime_launch_declares_expected_arguments() -> None:
    launch_description = _load_launch_description(
        RUNTIME_LAUNCH_PATH, "mid360_livo_d435i_launch"
    )
    arg_actions = [
        entity
        for entity in launch_description.entities
        if isinstance(entity, DeclareLaunchArgument)
    ]
    declared_args = {action.name for action in arg_actions}
    assert EXPECTED_RUNTIME_ARGUMENTS.issubset(declared_args)


def test_runtime_launch_starts_expected_nodes() -> None:
    launch_description = _load_launch_description(
        RUNTIME_LAUNCH_PATH, "mid360_livo_d435i_launch_nodes"
    )
    node_actions = [
        entity for entity in launch_description.entities if isinstance(entity, Node)
    ]
    actual_wiring = {(node.node_package, node.node_executable) for node in node_actions}
    assert EXPECTED_RUNTIME_NODE_WIRING.issubset(actual_wiring)


def test_calib_capture_launch_starts_expected_nodes() -> None:
    launch_description = _load_launch_description(
        CALIB_LAUNCH_PATH, "mid360_d435i_calib_capture_launch"
    )
    node_actions = [
        entity for entity in launch_description.entities if isinstance(entity, Node)
    ]
    actual_wiring = {(node.node_package, node.node_executable) for node in node_actions}
    assert EXPECTED_CALIB_NODE_WIRING.issubset(actual_wiring)
