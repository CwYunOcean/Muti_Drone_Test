from importlib.util import module_from_spec
from importlib.util import spec_from_file_location
from pathlib import Path

from ament_index_python.packages import PackageNotFoundError
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node


PACKAGE_NAME = "fast_livo2_mid360_bringup"
EXPERIMENT_LAUNCH_PATH = "mid360_livo_d435i_sync_experiment.launch.py"
EXPECTED_ARGUMENTS = {
    "livox_config_file",
    "livo_params_file",
    "camera_params_file",
    "realsense_serial_no",
    "base_link_pitch_rad",
    "img_time_offset",
}
EXPECTED_NODE_WIRING = {
    ("livox_ros_driver2", "livox_ros_driver2_node"),
    ("livox_ros_driver2", "sync_node_ros2"),
    ("realsense2_camera", "realsense2_camera_node"),
    ("fast_livo", "fastlivo_mapping"),
    ("tf2_ros", "static_transform_publisher"),
}


def _installed_launch_path() -> Path:
    return Path(get_package_share_directory(PACKAGE_NAME)) / "launch" / EXPERIMENT_LAUNCH_PATH


def _load_launch_description():
    launch_path = _installed_launch_path()
    assert launch_path.exists(), f"Missing installed launch file: {launch_path}"

    spec = spec_from_file_location("mid360_livo_d435i_sync_experiment_launch", launch_path)
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


def test_sync_experiment_launch_file_exists() -> None:
    assert _installed_launch_path().exists(), f"Missing installed launch file: {_installed_launch_path()}"


def test_sync_experiment_launch_declares_expected_arguments() -> None:
    launch_description = _load_launch_description()
    declared_args = {
        entity.name for entity in launch_description.entities
        if isinstance(entity, DeclareLaunchArgument)
    }
    assert EXPECTED_ARGUMENTS.issubset(declared_args)


def test_sync_experiment_launch_wiring_matches_contract() -> None:
    launch_description = _load_launch_description()
    node_actions = [
        entity for entity in launch_description.entities
        if isinstance(entity, Node)
    ]
    actual_wiring = {
        (node.node_package, node.node_executable)
        for node in node_actions
    }
    assert EXPECTED_NODE_WIRING.issubset(actual_wiring)
