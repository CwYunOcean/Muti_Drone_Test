from pathlib import Path

from ament_index_python.packages import get_package_share_directory
import pytest
import yaml


PACKAGE_NAME = "fast_livo2_mid360_bringup"
PACKAGE_SHARE = Path(get_package_share_directory(PACKAGE_NAME))
RUNTIME_CONFIG_PATH = PACKAGE_SHARE / "config" / "fast_livo2_mid360_livo_d435i.yaml"
SYNC_RUNTIME_CONFIG_PATH = (
    PACKAGE_SHARE / "config" / "fast_livo2_mid360_livo_d435i_sync_experiment.yaml"
)
CAMERA_CONFIG_PATH = PACKAGE_SHARE / "config" / "d435i_infra1_pinhole.yaml"
FAST_CALIB_CONFIG_PATH = PACKAGE_SHARE / "config" / "fast_calib_mid360_d435i.yaml"


def load_yaml(path: Path) -> dict:
    return yaml.safe_load(path.read_text())


def _require_installed(path: Path) -> None:
    if not path.exists():
        pytest.fail(f"Missing installed config file: {path}")


def _require_mapping(mapping: dict, key: str, context: str) -> dict:
    assert key in mapping, f"Missing '{key}' in {context}"
    value = mapping[key]
    assert isinstance(value, dict), f"Expected mapping at {context}.{key}"
    return value


def test_mid360_livo_d435i_runtime_and_camera_config_files_exist() -> None:
    assert PACKAGE_SHARE.exists(), f"Missing package share directory: {PACKAGE_SHARE}"
    _require_installed(RUNTIME_CONFIG_PATH)
    _require_installed(SYNC_RUNTIME_CONFIG_PATH)
    _require_installed(CAMERA_CONFIG_PATH)


def test_mid360_d435i_fast_calib_config_file_exists() -> None:
    assert PACKAGE_SHARE.exists(), f"Missing package share directory: {PACKAGE_SHARE}"
    _require_installed(FAST_CALIB_CONFIG_PATH)


def test_mid360_livo_d435i_runtime_config_matches_contract() -> None:
    _require_installed(RUNTIME_CONFIG_PATH)
    config = load_yaml(RUNTIME_CONFIG_PATH)
    ros_root = _require_mapping(config, "/**", "runtime config root")
    params = _require_mapping(ros_root, "ros__parameters", "runtime config root")
    common = _require_mapping(params, "common", "runtime ros__parameters")
    extrin_calib = _require_mapping(params, "extrin_calib", "runtime ros__parameters")
    time_offset = _require_mapping(params, "time_offset", "runtime ros__parameters")

    assert common["img_en"] == 1
    assert common["img_topic"].endswith("/infra1/image_rect_raw")
    assert common["lid_topic"] == "/livox/lidar"
    assert common["imu_topic"] == "/livox/imu"
    assert common["ros_driver_bug_fix"] is False

    assert "Rcl" in extrin_calib, "Missing 'Rcl' in runtime ros__parameters.extrin_calib"
    assert "Pcl" in extrin_calib, "Missing 'Pcl' in runtime ros__parameters.extrin_calib"
    assert len(extrin_calib["Rcl"]) == 9
    assert len(extrin_calib["Pcl"]) == 3

    assert "img_time_offset" in time_offset, (
        "Missing 'img_time_offset' in runtime ros__parameters.time_offset"
    )


def test_mid360_livo_d435i_sync_runtime_config_matches_contract() -> None:
    _require_installed(SYNC_RUNTIME_CONFIG_PATH)
    config = load_yaml(SYNC_RUNTIME_CONFIG_PATH)
    ros_root = _require_mapping(config, "/**", "sync runtime config root")
    params = _require_mapping(ros_root, "ros__parameters", "sync runtime config root")
    common = _require_mapping(params, "common", "sync runtime ros__parameters")

    assert common["img_en"] == 1
    assert common["img_topic"] == "/synced_image"
    assert common["lid_topic"] == "/synced_lidar"
    assert common["imu_topic"] == "/livox/imu"
    assert common["ros_driver_bug_fix"] is False


def test_mid360_d435i_fast_calib_config_matches_contract() -> None:
    _require_installed(FAST_CALIB_CONFIG_PATH)
    config = load_yaml(FAST_CALIB_CONFIG_PATH)
    fast_calib = _require_mapping(config, "fast_calib", "fast-calib config root")
    params = _require_mapping(
        fast_calib, "ros__parameters", "fast-calib config root.fast_calib"
    )

    assert params["lidar_topic"] == "/livox/lidar"
    assert params["bag_path"].endswith("calibration_sample")
    assert params["image_path"].endswith(".png")
    assert "output_path" in params
    assert "fx" in params
    assert "fy" in params
    assert "cx" in params
    assert "cy" in params
    assert "marker_size" in params
    assert "x_min" in params
    assert "x_max" in params
