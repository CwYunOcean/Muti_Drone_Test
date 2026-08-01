from pathlib import Path

from ament_index_python.packages import get_package_share_directory
import yaml


PACKAGE_NAME = "fast_livo2_mid360_bringup"
PACKAGE_SHARE = Path(get_package_share_directory(PACKAGE_NAME))
CONFIG_PATH = PACKAGE_SHARE / "config" / "fast_livo2_mid360_lio.yaml"
CAMERA_PATH = PACKAGE_SHARE / "config" / "camera_startup_pinhole.yaml"


def load_yaml(path: Path) -> dict:
    return yaml.safe_load(path.read_text())


def test_lio_config_file_exists() -> None:
    assert PACKAGE_SHARE.exists(), f"Missing package share directory: {PACKAGE_SHARE}"
    assert CONFIG_PATH.exists(), f"Missing config file: {CONFIG_PATH}"


def test_camera_startup_camera_file_exists() -> None:
    assert PACKAGE_SHARE.exists(), f"Missing package share directory: {PACKAGE_SHARE}"
    assert CAMERA_PATH.exists(), f"Missing camera file: {CAMERA_PATH}"


def test_lio_config_contains_expected_mid360_contract() -> None:
    config = load_yaml(CONFIG_PATH)
    params = config["/**"]["ros__parameters"]

    assert params["common"]["img_en"] == 0
    assert params["common"]["lidar_en"] == 1
    assert params["common"]["lid_topic"] == "/livox/lidar"
    assert params["common"]["imu_topic"] == "/livox/imu"
    assert params["preprocess"]["lidar_type"] == 1
    assert params["preprocess"]["feature_extract_enabled"] is False
    assert params["preprocess"]["scan_line"] == 6
    assert params["imu"]["imu_en"] is True
    assert params["lio"]["min_iterations"] == 5
    assert params["extrin_calib"]["extrinsic_T"] == [-0.011, -0.02329, 0.04412]
    assert params["extrin_calib"]["extrinsic_R"] == [
        1.0,
        0.0,
        0.0,
        0.0,
        1.0,
        0.0,
        0.0,
        0.0,
        1.0,
    ]
    assert params["extrin_calib"]["Rcl"] == [
        1.0,
        0.0,
        0.0,
        0.0,
        1.0,
        0.0,
        0.0,
        0.0,
        1.0,
    ]
    assert len(params["extrin_calib"]["Rcl"]) == 9
    assert params["extrin_calib"]["Pcl"] == [0.0, 0.0, 0.0]
    assert len(params["extrin_calib"]["Pcl"]) == 3


def test_camera_startup_camera_contains_valid_camera_namespace() -> None:
    config = load_yaml(CAMERA_PATH)
    params = config["/**"]["ros__parameters"]["camera"]

    assert params["model"] == "Pinhole"
    assert params["width"] > 0
    assert params["height"] > 0
    assert params["scale"] > 0
    assert params["fx"] == 1293.56944
    assert params["fy"] == 1293.3155
    assert params["cx"] == 626.91359
    assert params["cy"] == 522.799224
    assert params["d0"] == -0.076160
    assert params["d1"] == 0.123001
    assert params["d2"] == -0.00113
    assert params["d3"] == 0.000251
