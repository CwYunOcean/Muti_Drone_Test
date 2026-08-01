from pathlib import Path

import json
import yaml


REPO_ROOT = Path(__file__).resolve().parents[4]
PACKAGE_ROOT = REPO_ROOT / "overlay_ws" / "src" / "fast_lio2_mid360_bringup"
CONFIG_PATH = PACKAGE_ROOT / "config" / "fast_lio2_mid360.yaml"
DRONE1_DRIVER_CONFIG_PATH = PACKAGE_ROOT / "config" / "MID360_config_drone_1.json"
DRONE1_SDK_CONFIG_PATH = PACKAGE_ROOT / "config" / "livox_sdk2_mid360_drone_1.json"


def _load_yaml(path: Path) -> dict:
    return yaml.safe_load(path.read_text(encoding="utf-8"))


def _load_json(path: Path) -> dict:
    return json.loads(path.read_text(encoding="utf-8"))


def test_config_file_exists() -> None:
    assert CONFIG_PATH.exists(), f"Missing config file: {CONFIG_PATH}"


def test_config_contains_expected_mid360_source_contract() -> None:
    config = _load_yaml(CONFIG_PATH)
    params = config["/**"]["ros__parameters"]

    assert params["common"]["lid_topic"] == "/livox/lidar"
    assert params["common"]["imu_topic"] == "/livox/imu"
    assert params["preprocess"]["lidar_type"] == 1
    assert params["preprocess"]["scan_rate"] == 10
    assert params["mapping"]["extrinsic_est_en"] is False
    assert params["mapping"]["extrinsic_T"] == [-0.011, -0.02329, 0.04412]
    assert params["mapping"]["extrinsic_R"] == [
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


def test_drone1_livox_ros_driver_config_matches_detected_mid360_link() -> None:
    assert DRONE1_DRIVER_CONFIG_PATH.exists()
    config = _load_json(DRONE1_DRIVER_CONFIG_PATH)

    host_info = config["MID360"]["host_net_info"]
    assert host_info["cmd_data_ip"] == "192.168.1.50"
    assert host_info["push_msg_ip"] == "192.168.1.50"
    assert host_info["point_data_ip"] == "192.168.1.50"
    assert host_info["imu_data_ip"] == "192.168.1.50"
    assert config["lidar_configs"][0]["ip"] == "192.168.1.108"


def test_drone1_livox_sdk2_quick_start_config_matches_detected_mid360_link() -> None:
    assert DRONE1_SDK_CONFIG_PATH.exists()
    config = _load_json(DRONE1_SDK_CONFIG_PATH)

    host_info = config["MID360"]["host_net_info"][0]
    assert host_info["host_ip"] == "192.168.1.50"
    assert host_info["cmd_data_port"] == 56101
    assert host_info["push_msg_port"] == 56201
    assert host_info["point_data_port"] == 56301
    assert host_info["imu_data_port"] == 56401
