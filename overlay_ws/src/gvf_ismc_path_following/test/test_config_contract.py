from pathlib import Path

import yaml


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
CONFIG_PATH = PACKAGE_ROOT / "config" / "gvf_ismc_path_following.yaml"


def _load_yaml(path: Path) -> dict:
    return yaml.safe_load(path.read_text(encoding="utf-8"))


def test_config_file_exists() -> None:
    assert CONFIG_PATH.exists(), f"Missing config file: {CONFIG_PATH}"


def test_config_defaults_match_gvf_reference_contract() -> None:
    config = _load_yaml(CONFIG_PATH)
    params = config["/gvf_reference_node"]["ros__parameters"]

    assert params["odom_topic"] == "/drone_1/aft_mapped_to_init_level"
    assert params["reference_topic"] == "/gvf/reference"
    assert params["target_height_m"] == 1.2
    assert params["trajectory_type"] in ("circular", "three_leaf")
    assert params["radius_m"] == 3.5
    assert params["base_radius_m"] == 3.5
    assert params["lobe_amplitude_m"] == 0.6
    assert params["lateral_gain"] == 1.0
    assert params["vertical_gain"] == 0.4
    assert params["max_speed_mps"] == 0.55
    assert params["min_planar_speed_for_yaw_mps"] == 0.05
    assert params["yaw_alpha"] == 0.1


def test_config_defaults_match_ismc_tracker_contract() -> None:
    config = _load_yaml(CONFIG_PATH)
    params = config["/ismc_velocity_tracker_node"]["ros__parameters"]

    assert params["odom_topic"] == "/drone_1/aft_mapped_to_init_level"
    assert params["reference_topic"] == "/gvf/reference"
    assert params["command_topic"] == "/drone_1_planning/pos_cmd"
    assert params["frame_id"] == "world"
    assert params["use_px4_position_hold_for_z"] is False
    assert params["target_height_m"] == 1.2
    assert params["control_rate_hz"] == 50.0
    assert params["publish_position_nan"] is True
    assert params["lambda"] == 1.6
    assert params["k"] == 0.8
    assert params["gamma"] == 0.0
    assert params["c1"] == 0.05
    assert params["epsilon"] == 0.01
    assert params["adaptation_gain"] == 0.0
    assert params["max_acceleration_mps2"] == 2.0
    assert params["enable_adaptation"] is False
    assert params["input_timeout_sec"] == 0.5
