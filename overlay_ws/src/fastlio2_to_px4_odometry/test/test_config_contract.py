from pathlib import Path

import yaml


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
CONFIG_PATH = PACKAGE_ROOT / "config" / "fastlio2_to_px4_odometry.yaml"
NODE_PATH = PACKAGE_ROOT / "src" / "fastlio2_to_px4_odometry_node.cpp"


def _load_yaml(path: Path) -> dict:
    return yaml.safe_load(path.read_text(encoding="utf-8"))


def test_config_file_exists() -> None:
    assert CONFIG_PATH.exists(), f"Missing config file: {CONFIG_PATH}"


def test_config_defaults_match_fastlio2_bridge_contract() -> None:
    config = _load_yaml(CONFIG_PATH)
    params = config["/fastlio2_to_px4_odometry"]["ros__parameters"]

    assert params["input_topic"] == "/drone_1/aft_mapped_to_init_level"
    assert params["output_topic"] == "/drone_1/fmu/in/vehicle_visual_odometry"
    assert params["world_axis"] == [1, 0, 2]
    assert params["world_sign"] == [1.0, 1.0, -1.0]
    assert params["body_axis"] == [0, 1, 2]
    assert params["body_sign"] == [1.0, -1.0, -1.0]
    assert params["body_extrinsic_translation_m"] == [0.0, 0.0, 0.0]
    # Mount tilt is fully handled by fastlio2_to_ego_swarm_leveling; this
    # bridge must stay a pure ENU->NED axis mapper.
    assert params["body_extrinsic_rpy_rad"] == [0.0, 0.0, 0.0]
    assert params["quality"] == 100


def test_node_defaults_match_drone1_topics_for_direct_runs() -> None:
    source = NODE_PATH.read_text(encoding="utf-8")

    assert '"input_topic", "/drone_1/aft_mapped_to_init_level"' in source
    assert '"output_topic", "/drone_1/fmu/in/vehicle_visual_odometry"' in source
