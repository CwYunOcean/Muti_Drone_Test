from pathlib import Path

import yaml


def test_leveling_config_contract() -> None:
    config_path = (
        Path(__file__).resolve().parents[1] /
        "config" /
        "fastlio2_to_ego_swarm_leveling.yaml"
    )
    config = yaml.safe_load(config_path.read_text(encoding="utf-8"))
    params = config["/**"]["ros__parameters"]

    assert params["input_odom_topic"] == "/drone_1/aft_mapped_to_init"
    assert params["input_cloud_topic"] == "/drone_1/cloud_registered"
    assert params["output_odom_topic"] == "/drone_1/aft_mapped_to_init_level"
    assert params["output_cloud_topic"] == "/drone_1/cloud_registered_level"
    assert params["world_frame_id"] == "world"
    assert params["output_frame_id"] == "camera_init_level"
    assert params["output_child_frame_id"] == "body_level"
    assert params["level_rpy_rad"] == [0.0, 0.5235987755982988, 0.0]
