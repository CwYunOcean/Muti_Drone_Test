from pathlib import Path


def test_leveling_node_broadcasts_level_frames() -> None:
    source_path = (
        Path(__file__).resolve().parents[1] /
        "src" /
        "fastlio2_to_ego_swarm_leveling_node.cpp"
    )
    source = source_path.read_text(encoding="utf-8")

    assert "tf2_ros/transform_broadcaster.h" in source
    assert "tf2_ros/static_transform_broadcaster.h" in source
    assert "world_frame_id" in source
    assert "sendTransform" in source


def test_leveling_node_defaults_to_drone1_topics_for_direct_runs() -> None:
    source_path = (
        Path(__file__).resolve().parents[1] /
        "src" /
        "fastlio2_to_ego_swarm_leveling_node.cpp"
    )
    source = source_path.read_text(encoding="utf-8")

    assert '"input_odom_topic", "/drone_1/aft_mapped_to_init"' in source
    assert '"input_cloud_topic", "/drone_1/cloud_registered"' in source
    assert '"output_odom_topic", "/drone_1/aft_mapped_to_init_level"' in source
    assert '"output_cloud_topic", "/drone_1/cloud_registered_level"' in source
