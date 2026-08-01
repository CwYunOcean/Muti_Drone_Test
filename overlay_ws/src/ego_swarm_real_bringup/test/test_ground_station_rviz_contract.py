from pathlib import Path


RVIZ_CONFIG_PATH = (
    Path(__file__).resolve().parents[1] / "rviz" / "ego_swarm_ground_station.rviz"
)


def test_ground_station_rviz_config_exists() -> None:
    assert RVIZ_CONFIG_PATH.exists(), f"Missing RViz config: {RVIZ_CONFIG_PATH}"


def test_ground_station_rviz_shows_both_drones_in_shared_frame() -> None:
    rviz_config = RVIZ_CONFIG_PATH.read_text(encoding="utf-8")

    assert "Fixed Frame: camera_init_level" in rviz_config
    for drone in (0, 1):
        assert f"Value: /drone_{drone}/cloud_registered_level" in rviz_config
        assert f"Value: /drone_{drone}/aft_mapped_to_init_level" in rviz_config
        assert f"Value: /drone_{drone}_plan_vis/optimal_list" in rviz_config


def test_ground_station_rviz_has_one_goal_tool_per_drone() -> None:
    rviz_config = RVIZ_CONFIG_PATH.read_text(encoding="utf-8")

    assert "Value: /drone_0/move_base_simple/goal" in rviz_config
    assert "Value: /drone_1/move_base_simple/goal" in rviz_config
