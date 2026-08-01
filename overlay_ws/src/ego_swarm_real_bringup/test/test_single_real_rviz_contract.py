from pathlib import Path


RVIZ_CONFIG_PATH = Path(__file__).resolve().parents[1] / "rviz" / "ego_single_real.rviz"


def test_single_real_rviz_config_exists() -> None:
    assert RVIZ_CONFIG_PATH.exists(), f"Missing RViz config: {RVIZ_CONFIG_PATH}"


def test_single_real_rviz_config_tracks_real_topics() -> None:
    rviz_config = RVIZ_CONFIG_PATH.read_text(encoding="utf-8")

    assert "Fixed Frame: camera_init_level" in rviz_config
    assert "Value: /drone_1/cloud_registered_level" in rviz_config
    assert "Value: /drone_1/aft_mapped_to_init_level" in rviz_config
    assert "Value: /drone_1_grid/grid_map/occupancy_inflate" in rviz_config
    assert "Value: /drone_1_plan_vis/goal_point" in rviz_config
    assert "Value: /drone_1_plan_vis/global_list" in rviz_config
    assert "Value: /drone_1_plan_vis/init_list" in rviz_config
    assert "Value: /drone_1_plan_vis/optimal_list" in rviz_config
    assert "Value: /drone_1_plan_vis/a_star_list" in rviz_config
    assert "Value: /drone_1/move_base_simple/goal" in rviz_config


def test_single_real_rviz_inflated_map_uses_height_coloring() -> None:
    rviz_config = RVIZ_CONFIG_PATH.read_text(encoding="utf-8")

    assert "Name: Inflated Map" in rviz_config
    assert "Color Transformer: AxisColor" in rviz_config
    assert "Use rainbow: true" in rviz_config
