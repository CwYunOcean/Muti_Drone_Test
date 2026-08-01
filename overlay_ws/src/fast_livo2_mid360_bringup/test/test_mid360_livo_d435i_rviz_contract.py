from pathlib import Path

from ament_index_python.packages import get_package_share_directory


PACKAGE_NAME = "fast_livo2_mid360_bringup"
PACKAGE_SHARE = Path(get_package_share_directory(PACKAGE_NAME))
RVIZ_CONFIG_PATH = PACKAGE_SHARE / "rviz" / "fast_livo2_livo_d435i.rviz"


def test_livo_rviz_config_subscribes_to_published_fast_livo_topics() -> None:
    assert RVIZ_CONFIG_PATH.exists(), f"Missing RViz config: {RVIZ_CONFIG_PATH}"

    rviz_config = RVIZ_CONFIG_PATH.read_text(encoding="utf-8")

    assert "Value: /cloud_registered" in rviz_config
    assert "Value: /cloud_visual_sub_map_before" in rviz_config
    assert "Value: /rgb_img" in rviz_config
    assert "Value: /cloud_visual_map\n" not in rviz_config
    assert "Value: /cloud_visual_sub_map\n" not in rviz_config
