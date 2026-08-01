from pathlib import Path

from ament_index_python.packages import get_package_share_directory
import yaml


PACKAGE_NAME = "fast_livo2_mid360_bringup"
PACKAGE_SHARE = Path(get_package_share_directory(PACKAGE_NAME))
RUNTIME_CONFIG_PATH = PACKAGE_SHARE / "config" / "fast_livo2_mid360_livo_d435i.yaml"


def load_yaml(path: Path) -> dict:
    return yaml.safe_load(path.read_text())


def test_mid360_livo_d435i_uav_flags_are_booleans() -> None:
    config = load_yaml(RUNTIME_CONFIG_PATH)
    params = config["/**"]["ros__parameters"]
    uav = params["uav"]

    assert isinstance(uav["imu_rate_odom"], bool)
    assert isinstance(uav["gravity_align_en"], bool)
