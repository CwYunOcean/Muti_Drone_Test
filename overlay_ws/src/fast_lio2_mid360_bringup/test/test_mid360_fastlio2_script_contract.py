from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[4]
SCRIPT_PATH = REPO_ROOT / "scripts" / "run_mid360_fastlio2.sh"
ROBOT_CONFIG_PATH = REPO_ROOT / "scripts" / "load_robot_config.sh"


def test_fastlio2_script_exists() -> None:
    assert SCRIPT_PATH.exists(), f"Missing script: {SCRIPT_PATH}"


def test_fastlio2_script_builds_and_launches_new_package() -> None:
    script = SCRIPT_PATH.read_text(encoding="utf-8")

    assert "colcon build --symlink-install --packages-select fast_lio2_mid360_bringup" in script
    assert "ros2 launch fast_lio2_mid360_bringup mid360_fastlio2.launch.py" in script
    assert "MID360_config_drone_1.json" in script


def test_fastlio2_script_defaults_to_project_owned_drone1_livox_config() -> None:
    script = SCRIPT_PATH.read_text(encoding="utf-8")

    assert (
        'LIVOX_CONFIG_FILE="${LIVOX_CONFIG_FILE:-$OVERLAY_WS/src/'
        'fast_lio2_mid360_bringup/config/MID360_config_drone_1.json}"'
    ) in script


def test_fastlio2_script_defaults_to_jazzy_ros_setup() -> None:
    script = SCRIPT_PATH.read_text(encoding="utf-8")
    robot_config = ROBOT_CONFIG_PATH.read_text(encoding="utf-8")

    assert 'source "$BASE_REPO_ROOT/scripts/load_robot_config.sh"' in script
    assert ': "${ROS_DISTRO:=jazzy}"' in robot_config
    assert 'source "/opt/ros/$ROS_DISTRO/setup.bash"' in script


def test_fastlio2_script_can_run_without_rviz_on_headless_systems() -> None:
    script = SCRIPT_PATH.read_text(encoding="utf-8")

    assert "OPEN_RVIZ" in script
    assert "DISPLAY" in script
    assert "WAYLAND_DISPLAY" in script
    assert "rviz2 -d" in script


def test_fastlio2_script_defaults_to_drone1_and_supports_override() -> None:
    script = SCRIPT_PATH.read_text(encoding="utf-8")
    robot_config = ROBOT_CONFIG_PATH.read_text(encoding="utf-8")

    assert ': "${DRONE_ID:=1}"' in robot_config
    assert 'drone_id:="$DRONE_ID"' in script
