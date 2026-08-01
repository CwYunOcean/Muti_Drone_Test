from pathlib import Path


def _script_path() -> Path:
    return Path(__file__).resolve().parents[4] / "scripts" / "run_ego_single_real.sh"


def _robot_config_path() -> Path:
    return Path(__file__).resolve().parents[4] / "scripts" / "load_robot_config.sh"


def test_single_real_script_exists() -> None:
    script_path = _script_path()
    assert script_path.exists(), f"Missing launch script: {script_path}"


def test_single_real_script_launches_stack_in_background() -> None:
    script_path = _script_path()
    script_lines = script_path.read_text(encoding="utf-8").splitlines()

    assert any(
        line.strip() == "LAUNCH_PID=$!" for line in script_lines
    ), "Real-hardware stack should be launched in the background."


def test_single_real_script_uses_fastdds_and_real_launch() -> None:
    script = _script_path().read_text(encoding="utf-8")
    robot_config = _robot_config_path().read_text(encoding="utf-8")

    assert 'source "$BASE_REPO_ROOT/scripts/load_robot_config.sh"' in script
    assert "export RMW_IMPLEMENTATION" in script
    assert ': "${RMW_IMPLEMENTATION:=rmw_fastrtps_cpp}"' in robot_config
    assert "ros2 launch ego_swarm_real_bringup single_real.launch.py" in script


def test_single_real_script_defaults_to_jazzy_ros_setup() -> None:
    script = _script_path().read_text(encoding="utf-8")
    robot_config = _robot_config_path().read_text(encoding="utf-8")

    assert ': "${ROS_DISTRO:=jazzy}"' in robot_config
    assert 'source "/opt/ros/$ROS_DISTRO/setup.bash"' in script


def test_single_real_script_supports_planner_input_mode() -> None:
    script = _script_path().read_text(encoding="utf-8")

    assert 'PLANNER_INPUT_MODE="${PLANNER_INPUT_MODE:-fastlio2_leveled}"' in script
    assert 'planner_input_mode:="$PLANNER_INPUT_MODE"' in script


def test_single_real_script_defaults_to_drone1_and_supports_override() -> None:
    script = _script_path().read_text(encoding="utf-8")
    robot_config = _robot_config_path().read_text(encoding="utf-8")

    assert ': "${DRONE_ID:=1}"' in robot_config
    assert 'drone_id:="$DRONE_ID"' in script
