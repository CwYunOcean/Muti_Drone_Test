from pathlib import Path


def _scripts_dir() -> Path:
    return Path(__file__).resolve().parents[4] / "scripts"


def _start_script_path() -> Path:
    return _scripts_dir() / "start_position_mode.sh"


def _bridge_script_path() -> Path:
    return _scripts_dir() / "run_px4_position_mode.sh"


def _robot_config_path() -> Path:
    return _scripts_dir() / "load_robot_config.sh"


def test_position_mode_launcher_exists() -> None:
    script_path = _start_script_path()
    assert script_path.exists(), f"Missing position-mode launcher: {script_path}"


def test_position_mode_bridge_script_exists() -> None:
    script_path = _bridge_script_path()
    assert script_path.exists(), f"Missing position-mode bridge script: {script_path}"


def test_position_mode_launcher_opens_three_safe_terminals() -> None:
    script = _start_script_path().read_text(encoding="utf-8")

    assert "micro_dds.sh" in script
    assert "run_mid360_fastlio2.sh" in script
    assert "run_px4_position_mode.sh" in script
    assert "run_ego_single_real.sh" not in script
    assert "run_gvf_ismc_single_real.sh" not in script
    assert "position_cmd_to_px4_bridge" not in script


def test_position_mode_launcher_supports_headless_tmux_mode() -> None:
    script = _start_script_path().read_text(encoding="utf-8")

    assert "DISPLAY" in script
    assert "WAYLAND_DISPLAY" in script
    assert "tmux" in script
    assert "start_tmux_session" in script
    assert "OPEN_RVIZ=false" in script


def test_position_mode_launcher_cleans_up_child_terminals() -> None:
    script = _start_script_path().read_text(encoding="utf-8")

    assert "TERMINAL_PIDS" in script
    assert "TMUX_SESSION" in script
    assert "cleanup()" in script
    assert "trap cleanup EXIT INT TERM HUP" in script
    assert "kill" in script


def test_position_mode_bridge_only_publishes_visual_odometry_to_px4() -> None:
    script = _bridge_script_path().read_text(encoding="utf-8")
    robot_config = _robot_config_path().read_text(encoding="utf-8")

    assert "fastlio2_to_ego_swarm_leveling_node" in script
    assert "fastlio2_to_px4_odometry_node" in script
    assert 'source "$BASE_REPO_ROOT/scripts/load_robot_config.sh"' in script
    assert ': "${DRONE_ID:=1}"' in robot_config
    assert 'TOPIC_PREFIX="/drone_${DRONE_ID}"' in script
    assert '${TOPIC_PREFIX}/aft_mapped_to_init_level' in script
    assert '${TOPIC_PREFIX}/fmu/in/vehicle_visual_odometry' in script
    assert "position_cmd_to_px4_bridge" not in script
    assert "/vehicle_command" not in script
    assert "/offboard_control_mode" not in script
    assert "/trajectory_setpoint" not in script
