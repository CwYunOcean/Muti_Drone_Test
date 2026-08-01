from pathlib import Path


def _sync_script_path() -> Path:
    return (
        Path(__file__).resolve().parents[4]
        / "scripts"
        / "run_mid360_livo_d435i_sync.sh"
    )


def test_sync_script_opens_rviz_before_waiting_on_launch() -> None:
    script_path = _sync_script_path()
    assert script_path.exists(), f"Missing launch script: {script_path}"

    script_lines = script_path.read_text(encoding="utf-8").splitlines()
    launch_pid_index = next(
        index
        for index, line in enumerate(script_lines)
        if line.strip() == "LAUNCH_PID=$!"
    )
    rviz_index = next(
        index
        for index, line in enumerate(script_lines)
        if line.strip() == 'rviz2 -d "$RVIZ_CONFIG_FILE"'
    )
    post_launch_wait_indexes = [
        index
        for index, line in enumerate(script_lines)
        if line.strip() == 'wait "$LAUNCH_PID"' and index > launch_pid_index
    ]

    assert launch_pid_index < rviz_index, (
        "RViz must launch after the stack starts in the background."
    )
    if post_launch_wait_indexes:
        assert rviz_index < post_launch_wait_indexes[0], (
            "RViz must open before the main flow blocks on the launch process; "
            "otherwise the UI never appears while the stack is running."
        )


def test_sync_script_uses_lower_load_realsense_profile() -> None:
    script_path = _sync_script_path()
    assert script_path.exists(), f"Missing launch script: {script_path}"

    script = script_path.read_text(encoding="utf-8")

    assert 'infra_profile:="640x480x10"' in script, (
        "The sync launch helper should force a lower-load RealSense profile "
        "to better match the reference bag rate and reduce capture pressure."
    )
