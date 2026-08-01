from pathlib import Path


def _script_path() -> Path:
    return Path(__file__).resolve().parents[4] / "scripts" / "run_gvf_ismc_single_real.sh"


def _bag_script_path() -> Path:
    return Path(__file__).resolve().parents[4] / "scripts" / "record_gvf_ismc_experiment.sh"


def _fastlio_script_path() -> Path:
    return Path(__file__).resolve().parents[4] / "scripts" / "run_mid360_fastlio2.sh"


def _robot_config_path() -> Path:
    return Path(__file__).resolve().parents[4] / "scripts" / "load_robot_config.sh"


def _rviz_config_path() -> Path:
    return Path(__file__).resolve().parents[4] / "scripts" / "gvf_ismc_reference.rviz"


def test_single_real_script_exists() -> None:
    script_path = _script_path()
    assert script_path.exists(), f"Missing launch script: {script_path}"


def test_record_script_exists() -> None:
    script_path = _bag_script_path()
    assert script_path.exists(), f"Missing rosbag script: {script_path}"


def test_fastlio_script_exists() -> None:
    script_path = _fastlio_script_path()
    assert script_path.exists(), f"Missing FAST-LIO2 script: {script_path}"


def test_rviz_config_exists() -> None:
    rviz_path = _rviz_config_path()
    assert rviz_path.exists(), f"Missing RViz config: {rviz_path}"


def test_single_real_script_uses_fastdds_and_real_launch() -> None:
    script = _script_path().read_text(encoding="utf-8")
    robot_config = _robot_config_path().read_text(encoding="utf-8")

    assert 'source "$BASE_REPO_ROOT/scripts/load_robot_config.sh"' in script
    assert "export RMW_IMPLEMENTATION" in script
    assert ': "${RMW_IMPLEMENTATION:=rmw_fastrtps_cpp}"' in robot_config
    assert ': "${ROS_DISTRO:=jazzy}"' in robot_config
    assert 'source "/opt/ros/$ROS_DISTRO/setup.bash"' in script
    assert 'source "$OVERLAY_WS/install/setup.bash"' in script
    assert "ros2 launch gvf_ismc_real_bringup single_real.launch.py" in script
    assert ': "${DRONE_ID:=1}"' in robot_config
    assert 'drone_id:="$DRONE_ID"' in script


def test_single_real_script_supports_acceleration_feedforward_toggle() -> None:
    script = _script_path().read_text(encoding="utf-8")

    assert 'USE_ACCELERATION_FF="${USE_ACCELERATION_FF:-true}"' in script
    assert 'use_acceleration_feedforward:="$USE_ACCELERATION_FF"' in script
    assert 'echo "use_acceleration_feedforward=$USE_ACCELERATION_FF"' in script


def test_record_script_records_core_experiment_topics() -> None:
    script = _bag_script_path().read_text(encoding="utf-8")

    assert "export RMW_IMPLEMENTATION=rmw_fastrtps_cpp" in script
    assert 'ROS_DISTRO="${ROS_DISTRO:-jazzy}"' in script
    assert 'source "/opt/ros/$ROS_DISTRO/setup.bash"' in script
    assert 'source "$OVERLAY_WS/install/setup.bash"' in script
    assert 'DRONE_ID="${DRONE_ID:-1}"' in script
    assert 'TOPIC_PREFIX="/drone_${DRONE_ID}"' in script
    assert 'ros2 topic echo "${TOPIC_PREFIX}/aft_mapped_to_init" --once' in script
    assert 'ros2 topic echo "${TOPIC_PREFIX}/aft_mapped_to_init_level" --once' in script
    assert 'ros2 topic echo /gvf/reference --once' in script
    assert 'ros2 topic echo /gvf/reference_path --once' in script
    assert 'ros2 topic echo /gvf/reference_marker --once' in script
    assert 'ros2 topic echo "${PLANNING_PREFIX}/pos_cmd" --once' in script
    assert "ros2 bag record" in script
    assert '${TOPIC_PREFIX}/aft_mapped_to_init' in script
    assert '${TOPIC_PREFIX}/aft_mapped_to_init_level' in script
    assert "/gvf/reference" in script
    assert "/gvf/reference_path" in script
    assert "/gvf/reference_marker" in script
    assert '${PLANNING_PREFIX}/pos_cmd' in script
    assert "/tf" in script
    assert "/tf_static" in script
    assert '${FMU_IN_PREFIX}/offboard_control_mode' in script
    assert '${FMU_IN_PREFIX}/trajectory_setpoint' in script
    assert '${FMU_OUT_PREFIX}/vehicle_status_v1' in script


def test_fastlio_script_uses_same_rmw_as_recording_and_control() -> None:
    script = _fastlio_script_path().read_text(encoding="utf-8")
    robot_config = _robot_config_path().read_text(encoding="utf-8")

    assert 'source "$BASE_REPO_ROOT/scripts/load_robot_config.sh"' in script
    assert "export RMW_IMPLEMENTATION" in script
    assert ': "${RMW_IMPLEMENTATION:=rmw_fastrtps_cpp}"' in robot_config
    assert "ros2 launch fast_lio2_mid360_bringup mid360_fastlio2.launch.py" in script


def test_rviz_config_references_gvf_visualization_topics() -> None:
    rviz_text = _rviz_config_path().read_text(encoding="utf-8")

    assert "/gvf/reference_path" in rviz_text
    assert "/gvf/reference_marker" in rviz_text
    assert "/gvf/actual_path" in rviz_text
