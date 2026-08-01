from pathlib import Path


def _script_path() -> Path:
    return Path(__file__).resolve().parents[4] / "scripts" / "replay_gvf_ismc_bag.sh"


def _rviz_config_path() -> Path:
    return Path(__file__).resolve().parents[4] / "scripts" / "gvf_ismc_bag_replay.rviz"


def test_replay_script_exists() -> None:
    script_path = _script_path()
    assert script_path.exists(), f"Missing replay script: {script_path}"


def test_replay_rviz_config_exists() -> None:
    rviz_path = _rviz_config_path()
    assert rviz_path.exists(), f"Missing replay RViz config: {rviz_path}"


def test_replay_script_uses_replay_launch_and_rosbag_clock() -> None:
    script = _script_path().read_text(encoding="utf-8")

    assert "export RMW_IMPLEMENTATION=rmw_fastrtps_cpp" in script
    assert 'ROS_DISTRO="${ROS_DISTRO:-jazzy}"' in script
    assert 'source "/opt/ros/$ROS_DISTRO/setup.bash"' in script
    assert 'source "$OVERLAY_WS/install/setup.bash"' in script
    assert 'DRONE_ID="${DRONE_ID:-1}"' in script
    assert "ros2 launch gvf_ismc_real_bringup replay_visualization.launch.py" in script
    assert 'drone_id:="$DRONE_ID"' in script
    assert "ros2 bag play" in script
    assert "--clock" in script
    assert 'BAG_RATE="${BAG_RATE:-1.0}"' in script
    assert 'BAG_LOOP="${BAG_LOOP:-false}"' in script


def test_replay_script_validates_required_odom_topics() -> None:
    script = _script_path().read_text(encoding="utf-8")

    assert "ros2 bag info" in script
    assert '${TOPIC_PREFIX}/aft_mapped_to_init' in script
    assert '${TOPIC_PREFIX}/aft_mapped_to_init_level' in script
    assert "This bag does not contain recorded FAST-LIO odometry." in script


def test_replay_script_passes_rviz_config_to_launch() -> None:
    script = _script_path().read_text(encoding="utf-8")

    assert "rviz_config_file:=" in script
    assert "gvf_ismc_bag_replay.rviz" in script


def test_replay_rviz_config_references_expected_topics_and_fixed_frame() -> None:
    rviz_text = _rviz_config_path().read_text(encoding="utf-8")

    assert "Fixed Frame: world" in rviz_text
    assert "/aft_mapped_to_init" in rviz_text
    assert "/gvf/actual_path" in rviz_text
    assert "/gvf/reference_path" in rviz_text
    assert "/gvf/reference_marker" in rviz_text
