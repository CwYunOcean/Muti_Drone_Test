from pathlib import Path


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
CMAKE_PATH = PACKAGE_ROOT / "CMakeLists.txt"
GVF_NODE_PATH = PACKAGE_ROOT / "src" / "gvf_reference_node.cpp"
ISMC_NODE_PATH = PACKAGE_ROOT / "src" / "ismc_velocity_tracker_node.cpp"


def test_gvf_reference_node_declares_reference_topic_and_publishes_gvf_reference() -> None:
    source = GVF_NODE_PATH.read_text(encoding="utf-8")

    assert (
        'declare_parameter<std::string>("odom_topic", "/drone_1/aft_mapped_to_init_level")'
        in source
    )
    assert 'declare_parameter<std::string>("reference_topic", "/gvf/reference")' in source
    assert 'declare_parameter<std::string>("actual_path_topic", "/gvf/actual_path")' in source
    assert "create_publisher<gvf_path_following_msgs::msg::GVFReference>" in source
    assert 'declare_parameter<std::string>("reference_path_topic"' in source
    assert 'declare_parameter<std::string>("reference_marker_topic"' in source
    assert "create_publisher<nav_msgs::msg::Path>" in source
    assert "create_publisher<visualization_msgs::msg::Marker>" in source
    assert 'declare_parameter<std::string>("trajectory_type", "circular")' in source
    assert "evaluate_circular_gvf" in source
    assert "evaluate_three_leaf_gvf" in source


def test_ismc_tracker_node_declares_command_topic_and_publishes_position_command() -> None:
    source = ISMC_NODE_PATH.read_text(encoding="utf-8")

    assert (
        'declare_parameter<std::string>("odom_topic", "/drone_1/aft_mapped_to_init_level")'
        in source
    )
    assert (
        'declare_parameter<std::string>("command_topic", "/drone_1_planning/pos_cmd")'
        in source
    )
    assert "create_publisher<quadrotor_msgs::msg::PositionCommand>" in source
    assert "std::numeric_limits<double>::quiet_NaN()" in source
    assert 'declare_parameter<double>("lambda", 2.0)' in source
    assert 'declare_parameter<double>("k", 1.0)' in source
    assert 'declare_parameter<double>("c1", 0.05)' in source
    assert 'declare_parameter<double>("epsilon", 0.01)' in source
    assert 'declare_parameter<double>("adaptation_gain", 0.0)' in source
    assert 'declare_parameter<double>("max_acceleration_mps2", 3.0)' in source
    assert 'declare_parameter<double>("input_timeout_sec", 0.5)' in source
    assert 'declare_parameter<bool>("use_px4_position_hold_for_z", false)' in source
    assert 'declare_parameter<double>("target_height_m", 1.2)' in source
    assert "if (!odom_is_fresh(now_time) || !reference_is_fresh(now_time))" in source


def test_nodes_install_to_package_libexec_directory() -> None:
    cmake = CMAKE_PATH.read_text(encoding="utf-8")

    assert "RUNTIME DESTINATION lib/${PROJECT_NAME}" in cmake
