from pathlib import Path


LASER_MAPPING_PATH = Path(__file__).resolve().parents[1] / "src" / "laserMapping.cpp"


def test_fastlio_odometry_publishes_state_velocity_in_twist() -> None:
    source = LASER_MAPPING_PATH.read_text(encoding="utf-8")

    assert "void publish_odometry(" in source
    assert "state_point.vel(0)" in source
    assert "state_point.vel(1)" in source
    assert "state_point.vel(2)" in source
    assert "odomAftMapped.twist.twist.linear.x" in source
    assert "odomAftMapped.twist.twist.linear.y" in source
    assert "odomAftMapped.twist.twist.linear.z" in source
