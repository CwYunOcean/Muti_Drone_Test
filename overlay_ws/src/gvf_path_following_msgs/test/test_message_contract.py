from pathlib import Path


def test_gvf_reference_message_contract() -> None:
    message_path = Path(__file__).resolve().parents[1] / "msg" / "GVFReference.msg"

    assert message_path.exists()

    message_definition = message_path.read_text(encoding="utf-8")

    required_fields = [
        "std_msgs/Header header",
        "geometry_msgs/Vector3 desired_velocity",
        "float64 desired_yaw",
        "float64 desired_yaw_rate",
        "float64 phi1",
        "float64 phi2",
    ]

    for field in required_fields:
        assert field in message_definition
