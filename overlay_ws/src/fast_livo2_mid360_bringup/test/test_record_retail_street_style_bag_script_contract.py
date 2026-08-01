from pathlib import Path


def _script_path() -> Path:
    return (
        Path(__file__).resolve().parents[4]
        / "scripts"
        / "record_retail_street_style_bag.sh"
    )


def test_record_script_maps_topics_to_retail_street_style_names() -> None:
    script_path = _script_path()
    assert script_path.exists(), f"Missing recording helper script: {script_path}"

    script = script_path.read_text(encoding="utf-8")

    assert "image_transport" in script
    assert "--remap in:=/camera/camera/infra1/image_rect_raw" in script
    assert "--remap out:=/left_camera/image" in script
    assert "/livox/lidar" in script
    assert "/livox/imu" in script
    assert "/left_camera/image" in script

    record_section = script.split('ros2 bag record -o "$BAG_PATH"', 1)[1]
    assert "/camera/camera/infra1/image_rect_raw" not in record_section, (
        "The recording helper should not duplicate the raw infra stream once "
        "it has been republished as /left_camera/image."
    )
