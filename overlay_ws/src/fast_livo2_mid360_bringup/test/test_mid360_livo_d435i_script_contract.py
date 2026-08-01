from pathlib import Path


def _script_path() -> Path:
    return (
        Path(__file__).resolve().parents[4]
        / "scripts"
        / "run_mid360_livo_d435i.sh"
    )


def test_non_sync_script_uses_explicit_realsense_profile() -> None:
    script_path = _script_path()
    assert script_path.exists(), f"Missing launch script: {script_path}"

    script = script_path.read_text(encoding="utf-8")

    assert 'infra_profile:="640x480x10"' in script, (
        "The non-sync launch helper should explicitly pass the same lower-load "
        "RealSense profile used by the sync helper."
    )
