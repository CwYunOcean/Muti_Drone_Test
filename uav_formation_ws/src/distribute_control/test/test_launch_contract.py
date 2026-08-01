from pathlib import Path


LAUNCH_DIR = Path(__file__).resolve().parents[1] / 'launch'


def test_sitl_launch_assigns_px4_dds_namespace_per_drone() -> None:
    source = (LAUNCH_DIR / 'formation_sitl_launch.py').read_text()
    assert "'PX4_UXRCE_DDS_NS': f'drone_{drone_id}'" in source
    assert "cmd=[px4_bin, '-i', str(drone_id)]" in source


def test_real_launch_requires_explicit_px4_system_id() -> None:
    source = (LAUNCH_DIR / 'single_real.launch.py').read_text()
    assert "DeclareLaunchArgument('target_system'" in source
    assert "'system.target_system': target_system" in source
