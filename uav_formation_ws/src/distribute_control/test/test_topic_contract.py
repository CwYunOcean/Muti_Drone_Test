from pathlib import Path
import sys


LAUNCH_DIR = Path(__file__).resolve().parents[1] / 'launch'
sys.path.insert(0, str(LAUNCH_DIR))

from formation_launch_common import drone_prefix, formation_topics  # noqa: E402


def test_drone_prefix_and_fmu_topics_are_drone_scoped() -> None:
    assert drone_prefix(1) == '/drone_1'
    topics = formation_topics(2)
    assert topics['fmu_in_prefix'] == '/drone_2/fmu/in'
    assert topics['fmu_out_prefix'] == '/drone_2/fmu/out'
    assert topics['virtual_w'] == '/drone_2/virtual_w'


def test_topic_contract_does_not_use_legacy_px4_prefix() -> None:
    source_root = Path(__file__).resolve().parents[1]
    source_files = list((source_root / 'launch').glob('*.py'))
    source_files += list((source_root / 'src').rglob('*.cpp'))
    source_files += list((source_root / 'src').rglob('*.hpp'))
    source = '\n'.join(path.read_text() for path in source_files)
    assert '/px4_' not in source
