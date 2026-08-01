from pathlib import Path
import subprocess


REPO_ROOT = Path(__file__).resolve().parents[1]
FAST_LIO_SOURCE = "slam_ws/src/FAST_LIO/package.xml"
LOCAL_OUTPUTS = (
    "slam_ws/src/FAST-Calib-ROS2/calib_data/mid360_11/11.png",
    "slam_ws/src/FAST-LIVO2/Log/imu.txt",
    "slam_ws/src/FAST_LIO/Log/imu.txt",
    "slam_ws/src/FAST_LIO/PCD/1",
    "slam_ws/src/rpg_vikit/ername",
)


def run_git(*args: str) -> subprocess.CompletedProcess[str]:
    return subprocess.run(
        ["git", *args],
        cwd=REPO_ROOT,
        capture_output=True,
        check=False,
        text=True,
    )


def test_slam_source_is_tracked_and_local_outputs_are_ignored() -> None:
    ignored_source = run_git("check-ignore", "-q", FAST_LIO_SOURCE)
    tracked_source = run_git("ls-files", "--error-unmatch", FAST_LIO_SOURCE)

    assert ignored_source.returncode == 1, ignored_source.stderr
    assert tracked_source.returncode == 0, tracked_source.stderr

    for local_output in LOCAL_OUTPUTS:
        ignored_output = run_git("check-ignore", "-q", local_output)
        assert ignored_output.returncode == 0, local_output
