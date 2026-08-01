from pathlib import Path
import subprocess


REPO_ROOT = Path(__file__).resolve().parents[1]
EGO_SWARM_SOURCE = "nav_ws/src/ego-swarm-ros2/planner/ego_planner/package.xml"


def run_git(*args: str) -> subprocess.CompletedProcess[str]:
    return subprocess.run(
        ["git", *args],
        cwd=REPO_ROOT,
        capture_output=True,
        check=False,
        text=True,
    )


def test_ego_swarm_source_is_tracked_and_not_ignored() -> None:
    ignored = run_git("check-ignore", "-q", EGO_SWARM_SOURCE)
    tracked = run_git("ls-files", "--error-unmatch", EGO_SWARM_SOURCE)

    assert ignored.returncode == 1, ignored.stderr
    assert tracked.returncode == 0, tracked.stderr
