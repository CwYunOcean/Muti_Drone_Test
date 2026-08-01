from __future__ import annotations

import os
import subprocess
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[1]
CONFIG_SCRIPT = REPO_ROOT / "scripts" / "load_robot_config.sh"


def load_config(extra_env: dict[str, str] | None = None) -> dict[str, str]:
    environment = os.environ.copy()
    environment.pop("ROBOT_CONFIG", None)
    environment.pop("DRONE_ID", None)
    environment.pop("TARGET_SYSTEM", None)
    environment.pop("PX4_DDS_NAMESPACE", None)
    environment.pop("ROS_DOMAIN_ID", None)
    environment.pop("LIVOX_CONFIG_FILE", None)
    if extra_env:
        environment.update(extra_env)

    result = subprocess.run(
        [
            "bash",
            "-c",
            'set -euo pipefail; source "$1"; env | rg "^(DRONE_ID|TARGET_SYSTEM|PX4_DDS_NAMESPACE|ROS_DOMAIN_ID|LIVOX_CONFIG_FILE)="',
            "bash",
            str(CONFIG_SCRIPT),
        ],
        check=True,
        capture_output=True,
        cwd=REPO_ROOT,
        env=environment,
        text=True,
    )
    return dict(line.split("=", 1) for line in result.stdout.splitlines())


def test_uses_stable_defaults_without_local_config(tmp_path: Path) -> None:
    values = load_config({"ROBOT_CONFIG": str(tmp_path / "missing.env")})

    assert values["DRONE_ID"] == "1"
    assert values["TARGET_SYSTEM"] == ""
    assert values["PX4_DDS_NAMESPACE"] == "drone_1"
    assert values["ROS_DOMAIN_ID"] == "0"
    assert values["LIVOX_CONFIG_FILE"] == ""


def test_loads_machine_local_values(tmp_path: Path) -> None:
    config_file = tmp_path / "robot.env"
    config_file.write_text(
        "\n".join(
            [
                "DRONE_ID=2",
                "TARGET_SYSTEM=12",
                "PX4_DDS_NAMESPACE=drone_2",
                "ROS_DOMAIN_ID=5",
                "LIVOX_CONFIG_FILE=/etc/drone_2_mid360.json",
                "",
            ]
        )
    )

    values = load_config({"ROBOT_CONFIG": str(config_file)})

    assert values == {
        "DRONE_ID": "2",
        "TARGET_SYSTEM": "12",
        "PX4_DDS_NAMESPACE": "drone_2",
        "ROS_DOMAIN_ID": "5",
        "LIVOX_CONFIG_FILE": "/etc/drone_2_mid360.json",
    }


def test_command_environment_overrides_machine_local_values(tmp_path: Path) -> None:
    config_file = tmp_path / "robot.env"
    config_file.write_text(
        "\n".join(
            [
                "DRONE_ID=1",
                "TARGET_SYSTEM=1",
                "PX4_DDS_NAMESPACE=drone_1",
                "ROS_DOMAIN_ID=0",
                "LIVOX_CONFIG_FILE=/etc/drone_1_mid360.json",
                "",
            ]
        )
    )

    values = load_config(
        {
            "ROBOT_CONFIG": str(config_file),
            "DRONE_ID": "3",
            "TARGET_SYSTEM": "13",
            "PX4_DDS_NAMESPACE": "drone_3",
            "ROS_DOMAIN_ID": "6",
            "LIVOX_CONFIG_FILE": "/tmp/drone_3_mid360.json",
        }
    )

    assert values == {
        "DRONE_ID": "3",
        "TARGET_SYSTEM": "13",
        "PX4_DDS_NAMESPACE": "drone_3",
        "ROS_DOMAIN_ID": "6",
        "LIVOX_CONFIG_FILE": "/tmp/drone_3_mid360.json",
    }
