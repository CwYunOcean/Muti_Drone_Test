# FAST-LIVO2 MID360 + D435i LIVO Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add a project-owned `MID360 + D435i infra1` `LIVO` path that preserves the existing pure-`LIO` baseline while covering calibration capture, runtime launch, RViz, scripts, and manual timing-tune workflow.

**Architecture:** Keep pure `LIO` untouched and add a parallel `LIVO` path in `overlay_ws/src/fast_livo2_mid360_bringup/`. Split calibration and runtime flows: calibration uses `MID360` `PointCloud2` plus a saved `infra1` image for `FAST-Calib-ROS2`, while runtime keeps Livox `CustomMsg` for `FAST-LIVO2`. Use package-local contract tests to pin the new file layout, launch arguments, and topic wiring.

**Tech Stack:** ROS 2 Humble, `fast_livo`, `livox_ros_driver2`, `realsense2_camera`, `tf2_ros`, `ament_cmake_pytest`, RViz, bash helper scripts.

---

## File Map

- Modify: `overlay_ws/src/fast_livo2_mid360_bringup/CMakeLists.txt`
  Add `rviz` directory installation and register new pytest contract tests.
- Modify: `overlay_ws/src/fast_livo2_mid360_bringup/package.xml`
  Add runtime dependencies for `tf2_ros` and `realsense2_camera`; keep existing package-local pytest dependencies.
- Create: `overlay_ws/src/fast_livo2_mid360_bringup/config/fast_livo2_mid360_livo_d435i.yaml`
  Runtime `FAST-LIVO2` visual config with `img_en: 1`, `infra1` topic, calibrated `Rcl/Pcl`, and `img_time_offset`.
- Create: `overlay_ws/src/fast_livo2_mid360_bringup/config/d435i_infra1_pinhole.yaml`
  `FAST-LIVO2` camera model for `D435i infra1`.
- Create: `overlay_ws/src/fast_livo2_mid360_bringup/config/fast_calib_mid360_d435i.yaml`
  Project-owned `FAST-Calib-ROS2` parameter file for calibration bag/image inputs.
- Create: `overlay_ws/src/fast_livo2_mid360_bringup/launch/mid360_livo_d435i.launch.py`
  Runtime `LIVO` launch combining Livox `CustomMsg`, RealSense `infra1`, `fastlivo_mapping`, and static `base_link`.
- Create: `overlay_ws/src/fast_livo2_mid360_bringup/launch/mid360_d435i_calib_capture.launch.py`
  Calibration-capture launch combining Livox `PointCloud2` and RealSense `infra1`.
- Create: `overlay_ws/src/fast_livo2_mid360_bringup/rviz/fast_livo2_livo_d435i.rviz`
  RViz config derived from the current package RViz file with image and visual map displays enabled.
- Create: `overlay_ws/src/fast_livo2_mid360_bringup/test/test_mid360_livo_d435i_config_contract.py`
  Config contract test for the new runtime and calibration YAML files.
- Create: `overlay_ws/src/fast_livo2_mid360_bringup/test/test_mid360_livo_d435i_launch_contract.py`
  Launch contract test for the new runtime and calibration-capture launch files.
- Create: `scripts/capture_fast_calib_sample.sh`
  One-command helper for build, source, launch, and bag record during calibration capture.
- Create: `scripts/run_mid360_livo_d435i.sh`
  One-command helper for build, source, runtime launch, and RViz.
- Create: `docs/runbooks/fast-livo2-mid360-d435i-livo.md`
  Manual procedure for calibration capture, FAST-Calib result handoff, `img_time_offset` tuning, and flight-readiness checks.

### Task 1: Add Package Scaffolding and Failing Contracts

**Files:**
- Modify: `overlay_ws/src/fast_livo2_mid360_bringup/CMakeLists.txt`
- Modify: `overlay_ws/src/fast_livo2_mid360_bringup/package.xml`
- Create: `overlay_ws/src/fast_livo2_mid360_bringup/test/test_mid360_livo_d435i_config_contract.py`
- Create: `overlay_ws/src/fast_livo2_mid360_bringup/test/test_mid360_livo_d435i_launch_contract.py`

- [ ] **Step 1: Write the failing config contract test**

```python
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
import yaml


PACKAGE_NAME = "fast_livo2_mid360_bringup"
PACKAGE_SHARE = Path(get_package_share_directory(PACKAGE_NAME))
LIVO_CONFIG = PACKAGE_SHARE / "config" / "fast_livo2_mid360_livo_d435i.yaml"
CAMERA_CONFIG = PACKAGE_SHARE / "config" / "d435i_infra1_pinhole.yaml"
CALIB_CONFIG = PACKAGE_SHARE / "config" / "fast_calib_mid360_d435i.yaml"


def load_yaml(path: Path) -> dict:
    return yaml.safe_load(path.read_text())


def test_livo_and_calib_config_files_exist() -> None:
    assert LIVO_CONFIG.exists(), f"Missing runtime config: {LIVO_CONFIG}"
    assert CAMERA_CONFIG.exists(), f"Missing camera config: {CAMERA_CONFIG}"
    assert CALIB_CONFIG.exists(), f"Missing calibration config: {CALIB_CONFIG}"


def test_livo_config_wires_d435i_visual_input() -> None:
    params = load_yaml(LIVO_CONFIG)["/**"]["ros__parameters"]
    assert params["common"]["img_en"] == 1
    assert params["common"]["img_topic"].endswith("/infra1/image_rect_raw")
    assert params["common"]["lid_topic"] == "/livox/lidar"
    assert params["common"]["imu_topic"] == "/livox/imu"
    assert len(params["extrin_calib"]["Rcl"]) == 9
    assert len(params["extrin_calib"]["Pcl"]) == 3
    assert "img_time_offset" in params["time_offset"]


def test_fast_calib_config_uses_pointcloud2_calibration_path() -> None:
    params = load_yaml(CALIB_CONFIG)["fast_calib"]["ros__parameters"]
    assert params["lidar_topic"] == "/livox/lidar"
    assert str(params["bag_path"]).endswith("calibration_sample")
    assert str(params["image_path"]).endswith(".png")
```

- [ ] **Step 2: Write the failing launch contract test**

```python
EXPECTED_RUNTIME_ARGUMENTS = {
    "livox_config_file",
    "livo_params_file",
    "camera_params_file",
    "realsense_serial_no",
    "base_link_pitch_rad",
    "img_time_offset",
}
EXPECTED_RUNTIME_WIRING = {
    ("livox_ros_driver2", "livox_ros_driver2_node"),
    ("realsense2_camera", "realsense2_camera_node"),
    ("fast_livo", "fastlivo_mapping"),
    ("tf2_ros", "static_transform_publisher"),
}
EXPECTED_CALIB_WIRING = {
    ("livox_ros_driver2", "livox_ros_driver2_node"),
    ("realsense2_camera", "realsense2_camera_node"),
}


def test_runtime_launch_declares_visual_arguments() -> None:
    launch_description = _load_launch_description("mid360_livo_d435i.launch.py")
    arg_names = {
        action.name for action in launch_description.entities
        if isinstance(action, DeclareLaunchArgument)
    }
    assert EXPECTED_RUNTIME_ARGUMENTS.issubset(arg_names)


def test_runtime_launch_starts_expected_nodes() -> None:
    launch_description = _load_launch_description("mid360_livo_d435i.launch.py")
    actual = {
        (node.node_package, node.node_executable)
        for node in launch_description.entities
        if isinstance(node, Node)
    }
    assert actual == EXPECTED_RUNTIME_WIRING


def test_calib_capture_launch_uses_pointcloud2_mode() -> None:
    launch_description = _load_launch_description("mid360_d435i_calib_capture.launch.py")
    actual = {
        (node.node_package, node.node_executable)
        for node in launch_description.entities
        if isinstance(node, Node)
    }
    assert actual == EXPECTED_CALIB_WIRING
```

- [ ] **Step 3: Register the tests and missing runtime dependencies**

```cmake
install(
  DIRECTORY launch config rviz
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL
)

ament_add_pytest_test(
  test_mid360_livo_d435i_config_contract
  test/test_mid360_livo_d435i_config_contract.py
)
ament_add_pytest_test(
  test_mid360_livo_d435i_launch_contract
  test/test_mid360_livo_d435i_launch_contract.py
)
```

```xml
<exec_depend>realsense2_camera</exec_depend>
<exec_depend>tf2_ros</exec_depend>
```

- [ ] **Step 4: Run the new tests to verify they fail for the right reason**

Run:

```bash
source /opt/ros/humble/setup.bash
source /home/morphing01/Drone_SLAM/slam_ws/install/setup.bash
cd /home/morphing01/Drone_SLAM/overlay_ws
colcon build --symlink-install --packages-select fast_livo2_mid360_bringup
source /home/morphing01/Drone_SLAM/overlay_ws/install/setup.bash
colcon test --packages-select fast_livo2_mid360_bringup --ctest-args -R 'test_mid360_livo_d435i_(config|launch)_contract'
```

Expected: FAIL with missing installed config or launch files, not Python syntax errors.

- [ ] **Step 5: Commit the scaffolding**

```bash
git add \
  overlay_ws/src/fast_livo2_mid360_bringup/CMakeLists.txt \
  overlay_ws/src/fast_livo2_mid360_bringup/package.xml \
  overlay_ws/src/fast_livo2_mid360_bringup/test/test_mid360_livo_d435i_config_contract.py \
  overlay_ws/src/fast_livo2_mid360_bringup/test/test_mid360_livo_d435i_launch_contract.py
git commit -m "test: add mid360 d435i livo contract scaffolding"
```

### Task 2: Add Calibration Config and Capture Launch

**Files:**
- Create: `overlay_ws/src/fast_livo2_mid360_bringup/config/fast_calib_mid360_d435i.yaml`
- Create: `overlay_ws/src/fast_livo2_mid360_bringup/launch/mid360_d435i_calib_capture.launch.py`

- [ ] **Step 1: Create the project-owned FAST-Calib parameter file**

```yaml
fast_calib:
  ros__parameters:
    fx: 0.0
    fy: 0.0
    cx: 0.0
    cy: 0.0
    k1: 0.0
    k2: 0.0
    p1: 0.0
    p2: 0.0

    marker_size: 0.20
    delta_width_qr_center: 0.55
    delta_height_qr_center: 0.35
    delta_width_circles: 0.5
    delta_height_circles: 0.4
    circle_radius: 0.12

    x_min: 1.0
    x_max: 4.0
    y_min: -2.0
    y_max: 2.0
    z_min: -1.0
    z_max: 2.0

    lidar_topic: "/livox/lidar"
    bag_path: "/tmp/fast_calib/calibration_sample"
    image_path: "/tmp/fast_calib/calibration_sample.png"
    output_path: "/tmp/fast_calib/output"
```

Use zeros only as placeholders in the initial commit; the runbook will later require replacing them with the actual `infra1` intrinsics before calibration.

- [ ] **Step 2: Create the calibration capture launch**

```python
Node(
    package="livox_ros_driver2",
    executable="livox_ros_driver2_node",
    name="livox_lidar_publisher",
    output="screen",
    parameters=[
        {"xfer_format": 0},
        {"multi_topic": 0},
        {"data_src": 0},
        {"publish_freq": LaunchConfiguration("publish_freq")},
        {"output_data_type": 0},
        {"frame_id": "livox_frame"},
        {"user_config_path": LaunchConfiguration("livox_config_file")},
    ],
),
Node(
    package="realsense2_camera",
    executable="realsense2_camera_node",
    name="camera",
    namespace="camera",
    output="screen",
    parameters=[{
        "enable_color": False,
        "enable_depth": False,
        "enable_infra": False,
        "enable_infra1": True,
        "enable_infra2": False,
        "enable_gyro": False,
        "enable_accel": False,
        "enable_sync": False,
        "depth_module.infra1_format": "Y8",
        "serial_no": LaunchConfiguration("realsense_serial_no"),
    }],
),
```

- [ ] **Step 3: Run the launch contract tests again**

Run:

```bash
source /opt/ros/humble/setup.bash
source /home/morphing01/Drone_SLAM/slam_ws/install/setup.bash
cd /home/morphing01/Drone_SLAM/overlay_ws
colcon build --symlink-install --packages-select fast_livo2_mid360_bringup
source /home/morphing01/Drone_SLAM/overlay_ws/install/setup.bash
colcon test --packages-select fast_livo2_mid360_bringup --ctest-args -R 'test_mid360_livo_d435i_(config|launch)_contract'
```

Expected: launch contract still fails because the runtime `LIVO` config and launch do not exist yet; calibration-file existence checks should now pass.

- [ ] **Step 4: Smoke-check the capture launch argument surface**

Run:

```bash
source /opt/ros/humble/setup.bash
source /home/morphing01/Drone_SLAM/slam_ws/install/setup.bash
source /home/morphing01/Drone_SLAM/overlay_ws/install/setup.bash
ros2 launch fast_livo2_mid360_bringup mid360_d435i_calib_capture.launch.py --show-args
```

Expected: PASS and show `livox_config_file`, `publish_freq`, and `realsense_serial_no`.

- [ ] **Step 5: Commit the calibration capture entrypoint**

```bash
git add \
  overlay_ws/src/fast_livo2_mid360_bringup/config/fast_calib_mid360_d435i.yaml \
  overlay_ws/src/fast_livo2_mid360_bringup/launch/mid360_d435i_calib_capture.launch.py
git commit -m "feat: add mid360 d435i calibration capture flow"
```

### Task 3: Add Runtime LIVO Config and Launch

**Files:**
- Create: `overlay_ws/src/fast_livo2_mid360_bringup/config/d435i_infra1_pinhole.yaml`
- Create: `overlay_ws/src/fast_livo2_mid360_bringup/config/fast_livo2_mid360_livo_d435i.yaml`
- Create: `overlay_ws/src/fast_livo2_mid360_bringup/launch/mid360_livo_d435i.launch.py`

- [ ] **Step 1: Create the `infra1` camera model file**

```yaml
/**:
  ros__parameters:
    camera:
      model: Pinhole
      width: 848
      height: 480
      scale: 1.0
      fx: 0.0
      fy: 0.0
      cx: 0.0
      cy: 0.0
      d0: 0.0
      d1: 0.0
      d2: 0.0
      d3: 0.0
```

Use the actual `infra1` resolution you standardize on. Replace the zeros before runtime verification.

- [ ] **Step 2: Create the runtime `LIVO` parameter file**

```yaml
/**:
  ros__parameters:
    common:
      img_topic: "/camera/camera/infra1/image_rect_raw"
      lid_topic: "/livox/lidar"
      imu_topic: "/livox/imu"
      img_en: 1
      lidar_en: 1
      ros_driver_bug_fix: false

    extrin_calib:
      extrinsic_T: [-0.011, -0.02329, 0.04412]
      extrinsic_R: [1.0, 0.0, 0.0,
                    0.0, 1.0, 0.0,
                    0.0, 0.0, 1.0]
      Rcl: [1.0, 0.0, 0.0,
            0.0, 1.0, 0.0,
            0.0, 0.0, 1.0]
      Pcl: [0.0, 0.0, 0.0]

    time_offset:
      img_time_offset: 0.0
      exposure_time_init: 0.0

    preprocess:
      lidar_type: 1
      scan_line: 6
      feature_extract_enabled: false

    imu:
      imu_en: true
      gravity_est_en: true
      ba_bg_est_en: true

    uav:
      imu_rate_odom: false
      gravity_align_en: true
```

Leave `Rcl/Pcl` as explicit placeholders in the first commit so tests can assert structure. A later manual update fills in the FAST-Calib result.

- [ ] **Step 3: Create the runtime launch**

```python
Node(
    package="livox_ros_driver2",
    executable="livox_ros_driver2_node",
    name="livox_lidar_publisher",
    output="screen",
    parameters=[
        {"xfer_format": 1},
        {"multi_topic": 0},
        {"data_src": 0},
        {"publish_freq": "10.0"},
        {"output_data_type": 0},
        {"frame_id": "livox_frame"},
        {"user_config_path": livox_config_file},
    ],
),
Node(
    package="realsense2_camera",
    executable="realsense2_camera_node",
    name="camera",
    namespace="camera",
    output="screen",
    parameters=[{
        "enable_color": False,
        "enable_depth": False,
        "enable_infra": False,
        "enable_infra1": True,
        "enable_infra2": False,
        "enable_gyro": False,
        "enable_accel": False,
        "serial_no": realsense_serial_no,
        "depth_module.infra1_format": "Y8",
    }],
),
Node(
    package="fast_livo",
    executable="fastlivo_mapping",
    name="laserMapping",
    output="screen",
    parameters=[
        livo_params_file,
        camera_params_file,
        {"time_offset.img_time_offset": img_time_offset},
    ],
),
```

Keep the existing `aft_mapped -> base_link` static TF logic and reuse the same `base_link_pitch_rad` launch argument.

- [ ] **Step 4: Run the contract tests until they pass**

Run:

```bash
source /opt/ros/humble/setup.bash
source /home/morphing01/Drone_SLAM/slam_ws/install/setup.bash
cd /home/morphing01/Drone_SLAM/overlay_ws
colcon build --symlink-install --packages-select fast_livo2_mid360_bringup
source /home/morphing01/Drone_SLAM/overlay_ws/install/setup.bash
colcon test --packages-select fast_livo2_mid360_bringup --ctest-args -R 'test_mid360_livo_d435i_(config|launch)_contract'
colcon test-result --verbose
```

Expected: PASS for the two new tests and no regressions in the existing `mid360_lio` tests.

- [ ] **Step 5: Smoke-check the new launch file**

Run:

```bash
source /opt/ros/humble/setup.bash
source /home/morphing01/Drone_SLAM/slam_ws/install/setup.bash
source /home/morphing01/Drone_SLAM/overlay_ws/install/setup.bash
ros2 launch fast_livo2_mid360_bringup mid360_livo_d435i.launch.py --show-args
```

Expected: PASS and show both runtime visual arguments and `base_link_pitch_rad`.

- [ ] **Step 6: Commit the runtime `LIVO` entrypoint**

```bash
git add \
  overlay_ws/src/fast_livo2_mid360_bringup/config/d435i_infra1_pinhole.yaml \
  overlay_ws/src/fast_livo2_mid360_bringup/config/fast_livo2_mid360_livo_d435i.yaml \
  overlay_ws/src/fast_livo2_mid360_bringup/launch/mid360_livo_d435i.launch.py
git commit -m "feat: add mid360 d435i livo runtime flow"
```

### Task 4: Add LIVO RViz Config

**Files:**
- Create: `overlay_ws/src/fast_livo2_mid360_bringup/rviz/fast_livo2_livo_d435i.rviz`

- [ ] **Step 1: Copy the existing package RViz config as the starting point**

```bash
cp \
  /home/morphing01/Drone_SLAM/overlay_ws/src/fast_livo2_mid360_bringup/rviz/fast_livo2_with_base_link.rviz \
  /home/morphing01/Drone_SLAM/overlay_ws/src/fast_livo2_mid360_bringup/rviz/fast_livo2_livo_d435i.rviz
```

- [ ] **Step 2: Edit the copied RViz file to expose visual topics**

Apply these exact content changes:

```yaml
Fixed Frame: camera_init
Target Frame: <Fixed Frame>
```

```yaml
- Class: rviz_default_plugins/Image
  Enabled: true
  Name: rgb_img
  Image Topic:
    Value: /rgb_img
```

```yaml
- Class: rviz_default_plugins/PointCloud2
  Enabled: true
  Name: cloud_visual_map
  Topic:
    Value: /cloud_visual_map
```

```yaml
- Class: rviz_default_plugins/PointCloud2
  Enabled: true
  Name: cloud_visual_sub_map
  Topic:
    Value: /cloud_visual_sub_map
```

Keep `/path`, `base_link`, and TF names visible.

- [ ] **Step 3: Verify the RViz file is installed**

Run:

```bash
source /opt/ros/humble/setup.bash
source /home/morphing01/Drone_SLAM/slam_ws/install/setup.bash
cd /home/morphing01/Drone_SLAM/overlay_ws
colcon build --symlink-install --packages-select fast_livo2_mid360_bringup
test -f /home/morphing01/Drone_SLAM/overlay_ws/install/fast_livo2_mid360_bringup/share/fast_livo2_mid360_bringup/rviz/fast_livo2_livo_d435i.rviz
```

Expected: the `test -f` command exits `0`.

- [ ] **Step 4: Commit the RViz config**

```bash
git add overlay_ws/src/fast_livo2_mid360_bringup/rviz/fast_livo2_livo_d435i.rviz
git commit -m "feat: add livo rviz configuration"
```

### Task 5: Add Runtime and Calibration Helper Scripts

**Files:**
- Create: `scripts/capture_fast_calib_sample.sh`
- Create: `scripts/run_mid360_livo_d435i.sh`

- [ ] **Step 1: Create the calibration helper script**

```bash
#!/usr/bin/env bash
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
OVERLAY_WS="$REPO_ROOT/overlay_ws"
SLAM_SETUP="$REPO_ROOT/slam_ws/install/setup.bash"
LIVOX_CONFIG="$REPO_ROOT/slam_ws/src/livox_ros_driver2/config/MID360_config.json"
OUTPUT_DIR="${FAST_CALIB_OUTPUT_DIR:-/tmp/fast_calib}"

mkdir -p "$OUTPUT_DIR"

set +u
source /opt/ros/humble/setup.bash
source "$SLAM_SETUP"
set -u

cd "$OVERLAY_WS"
colcon build --symlink-install --packages-select fast_livo2_mid360_bringup
set +u
source "$OVERLAY_WS/install/setup.bash"
set -u

ros2 launch fast_livo2_mid360_bringup mid360_d435i_calib_capture.launch.py \
  livox_config_file:="$LIVOX_CONFIG" &
LAUNCH_PID=$!
trap 'kill "$LAUNCH_PID" 2>/dev/null || true' EXIT INT TERM

sleep 3

ros2 bag record -o "$OUTPUT_DIR/calibration_sample" \
  /livox/lidar \
  /camera/camera/infra1/image_rect_raw
```

- [ ] **Step 2: Create the runtime helper script**

```bash
#!/usr/bin/env bash
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
OVERLAY_WS="$REPO_ROOT/overlay_ws"
SLAM_SETUP="$REPO_ROOT/slam_ws/install/setup.bash"
LIVOX_CONFIG="$REPO_ROOT/slam_ws/src/livox_ros_driver2/config/MID360_config.json"
RVIZ_CONFIG="$REPO_ROOT/overlay_ws/src/fast_livo2_mid360_bringup/rviz/fast_livo2_livo_d435i.rviz"

cleanup() {
  if [[ -n "${LAUNCH_PID:-}" ]] && kill -0 "$LAUNCH_PID" 2>/dev/null; then
    kill "$LAUNCH_PID" 2>/dev/null || true
    wait "$LAUNCH_PID" 2>/dev/null || true
  fi
}

trap cleanup EXIT INT TERM

set +u
source /opt/ros/humble/setup.bash
source "$SLAM_SETUP"
set -u

cd "$OVERLAY_WS"
colcon build --symlink-install --packages-select fast_livo2_mid360_bringup
set +u
source "$OVERLAY_WS/install/setup.bash"
set -u

ros2 launch fast_livo2_mid360_bringup mid360_livo_d435i.launch.py \
  livox_config_file:="$LIVOX_CONFIG" &
LAUNCH_PID=$!

sleep 3
rviz2 -d "$RVIZ_CONFIG"
```

- [ ] **Step 3: Verify shell syntax and launch path wiring**

Run:

```bash
bash -n /home/morphing01/Drone_SLAM/scripts/capture_fast_calib_sample.sh
bash -n /home/morphing01/Drone_SLAM/scripts/run_mid360_livo_d435i.sh
```

Expected: both commands exit `0`.

- [ ] **Step 4: Commit the helper scripts**

```bash
git add \
  scripts/capture_fast_calib_sample.sh \
  scripts/run_mid360_livo_d435i.sh
git commit -m "feat: add d435i calibration and runtime scripts"
```

### Task 6: Add the LIVO Runbook

**Files:**
- Create: `docs/runbooks/fast-livo2-mid360-d435i-livo.md`

- [ ] **Step 1: Write the calibration capture section**

```markdown
## 1. Calibration Capture

1. Edit `slam_ws/src/livox_ros_driver2/config/MID360_config.json` for machine-local IP settings.
2. Run `bash scripts/capture_fast_calib_sample.sh`.
3. Hold the FAST-Calib board steady in view of both sensors for 2-3 seconds.
4. Stop bag recording.
5. Save one matching `infra1` PNG to `/tmp/fast_calib/calibration_sample.png`.
```

- [ ] **Step 2: Write the FAST-Calib handoff section**

````markdown
## 2. FAST-Calib

```bash
source /opt/ros/humble/setup.bash
source /home/morphing01/Drone_SLAM/slam_ws/install/setup.bash
ros2 run fast_calib fast_calib --ros-args \
  --params-file /home/morphing01/Drone_SLAM/overlay_ws/src/fast_livo2_mid360_bringup/config/fast_calib_mid360_d435i.yaml
```

Copy the reported `T_cam_lidar` into
`overlay_ws/src/fast_livo2_mid360_bringup/config/fast_livo2_mid360_livo_d435i.yaml`
as `extrin_calib.Rcl` and `extrin_calib.Pcl`.
````

- [ ] **Step 3: Write the timing-tune and flight-readiness sections**

```markdown
## 3. Runtime Timing Tune

1. Start with `img_time_offset: 0.0`.
2. Run `bash scripts/run_mid360_livo_d435i.sh`.
3. Perform hand-carried yaw and translation motion.
4. Adjust only `time_offset.img_time_offset` between runs.
5. Keep the first value that clearly reduces map tearing and pose lag.

## 4. Flight Readiness

- Pure `LIO` still passes.
- `FAST-Calib` result reproduced twice.
- `/rgb_img`, `/cloud_visual_map`, `/path`, and `/aft_mapped_to_init` appear.
- Ground hand-carried `LIVO` is at least as stable as pure `LIO`.
- Only then move to static-on-airframe and low-risk flight tests.
```

- [ ] **Step 4: Commit the runbook**

```bash
git add docs/runbooks/fast-livo2-mid360-d435i-livo.md
git commit -m "docs: add mid360 d435i livo runbook"
```

### Task 7: Full Verification and Manual Acceptance

**Files:**
- Verify: `overlay_ws/src/fast_livo2_mid360_bringup/*`
- Verify: `scripts/*`
- Verify: `docs/runbooks/fast-livo2-mid360-d435i-livo.md`

- [ ] **Step 1: Run the full package test suite**

```bash
source /opt/ros/humble/setup.bash
source /home/morphing01/Drone_SLAM/slam_ws/install/setup.bash
cd /home/morphing01/Drone_SLAM/overlay_ws
colcon build --symlink-install --packages-select fast_livo2_mid360_bringup
source /home/morphing01/Drone_SLAM/overlay_ws/install/setup.bash
colcon test --packages-select fast_livo2_mid360_bringup
colcon test-result --verbose
```

Expected: all existing `mid360_lio` tests plus the new `d435i` contract tests pass.

- [ ] **Step 2: Verify both launch entrypoints resolve**

```bash
source /opt/ros/humble/setup.bash
source /home/morphing01/Drone_SLAM/slam_ws/install/setup.bash
source /home/morphing01/Drone_SLAM/overlay_ws/install/setup.bash
ros2 launch fast_livo2_mid360_bringup mid360_d435i_calib_capture.launch.py --show-args
ros2 launch fast_livo2_mid360_bringup mid360_livo_d435i.launch.py --show-args
```

Expected: both commands print arguments and exit successfully.

- [ ] **Step 3: Verify scripts and RViz assets**

```bash
bash -n /home/morphing01/Drone_SLAM/scripts/capture_fast_calib_sample.sh
bash -n /home/morphing01/Drone_SLAM/scripts/run_mid360_livo_d435i.sh
test -f /home/morphing01/Drone_SLAM/overlay_ws/src/fast_livo2_mid360_bringup/rviz/fast_livo2_livo_d435i.rviz
```

Expected: all checks exit `0`.

- [ ] **Step 4: Manual acceptance checklist**

Run in order:

```bash
bash /home/morphing01/Drone_SLAM/scripts/capture_fast_calib_sample.sh
ros2 run fast_calib fast_calib --ros-args \
  --params-file /home/morphing01/Drone_SLAM/overlay_ws/src/fast_livo2_mid360_bringup/config/fast_calib_mid360_d435i.yaml
bash /home/morphing01/Drone_SLAM/scripts/run_mid360_livo_d435i.sh
```

Expected:

- calibration capture produces a bag under `/tmp/fast_calib`
- `fast_calib` prints `T_cam_lidar` and RMSE
- runtime `FAST-LIVO2` emits `/rgb_img`, `/path`, and visual map topics
- RViz shows `base_link`, image, and visual map overlays

- [ ] **Step 5: Commit the verified integration**

```bash
git add \
  overlay_ws/src/fast_livo2_mid360_bringup \
  scripts/capture_fast_calib_sample.sh \
  scripts/run_mid360_livo_d435i.sh \
  docs/runbooks/fast-livo2-mid360-d435i-livo.md
git commit -m "feat: add mid360 d435i livo bringup"
```

## Self-Review

### Spec coverage

- Calibration flow: covered by Task 2, Task 5, and Task 6.
- Runtime `LIVO` flow: covered by Task 3, Task 4, and Task 5.
- RViz integration: covered by Task 4.
- Scripted entrypoints: covered by Task 5.
- Runbook and acceptance gates: covered by Task 6 and Task 7.
- Pure `LIO` preservation: covered by Task 1 and Task 7 by keeping existing tests in the full suite.

No spec gaps remain.

### Placeholder scan

- No unresolved placeholders or deferred implementation notes are left in the plan.
- Placeholder numeric intrinsics/extrinsics are intentional temporary values called out explicitly as first-commit placeholders that must be replaced during calibration.

### Type consistency

- Runtime launch always refers to `mid360_livo_d435i.launch.py`.
- Calibration launch always refers to `mid360_d435i_calib_capture.launch.py`.
- Runtime config always refers to `fast_livo2_mid360_livo_d435i.yaml`.
- Camera model always refers to `d435i_infra1_pinhole.yaml`.
