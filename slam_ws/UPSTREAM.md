# Vendored SLAM Source Baseline

`slam_ws/src` is version-controlled directly by the root `Drone_SLAM`
repository. A clean clone contains the shared FAST-LIO, FAST-LIVO, Livox,
calibration, and support source used by this stack and must not run `vcs
import` for `slam_ws`.

The vendored baseline contains these source directories:

```text
FAST-Calib-ROS2/
FAST-LIVO2/
FAST_LIO/
Livox-SDK2/
livox_ros_driver2/
rpg_vikit/
```

The source tree had no nested Git metadata when it was first added. This
repository commit is therefore the authoritative reproducible baseline. A
historical source manifest mentioned FAST-LIO and EGO-Planner upstream
references, but it cannot establish an exact base SHA for this complete source
tree. Do not claim an upstream SHA without a separate source comparison.

Each vendored component retains its own upstream license. Preserve those
licenses and identify material source changes in their commits. Changes under
`slam_ws/src` are shared flight-source changes: make them on a feature branch,
rebuild affected SLAM and overlay packages, and never store airframe-specific
LiDAR addresses, calibration output, runtime logs, or PCD data in tracked
source.
