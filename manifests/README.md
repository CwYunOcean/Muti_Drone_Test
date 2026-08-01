# Upstream Source Locks

Import the pinned external sources into an empty workspace source directory:

```bash
vcs import livox_ws/src < manifests/livox_ws.repos
vcs import uav_formation_ws/src < manifests/uav_formation_ws.repos
```

`nav_ws/src/ego-swarm-ros2` and `slam_ws/src` are vendored directly in this
repository rather than restored from manifests. A clean clone already contains
the navigation and SLAM sources; see `nav_ws/UPSTREAM.md` and
`slam_ws/UPSTREAM.md` for their reference upstreams and licenses.

`px4_msgs` is intentionally not listed for `overlay_ws`. The existing overlay
and formation workspaces contain different message definitions, and PX4 DDS
requires the installed `px4_msgs` commit to match the PX4 firmware message ABI.
Before a release, choose one PX4 firmware commit, record its matching
`px4_msgs` SHA here, import that SHA into every workspace that builds PX4
bridges, then rebuild all of those workspaces from clean build directories.

Do not change a `version` value to a moving branch name. A source upgrade is a
release change: update the SHA, rebuild the affected workspace, and record the
test result in the release notes.
