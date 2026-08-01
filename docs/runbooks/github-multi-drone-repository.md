# GitHub Multi-Drone Repository

## Repository Boundary

`Drone_SLAM` version-controls project-owned ROS packages, the shared
EGO-Swarm navigation source, launch scripts, documentation, tests, source
manifests, and example configuration files. It does not version-control
generated ROS output, logs, bags, PX4 parameter exports, hardware calibration,
or external upstream trees restored from manifests.

```text
Drone_SLAM/
  overlay_ws/src/                 project-owned bridge and bringup packages
  nav_ws/src/ego-swarm-ros2/      vendored shared EGO-Swarm navigation source
  uav_formation_ws/src/distribute_control/
                                  project-owned formation controller
  manifests/                      pinned upstream source revisions
  scripts/                        host-independent launch helpers
  config/*.example                tracked templates
  config/robot.env                ignored per-robot identity
  config/MID360_config.json       ignored per-robot LiDAR network settings
  config/robot.params.yaml        ignored per-robot geometry calibration
```

The same `main` branch runs on all aircraft. Never create a `drone_1` or
`drone_2` code branch for addresses, vehicle ids, or calibration. Those are
deployment data, not source variants.

## First GitHub Push

Create an empty GitHub repository without a README, `.gitignore`, or license.
From the `Drone_SLAM` root, inspect the first commit before staging it:

```bash
git status --short
git add .
git diff --cached --stat
git diff --cached --name-only
git commit -m "chore: initialize drone slam integration repository"
git remote add origin git@github.com:<organization>/Drone_SLAM.git
git push -u origin main
```

Use the HTTPS remote instead when SSH keys are not configured. Do not force
push the `main` branch. GitHub branch protection should require pull requests
for changes that affect flight code or launch behavior.

The `.gitignore` intentionally excludes `slam_ws/src`, `livox_ws`, the two
vendored `px4_msgs` copies, build products, rosbags, telemetry logs, and the
current PX4 parameter export. `nav_ws/src/ego-swarm-ros2` is an intentional
exception: it is shared source and must appear in source-change commits. If an
external upstream tree or generated artifact appears in the staged file list,
stop and correct the ignore rule before committing.

## New Aircraft Setup

All aircraft use the same operating system, ROS distribution, PX4 firmware
release, and project commit. Start from a clean clone:

```bash
git clone git@github.com:<organization>/Drone_SLAM.git ~/Drone_SLAM
cd ~/Drone_SLAM

vcs import livox_ws/src < manifests/livox_ws.repos
vcs import slam_ws/src < manifests/slam_ws.repos
vcs import uav_formation_ws/src < manifests/uav_formation_ws.repos

cp config/robot.env.example config/robot.env
cp config/MID360_config.json.example config/MID360_config.json
cp config/robot.params.yaml.example config/robot.params.yaml
```

Before building a PX4 bridge, select one PX4 firmware commit and import its
matching `px4_msgs` commit into every workspace that builds PX4 nodes. The
existing `overlay_ws/src/px4_msgs` and `uav_formation_ws/src/px4_msgs` copies
are not ABI-compatible, so they are deliberately excluded from the GitHub
repository. Record the chosen SHA in `manifests/` before calling that build a
release. Rebuild every affected workspace after changing `px4_msgs`.

Build in dependency order after the external source import. The exact packages
depend on the active stack, but the usual order is Livox, SLAM, navigation
(already present in the clone), overlay, then formation. Source each completed
workspace before building the next one.

## Existing Aircraft Migration

Before an existing aircraft first pulls the commit that vendors navigation
source, preserve any ignored local copy for comparison. The pull may replace
files below `nav_ws/src/ego-swarm-ros2` with the shared baseline.

```bash
cd ~/Drone_SLAM
cp -a nav_ws/src/ego-swarm-ros2 ~/ego-swarm-ros2-before-vendoring
git pull --ff-only origin main
```

Compare the backup with the tracked source before deleting it. A difference
that is required by all aircraft belongs in a reviewed shared commit; a change
that only describes one aircraft belongs in the ignored `config/` files, not
in EGO-Swarm source.

## Only Per-Aircraft Edits

Each aircraft edits only the three ignored files under `config/`.

| File | Values to set | Why it is local |
| --- | --- | --- |
| `robot.env` | `DRONE_ID`, `PX4_DDS_NAMESPACE`, `TARGET_SYSTEM`, `ROS_DOMAIN_ID`, `PX4_DIR` | namespace, MAVLink system id, DDS domain, and local PX4 checkout |
| `MID360_config.json` | companion NIC address and MID360 address | each LiDAR Ethernet link has its own addresses |
| `robot.params.yaml` | LiDAR mount angle and shared-world origin/yaw | physical installation and takeoff position differ per aircraft |

`TARGET_SYSTEM` is the PX4 `MAV_SYS_ID`, not a ROS namespace. The namespace is
formed from `DRONE_ID`, for example `DRONE_ID=2` maps to
`/drone_2/fmu/in/...` and `/drone_2/fmu/out/...`. Set the PX4 XRCE-DDS client
namespace to the same `drone_2` string. The formation SITL launch does this
through `PX4_UXRCE_DDS_NS`; real hardware needs the PX4 client start argument
`-n drone_2`.

The launch helpers source `config/robot.env` automatically. An explicit
environment value remains higher priority for a one-off diagnostic, for
example:

```bash
DRONE_ID=3 TARGET_SYSTEM=13 ./scripts/run_formation_real.sh
```

Do not use this temporary override as the normal deployment mechanism. Put the
verified values back in the local `robot.env` file.

## PX4 Board Setup

Once per aircraft, configure the PX4 board itself:

1. Set a unique `MAV_SYS_ID` and put the same number in local `TARGET_SYSTEM`.
2. Start the XRCE-DDS client on the correct serial/UDP transport using
   `-n drone_<DRONE_ID>`.
3. Keep all collaborating aircraft on the same `UXRCE_DDS_DOM_ID` and ROS
   `ROS_DOMAIN_ID`.
4. Calibrate IMU, compass, radio, battery, and airframe on that aircraft.
   These calibration exports must remain local and are never imported from a
   different airframe.
5. Record the client transport device and baud rate in the aircraft operations
   note. A serial port name is hardware-specific.

Import only a reviewed shared PX4 parameter subset. Never import another
aircraft's complete `mav.parm`: it includes sensor calibration and identity
data in addition to common flight settings.

## Normal Update Procedure

On a development machine, make source changes on a feature branch, test them,
then merge them into `main`. On an aircraft, only update a clean worktree:

```bash
cd ~/Drone_SLAM
git status --short
git pull --ff-only origin main
```

`git status --short` must be empty before the pull. The three files in
`config/` are ignored, so they do not block updates. Rebuild the workspace that
contains changed packages, source its new `install/setup.bash`, and perform a
ground-level topic health check before flight.

When a field change is genuinely reusable, commit it from a development
machine as a feature branch and review it before merging. This includes a
change below `nav_ws/src/ego-swarm-ros2`. When it describes only one airframe,
move it to `config/robot.env`,
`config/MID360_config.json`, or `config/robot.params.yaml` instead of editing
tracked source.

## Release Gate

Before allowing a commit onto flight aircraft, verify all of the following:

- every PX4 node was rebuilt against the same PX4-compatible `px4_msgs` SHA;
- all vehicles show their own `/drone_N/fmu/in` and `/drone_N/fmu/out` topics;
- every aircraft has a distinct `MAV_SYS_ID` and matching `TARGET_SYSTEM`;
- all vehicles use one tested PX4 firmware release and ROS distribution;
- LiDAR IPs, world origins, and mounting angles came from ignored local files;
- time synchronization and DDS discovery work before arming;
- no bag, log, build directory, calibration export, or unexpected external
  source clone is staged.
