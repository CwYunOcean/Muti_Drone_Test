# UAV Formation Bringup

## Naming contract

The formation uses 1-based vehicle ids. Vehicle `N` owns the ROS namespace
`/drone_N`. PX4 topics therefore use paths such as:

```text
/drone_1/fmu/out/vehicle_local_position
/drone_1/fmu/in/trajectory_setpoint
```

The PX4 SITL `-i N` option selects the PX4 instance and system-id offset. It
does not select the ROS namespace by itself. The SITL launch sets
`PX4_UXRCE_DDS_NS=drone_N` for each instance.

For real hardware, start each PX4 client with the same namespace, for example:

```text
uxrce_dds_client start -t udp -p 8888 -h <agent-host> -n drone_1
```

## SITL

Build and start the standard controller:

```bash
cd ~/Drone_SLAM/uav_formation_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select px4_msgs distribute_control
source install/setup.bash
ros2 launch distribute_control multi_uav_launch.py total_uavs:=3 px4_dir:=~/PX4/PX4-Autopilot
```

The interactive double-W entry point is `multi_uav_launch_2.py`.
Setpoints are not armed automatically. Publish `/start_and_stop` and use the
normal PX4/QGroundControl arming procedure after confirming all vehicle-status,
odometry, and command topics are healthy.

## Real vehicle

Run one controller per vehicle, with the PX4 `MAV_SYS_ID` supplied explicitly:

```bash
DRONE_ID=1 TARGET_SYSTEM=1 ~/Drone_SLAM/scripts/run_formation_real.sh
```

Use `TARGET_SYSTEM` for the configured PX4 system id; it is independent of the
ROS namespace string. The real launch defaults to `control.auto_arm=false`.

## Health checks

```bash
ros2 topic list | rg '/drone_[0-9]+/fmu/(in|out)/'
ros2 topic echo /drone_1/fmu/out/vehicle_status --once
ros2 topic hz /drone_1/fmu/out/vehicle_local_position
ros2 topic hz /drone_1/fmu/in/offboard_control_mode
```

The controller holds zero velocity when local odometry is stale, an expected
neighbor has timed out, or PX4 has not reported Offboard mode. Automatic arming
is available only through an explicit `control.auto_arm=true` override after
the pre-flight checks are complete.
