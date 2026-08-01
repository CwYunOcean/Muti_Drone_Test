#!/usr/bin/env python3
"""Parse 0509 GVF+ISMC experiment bags and export key topics to CSV."""

import csv
import os
import sys
from pathlib import Path

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

BAGS_DIR = Path("/home/morphing01/Drone_SLAM/bags")
DRONE_ID = os.environ.get("DRONE_ID", "1")
TOPIC_PREFIX = f"/drone_{DRONE_ID}"
PLANNING_PREFIX = f"/drone_{DRONE_ID}_planning"
FMU_IN_PREFIX = f"{TOPIC_PREFIX}/fmu/in"
FMU_OUT_PREFIX = f"{TOPIC_PREFIX}/fmu/out"

# Topics to extract and their field mappings
# Each entry: (topic, msg_type, [field_group_name, csv_columns, extract_fn])
TOPIC_CONFIGS = [
    {
        "topic": f"{TOPIC_PREFIX}/aft_mapped_to_init_level",
        "msg_type": "nav_msgs/msg/Odometry",
        "csv_name": "odom_level",
        "columns": [
            "t_sec",
            "pos_x", "pos_y", "pos_z",
            "vel_x", "vel_y", "vel_z",
            "qx", "qy", "qz", "qw",
        ],
        "extract": lambda msg, t: [
            t,
            msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z,
            msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.linear.z,
            msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z, msg.pose.pose.orientation.w,
        ],
    },
    {
        "topic": "/gvf/reference",
        "msg_type": "gvf_path_following_msgs/msg/GVFReference",
        "csv_name": "gvf_reference",
        "columns": [
            "t_sec",
            "vel_x", "vel_y", "vel_z",
            "yaw", "yaw_rate",
            "phi1", "phi2",
        ],
        "extract": lambda msg, t: [
            t,
            msg.desired_velocity.x, msg.desired_velocity.y, msg.desired_velocity.z,
            msg.desired_yaw, msg.desired_yaw_rate,
            msg.phi1, msg.phi2,
        ],
    },
    {
        "topic": f"{PLANNING_PREFIX}/pos_cmd",
        "msg_type": "quadrotor_msgs/msg/PositionCommand",
        "csv_name": "pos_cmd",
        "columns": [
            "t_sec",
            "pos_x", "pos_y", "pos_z",
            "vel_x", "vel_y", "vel_z",
            "acc_x", "acc_y", "acc_z",
            "yaw", "yaw_dot",
        ],
        "extract": lambda msg, t: [
            t,
            msg.position.x, msg.position.y, msg.position.z,
            msg.velocity.x, msg.velocity.y, msg.velocity.z,
            msg.acceleration.x, msg.acceleration.y, msg.acceleration.z,
            msg.yaw, msg.yaw_dot,
        ],
    },
    {
        "topic": f"{FMU_IN_PREFIX}/trajectory_setpoint",
        "msg_type": "px4_msgs/msg/TrajectorySetpoint",
        "csv_name": "px4_setpoint",
        "columns": [
            "t_sec",
            "pos_x", "pos_y", "pos_z",
            "vel_x", "vel_y", "vel_z",
            "acc_x", "acc_y", "acc_z",
            "yaw", "yawspeed",
        ],
        "extract": lambda msg, t: [
            t,
            msg.position[0], msg.position[1], msg.position[2],
            msg.velocity[0], msg.velocity[1], msg.velocity[2],
            msg.acceleration[0], msg.acceleration[1], msg.acceleration[2],
            msg.yaw, msg.yawspeed,
        ],
    },
    {
        "topic": f"{FMU_OUT_PREFIX}/vehicle_odometry",
        "msg_type": "px4_msgs/msg/VehicleOdometry",
        "csv_name": "px4_odometry",
        "columns": [
            "t_sec",
            "pos_x", "pos_y", "pos_z",
            "vel_x", "vel_y", "vel_z",
            "qx", "qy", "qz", "qw",
        ],
        "extract": lambda msg, t: [
            t,
            msg.position[0], msg.position[1], msg.position[2],
            msg.velocity[0], msg.velocity[1], msg.velocity[2],
            msg.q[0], msg.q[1], msg.q[2], msg.q[3],
        ],
    },
]


def open_bag_reader(bag_path: str):
    storage_options = rosbag2_py.StorageOptions(uri=bag_path, storage_id="sqlite3")
    converter_options = rosbag2_py.ConverterOptions("", "")
    reader = rosbag2_py.SequentialReader()
    reader.open(storage_options, converter_options)
    return reader


def process_bag(bag_path: Path):
    bag_name = bag_path.name
    metadata_path = bag_path / "metadata.yaml"
    if not metadata_path.exists():
        print(f"  Skipping {bag_name}: no metadata.yaml (incomplete bag)")
        return

    print(f"Processing {bag_name} ...")
    reader = open_bag_reader(str(bag_path))

    topic_type_map = {}
    for t in reader.get_all_topics_and_types():
        topic_type_map[t.name] = t.type

    writers = {}
    files = {}
    config_by_topic = {}

    for cfg in TOPIC_CONFIGS:
        topic = cfg["topic"]
        if topic not in topic_type_map:
            continue
        csv_path = bag_path / f"{bag_name}_{cfg['csv_name']}.csv"
        f = open(csv_path, "w", newline="")
        w = csv.writer(f)
        w.writerow(cfg["columns"])
        writers[topic] = (w, cfg)
        files[topic] = f
        config_by_topic[topic] = cfg

    msg_type_cache = {}
    count = 0
    while reader.has_next():
        topic, data, timestamp = reader.read_next()
        if topic not in writers:
            continue

        w, cfg = writers[topic]
        msg_type_str = topic_type_map[topic]
        if msg_type_str not in msg_type_cache:
            msg_type_cache[msg_type_str] = get_message(msg_type_str)
        msg = deserialize_message(data, msg_type_cache[msg_type_str])

        t_sec = timestamp * 1e-9
        try:
            row = cfg["extract"](msg, t_sec)
            w.writerow(row)
            count += 1
        except Exception as e:
            pass

    for f in files.values():
        f.close()

    print(f"  Exported {count} messages to {len(files)} CSV files")


def main():
    bag_dirs = sorted(BAGS_DIR.glob("gvf_ismc_20260509_*"))
    if not bag_dirs:
        print("No 0509 bags found")
        return

    print(f"Found {len(bag_dirs)} bags from 2026-05-09\n")
    for bag_path in bag_dirs:
        process_bag(bag_path)
    print("\nDone.")


if __name__ == "__main__":
    main()
