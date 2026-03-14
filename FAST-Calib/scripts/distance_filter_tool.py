#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ROS 2 bag -> PCD converter + interactive distance filter selector.

Reads a ROS 2 bag (.db3 / .mcap directory), extracts PointCloud2 messages,
exports an ASCII PCD with intensity, then opens an Open3D window for
interactive 4-corner selection to define distance filter bounds.

Dependencies:
    pip install rosbags open3d numpy

Usage:
    python distance_filter_tool.py /path/to/ros2_bag_dir [/path/to/output_dir]
"""

import os
import sys
import struct
import numpy as np

try:
    from rosbags.rosbag2 import Reader
    from rosbags.typesys import get_types_from_msg, register_types, Stores
    from rosbags.typesys.stores.ros2_humble import (
        sensor_msgs__msg__PointCloud2 as PointCloud2Type,
    )
except ImportError:
    print("[ERROR] Please install rosbags: pip install rosbags", file=sys.stderr)
    sys.exit(1)

try:
    import open3d as o3d
except ImportError:
    print("[ERROR] Please install open3d: pip install open3d", file=sys.stderr)
    sys.exit(1)


def save_pcd_with_intensity(points, intensities, output_path):
    N = len(points)
    header = f"""# .PCD v0.7 - Point Cloud Data file format
VERSION 0.7
FIELDS x y z intensity
SIZE 4 4 4 4
TYPE F F F F
COUNT 1 1 1 1
WIDTH {N}
HEIGHT 1
POINTS {N}
DATA ascii
"""
    with open(output_path, 'w') as f:
        f.write(header)
        for (x, y, z), inten in zip(points, intensities):
            f.write(f"{x} {y} {z} {inten}\n")
    print(f"[PCD] Saved point cloud with intensity to: {output_path}")


def find_field(fields, name):
    """Find a field by name in PointCloud2 fields list."""
    for f in fields:
        if f.name == name:
            return f
    return None


def find_intensity_field(fields):
    candidates = ["intensity", "reflectivity", "i", "ref"]
    for f in fields:
        if f.name.lower() in candidates:
            return f
    return None


DTYPE_MAP = {
    (1, 1): ('B', 1),  # UINT8
    (1, 2): ('b', 1),  # INT8
    (2, 1): ('H', 2),  # UINT16
    (2, 2): ('h', 2),  # INT16
    (4, 1): ('I', 4),  # UINT32
    (4, 2): ('i', 4),  # INT32
    (4, 7): ('f', 4),  # FLOAT32
    (8, 7): ('d', 8),  # FLOAT64
}


def read_field_value(data, offset, field):
    key = (field.count if hasattr(field, 'count') and field.count else 1,)
    dtype_key = (field.offset, field.datatype) if hasattr(field, 'datatype') else None

    size = field.count if hasattr(field, 'count') else 1
    fmt_key = (size, field.datatype)
    if fmt_key in DTYPE_MAP:
        fmt, sz = DTYPE_MAP[fmt_key]
    else:
        fmt, sz = 'f', 4

    try:
        return struct.unpack_from(fmt, data, offset + field.offset)[0]
    except struct.error:
        return 0.0


def convert_bag_to_pcd(bag_path, output_dir, lidar_topic=None, pcd_name="cloud.pcd"):
    """Read all PointCloud2 messages from a ROS 2 bag and merge into a PCD."""
    print(f"[Bag] Opening ROS 2 bag: {bag_path}")

    all_points = []
    all_intensities = []

    with Reader(bag_path) as reader:
        # Auto-detect lidar topic if not specified
        pc2_topics = []
        for conn in reader.connections:
            if conn.msgtype == 'sensor_msgs/msg/PointCloud2':
                pc2_topics.append(conn.topic)

        if not pc2_topics:
            print("[ERROR] No PointCloud2 topics found in bag.", file=sys.stderr)
            return None

        if lidar_topic and lidar_topic in pc2_topics:
            target_topic = lidar_topic
        else:
            target_topic = pc2_topics[0]
            if lidar_topic:
                print(f"[WARN] Topic '{lidar_topic}' not found. Using '{target_topic}' instead.")

        print(f"[Bag] Reading PointCloud2 from topic: {target_topic}")

        intensity_field_detected = False
        intensity_field = None

        for conn, timestamp, rawdata in reader.messages():
            if conn.topic != target_topic:
                continue

            from rosbags.serde import deserialize_cdr
            msg = deserialize_cdr(rawdata, conn.msgtype)

            if not intensity_field_detected:
                intensity_field = find_intensity_field(msg.fields)
                intensity_field_detected = True
                if intensity_field:
                    print(f"[Bag] Detected intensity field: {intensity_field.name}")

            x_field = find_field(msg.fields, 'x')
            y_field = find_field(msg.fields, 'y')
            z_field = find_field(msg.fields, 'z')
            if not (x_field and y_field and z_field):
                continue

            point_step = msg.point_step
            data = bytes(msg.data)
            n_points = len(data) // point_step

            for i in range(n_points):
                offset = i * point_step
                try:
                    x = struct.unpack_from('f', data, offset + x_field.offset)[0]
                    y = struct.unpack_from('f', data, offset + y_field.offset)[0]
                    z = struct.unpack_from('f', data, offset + z_field.offset)[0]
                except struct.error:
                    continue

                if np.isnan(x) or np.isnan(y) or np.isnan(z):
                    continue

                inten = 0.0
                if intensity_field:
                    try:
                        fmt_key = (4, intensity_field.datatype)
                        if fmt_key in DTYPE_MAP:
                            fmt, _ = DTYPE_MAP[fmt_key]
                        else:
                            fmt = 'f'
                        inten = struct.unpack_from(fmt, data, offset + intensity_field.offset)[0]
                    except struct.error:
                        inten = 0.0

                all_points.append([x, y, z])
                all_intensities.append(inten)

    if not all_points:
        print("[ERROR] No points extracted from bag.", file=sys.stderr)
        return None

    print(f"[Bag] Total points extracted: {len(all_points)}")
    output_path = os.path.join(output_dir, pcd_name)
    save_pcd_with_intensity(all_points, all_intensities, output_path)
    return output_path


def select_and_save_points(pcd_folder, target_pcd_name):
    """Open3D interactive point selection for distance filter bounds."""
    pcd_path = os.path.join(pcd_folder, target_pcd_name)
    if not os.path.isfile(pcd_path):
        print(f"[ERROR] PCD file not found: {pcd_path}", file=sys.stderr)
        return

    pcd = o3d.io.read_point_cloud(pcd_path)
    if not pcd.has_points():
        print(f"[ERROR] {target_pcd_name} has no points, skipped", file=sys.stderr)
        return

    print(f"\nProcessing: {target_pcd_name}")
    print("Hold Shift + left-click to select at least 4 points on the calibration target, then press Q to close")

    vis = o3d.visualization.VisualizerWithEditing()
    vis.create_window(window_name=f"Select points - {target_pcd_name}")
    vis.add_geometry(pcd)
    vis.run()
    vis.destroy_window()

    selected_indices = vis.get_picked_points()

    if not selected_indices:
        print(f"[ERROR] No points selected for {target_pcd_name}", file=sys.stderr)
        return

    if len(selected_indices) < 4:
        print(f"[ERROR] Only {len(selected_indices)} points selected, need at least 4", file=sys.stderr)
        return

    selected_indices = selected_indices[:4]
    all_points = np.asarray(pcd.points)
    selected_points = all_points[selected_indices, :]

    mins = selected_points.min(axis=0)
    maxs = selected_points.max(axis=0)

    x_min = mins[0] - 0.2
    x_max = maxs[0] + 0.2
    y_min = mins[1] - 0.2
    y_max = maxs[1] + 0.2
    z_min = mins[2] - 0.2
    z_max = maxs[2] + 0.2

    base_name = os.path.splitext(target_pcd_name)[0]
    save_file = os.path.join(pcd_folder, f"{base_name}.txt")

    with open(save_file, 'w') as f:
        f.write("# 4 selected points (x y z)\n")
        for p in selected_points:
            f.write(f"{p[0]:.6f} {p[1]:.6f} {p[2]:.6f}\n")

        f.write("# range values in order:\n")
        f.write(f"x_min: {x_min:.1f}\n")
        f.write(f"x_max: {x_max:.1f}\n")
        f.write(f"y_min: {y_min:.1f}\n")
        f.write(f"y_max: {y_max:.1f}\n")
        f.write(f"z_min: {z_min:.1f}\n")
        f.write(f"z_max: {z_max:.1f}\n")

    print(f"[Save] Saved selection and range to: {save_file}")
    print("Done.")


if __name__ == "__main__":
    if len(sys.argv) > 1:
        bag_path = sys.argv[1]
    else:
        bag_path = os.getcwd()
        print(f"No bag path specified, using current directory: {bag_path}")

    if len(sys.argv) > 2:
        output_dir = sys.argv[2]
    else:
        output_dir = os.getcwd()
        print(f"No output directory specified, using current directory: {output_dir}")

    if not os.path.exists(bag_path):
        print(f"[ERROR] Bag path '{bag_path}' does not exist", file=sys.stderr)
        sys.exit(1)

    if not os.path.isdir(output_dir):
        print(f"[ERROR] Output directory '{output_dir}' does not exist", file=sys.stderr)
        sys.exit(1)

    lidar_topic = None
    if len(sys.argv) > 3:
        lidar_topic = sys.argv[3]

    pcd_path = convert_bag_to_pcd(
        bag_path=bag_path,
        output_dir=output_dir,
        lidar_topic=lidar_topic,
        pcd_name="pointcloud.pcd"
    )

    if pcd_path is None:
        print("[ERROR] PCD generation failed.", file=sys.stderr)
        sys.exit(1)

    select_and_save_points(
        pcd_folder=output_dir,
        target_pcd_name=os.path.basename(pcd_path)
    )
