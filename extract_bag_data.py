#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ROS 2 Bag Data Extractor for Calibration Pipeline

Extracts sensor data from a ROS 2 bag (.db3 / .mcap) into the directory
structure expected by the calibration scripts:

    output_dir/
    ├── camera_images/
    │   ├── front/           # images from /camera_front/image_raw
    │   ├── front_left/      # images from /camera_front_left/image_raw
    │   └── ...
    ├── lidar_points/
    │   ├── 000000.pcd       # point clouds from /rslidar_points
    │   └── ...
    └── imu_data/
        └── imu.csv          # IMU data from /imu/data

Usage:
    python extract_bag_data.py /path/to/ros2_bag [--output /path/to/output]

    # With custom topic mapping:
    python extract_bag_data.py /path/to/ros2_bag \\
        --lidar-topic /rslidar_points \\
        --camera-topics front=/cam0/image_raw front_left=/cam1/image_raw \\
        --imu-topic /imu/data

    # Extract only specific data types:
    python extract_bag_data.py /path/to/ros2_bag --only lidar camera

Dependencies:
    pip install rosbags numpy open3d
"""

import argparse
import os
import struct
import sys
from pathlib import Path

import numpy as np

try:
    from rosbags.rosbag2 import Reader
    from rosbags.serde import deserialize_cdr
except ImportError:
    print("ERROR: pip install rosbags", file=sys.stderr)
    sys.exit(1)


# ---------------------------------------------------------------------------
# PointCloud2 helpers
# ---------------------------------------------------------------------------

DTYPE_STRUCT = {
    1: {1: 'B', 2: 'b'},  # size 1
    2: {1: 'H', 2: 'h'},  # size 2
    4: {1: 'I', 2: 'i', 7: 'f'},  # size 4
    8: {7: 'd'},  # size 8
}


def _field_fmt(field):
    """Return struct format char for a PointField."""
    size = {1: 1, 2: 2, 3: 4, 4: 8, 7: 4, 8: 8}.get(field.datatype, 4)
    return DTYPE_STRUCT.get(size, {}).get(field.datatype, 'f')


def pc2_to_numpy(msg):
    """Convert a deserialized PointCloud2 message to (N,3+) numpy array.

    Returns dict with 'xyz' (N,3), and optionally 'intensity' (N,),
    'ring' (N,) arrays.
    """
    fields_by_name = {f.name: f for f in msg.fields}
    point_step = msg.point_step
    data = bytes(msg.data)
    n = len(data) // point_step

    result = {}
    xyz = np.empty((n, 3), dtype=np.float32)

    for axis_i, axis in enumerate(('x', 'y', 'z')):
        f = fields_by_name[axis]
        for i in range(n):
            xyz[i, axis_i] = struct.unpack_from('f', data, i * point_step + f.offset)[0]

    valid = ~np.isnan(xyz).any(axis=1)
    result['xyz'] = xyz[valid]

    for extra in ('intensity', 'reflectivity', 'ring'):
        if extra in fields_by_name:
            f = fields_by_name[extra]
            fmt = _field_fmt(f)
            arr = np.empty(n, dtype=np.float32)
            for i in range(n):
                arr[i] = struct.unpack_from(fmt, data, i * point_step + f.offset)[0]
            result[extra] = arr[valid]

    return result


def save_pcd_ascii(path, xyz, intensity=None):
    """Save point cloud as ASCII PCD file."""
    n = len(xyz)
    has_i = intensity is not None and len(intensity) == n

    if has_i:
        fields = "x y z intensity"
        sizes = "4 4 4 4"
        types = "F F F F"
        counts = "1 1 1 1"
    else:
        fields = "x y z"
        sizes = "4 4 4"
        types = "F F F"
        counts = "1 1 1"

    header = (
        f"# .PCD v0.7 - Point Cloud Data file format\n"
        f"VERSION 0.7\n"
        f"FIELDS {fields}\n"
        f"SIZE {sizes}\n"
        f"TYPE {types}\n"
        f"COUNT {counts}\n"
        f"WIDTH {n}\n"
        f"HEIGHT 1\n"
        f"VIEWPOINT 0 0 0 1 0 0 0\n"
        f"POINTS {n}\n"
        f"DATA ascii\n"
    )
    with open(path, 'w') as f:
        f.write(header)
        for i in range(n):
            line = f"{xyz[i,0]} {xyz[i,1]} {xyz[i,2]}"
            if has_i:
                line += f" {intensity[i]}"
            f.write(line + "\n")


# ---------------------------------------------------------------------------
# Image helpers
# ---------------------------------------------------------------------------

def decode_image_msg(raw_data, msg_type):
    """Decode a serialized image message to numpy BGR array."""
    msg = deserialize_cdr(raw_data, msg_type)

    if msg_type == 'sensor_msgs/msg/CompressedImage':
        import cv2
        arr = np.frombuffer(bytes(msg.data), dtype=np.uint8)
        img = cv2.imdecode(arr, cv2.IMREAD_COLOR)
        return img

    # sensor_msgs/msg/Image
    import cv2
    h, w = msg.height, msg.width
    encoding = msg.encoding
    raw = bytes(msg.data)

    if encoding in ('bgr8', 'rgb8'):
        img = np.frombuffer(raw, dtype=np.uint8).reshape(h, w, 3)
        if encoding == 'rgb8':
            img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
    elif encoding == 'mono8':
        img = np.frombuffer(raw, dtype=np.uint8).reshape(h, w)
    elif encoding in ('16UC1', 'mono16'):
        img = np.frombuffer(raw, dtype=np.uint16).reshape(h, w)
    elif encoding == 'bayer_rggb8':
        bayer = np.frombuffer(raw, dtype=np.uint8).reshape(h, w)
        img = cv2.cvtColor(bayer, cv2.COLOR_BayerRG2BGR)
    else:
        img = np.frombuffer(raw, dtype=np.uint8).reshape(h, w, -1)

    return img


# ---------------------------------------------------------------------------
# IMU helpers
# ---------------------------------------------------------------------------

def decode_imu_msg(raw_data, msg_type):
    """Decode a serialized IMU message to dict."""
    msg = deserialize_cdr(raw_data, msg_type)
    stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
    return {
        'timestamp': stamp,
        'ax': msg.linear_acceleration.x,
        'ay': msg.linear_acceleration.y,
        'az': msg.linear_acceleration.z,
        'gx': msg.angular_velocity.x,
        'gy': msg.angular_velocity.y,
        'gz': msg.angular_velocity.z,
    }


# ---------------------------------------------------------------------------
# Main extraction
# ---------------------------------------------------------------------------

def scan_bag_topics(bag_path):
    """Print all topics and types in a bag."""
    with Reader(bag_path) as reader:
        print(f"\nTopics in bag: {bag_path}")
        print(f"{'Topic':<50} {'Type':<45} {'Count':>8}")
        print("-" * 105)
        for conn in reader.connections:
            count = sum(1 for c, _, _ in reader.messages() if c.topic == conn.topic)
            # count is expensive; just show type
            print(f"{conn.topic:<50} {conn.msgtype:<45}")


def extract(bag_path, output_dir, lidar_topic, camera_topics, imu_topic,
            only=None, max_images=50, max_lidar=100, lidar_accumulate=True):
    """
    Extract data from a ROS 2 bag.

    Args:
        bag_path: path to bag directory or file
        output_dir: output directory
        lidar_topic: LiDAR PointCloud2 topic
        camera_topics: dict {camera_name: topic_name}
        imu_topic: IMU topic
        only: set of data types to extract ('lidar', 'camera', 'imu'), None = all
        max_images: max images per camera
        max_lidar: max point cloud frames
        lidar_accumulate: if True, merge all frames into one PCD
    """
    do_lidar = only is None or 'lidar' in only
    do_camera = only is None or 'camera' in only
    do_imu = only is None or 'imu' in only

    # Prepare output dirs
    if do_camera:
        for cam_name in camera_topics:
            os.makedirs(os.path.join(output_dir, 'camera_images', cam_name), exist_ok=True)

    if do_lidar:
        os.makedirs(os.path.join(output_dir, 'lidar_points'), exist_ok=True)

    if do_imu:
        os.makedirs(os.path.join(output_dir, 'imu_data'), exist_ok=True)

    # Reverse map: topic -> camera_name
    topic_to_cam = {}
    if do_camera:
        for cam_name, topic in camera_topics.items():
            topic_to_cam[topic] = cam_name
            # Also match /compressed variant
            topic_to_cam[topic + '/compressed'] = cam_name

    # Counters
    cam_counts = {name: 0 for name in camera_topics}
    lidar_count = 0
    imu_rows = []
    all_lidar_xyz = []
    all_lidar_intensity = []

    print(f"\nExtracting from: {bag_path}")
    print(f"Output:          {output_dir}")
    if do_lidar:
        print(f"LiDAR topic:     {lidar_topic}")
    if do_camera:
        for name, topic in camera_topics.items():
            print(f"Camera [{name}]:  {topic}")
    if do_imu:
        print(f"IMU topic:       {imu_topic}")
    print()

    with Reader(bag_path) as reader:
        for conn, timestamp, raw_data in reader.messages():
            topic = conn.topic
            msgtype = conn.msgtype

            # --- LiDAR ---
            if do_lidar and topic == lidar_topic and msgtype == 'sensor_msgs/msg/PointCloud2':
                if lidar_count >= max_lidar and not lidar_accumulate:
                    continue
                msg = deserialize_cdr(raw_data, msgtype)
                pc = pc2_to_numpy(msg)

                if lidar_accumulate:
                    all_lidar_xyz.append(pc['xyz'])
                    inten = pc.get('intensity', pc.get('reflectivity'))
                    if inten is not None:
                        all_lidar_intensity.append(inten)
                else:
                    out_path = os.path.join(output_dir, 'lidar_points', f'{lidar_count:06d}.pcd')
                    save_pcd_ascii(out_path, pc['xyz'], pc.get('intensity', pc.get('reflectivity')))

                lidar_count += 1
                if lidar_count % 10 == 0:
                    print(f"  LiDAR frames: {lidar_count}", end='\r')
                continue

            # --- Camera ---
            if do_camera and topic in topic_to_cam:
                cam_name = topic_to_cam[topic]
                if cam_counts[cam_name] >= max_images:
                    continue
                try:
                    img = decode_image_msg(raw_data, msgtype)
                    if img is not None:
                        import cv2
                        idx = cam_counts[cam_name]
                        out_path = os.path.join(output_dir, 'camera_images', cam_name, f'{idx:06d}.jpg')
                        cv2.imwrite(out_path, img)
                        cam_counts[cam_name] += 1
                except Exception as e:
                    print(f"  WARN: decode {topic} failed: {e}")
                continue

            # --- IMU ---
            if do_imu and topic == imu_topic and msgtype == 'sensor_msgs/msg/Imu':
                try:
                    row = decode_imu_msg(raw_data, msgtype)
                    imu_rows.append(row)
                except Exception:
                    pass
                continue

    # --- Post-processing ---
    print()

    # Save accumulated LiDAR
    if do_lidar and lidar_accumulate and all_lidar_xyz:
        merged_xyz = np.concatenate(all_lidar_xyz, axis=0)
        merged_inten = np.concatenate(all_lidar_intensity, axis=0) if all_lidar_intensity else None
        out_path = os.path.join(output_dir, 'lidar_points', 'merged.pcd')
        save_pcd_ascii(out_path, merged_xyz, merged_inten)
        print(f"LiDAR: {lidar_count} frames, {len(merged_xyz)} points -> {out_path}")
    elif do_lidar:
        print(f"LiDAR: {lidar_count} frames saved individually")

    # Camera summary
    if do_camera:
        for name, count in cam_counts.items():
            print(f"Camera [{name}]: {count} images")

    # Save IMU CSV
    if do_imu and imu_rows:
        csv_path = os.path.join(output_dir, 'imu_data', 'imu.csv')
        with open(csv_path, 'w') as f:
            f.write("timestamp,accel_x,accel_y,accel_z,gyro_x,gyro_y,gyro_z\n")
            for row in imu_rows:
                f.write(f"{row['timestamp']:.9f},{row['ax']},{row['ay']},{row['az']},"
                        f"{row['gx']},{row['gy']},{row['gz']}\n")
        print(f"IMU: {len(imu_rows)} samples -> {csv_path}")

    print("\nDone.")


def main():
    parser = argparse.ArgumentParser(
        description='Extract calibration data from a ROS 2 bag',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Auto-detect topics, extract everything:
  python extract_bag_data.py /path/to/bag

  # Custom topics:
  python extract_bag_data.py /path/to/bag \\
      --lidar-topic /rslidar_points \\
      --camera-topics front=/cam0/image_raw rear=/cam1/image_raw \\
      --imu-topic /imu/data

  # Only extract images:
  python extract_bag_data.py /path/to/bag --only camera

  # Save each lidar frame separately (not merged):
  python extract_bag_data.py /path/to/bag --no-accumulate

  # List topics in bag:
  python extract_bag_data.py /path/to/bag --list-topics
""")
    parser.add_argument('bag_path', help='Path to ROS 2 bag directory')
    parser.add_argument('--output', '-o', default=None,
                        help='Output directory (default: ./extracted_data)')
    parser.add_argument('--lidar-topic', default='/rslidar_points',
                        help='LiDAR PointCloud2 topic')
    parser.add_argument('--camera-topics', nargs='*', default=None,
                        help='Camera topics as name=topic pairs (e.g. front=/cam0/image_raw)')
    parser.add_argument('--imu-topic', default='/imu/data',
                        help='IMU topic')
    parser.add_argument('--only', nargs='*', choices=['lidar', 'camera', 'imu'],
                        help='Extract only specific data types')
    parser.add_argument('--max-images', type=int, default=50,
                        help='Max images per camera')
    parser.add_argument('--max-lidar', type=int, default=100,
                        help='Max LiDAR frames (when not accumulating)')
    parser.add_argument('--no-accumulate', action='store_true',
                        help='Save each LiDAR frame separately instead of merging')
    parser.add_argument('--list-topics', action='store_true',
                        help='Only list topics in bag, do not extract')

    args = parser.parse_args()

    if not os.path.exists(args.bag_path):
        print(f"ERROR: {args.bag_path} does not exist", file=sys.stderr)
        sys.exit(1)

    if args.list_topics:
        scan_bag_topics(args.bag_path)
        sys.exit(0)

    output_dir = args.output or os.path.join(os.getcwd(), 'extracted_data')

    # Parse camera topics
    camera_topics = {}
    if args.camera_topics:
        for item in args.camera_topics:
            if '=' not in item:
                print(f"ERROR: camera topic must be name=topic, got: {item}", file=sys.stderr)
                sys.exit(1)
            name, topic = item.split('=', 1)
            camera_topics[name] = topic
    else:
        # Auto-detect image topics from bag
        with Reader(args.bag_path) as reader:
            for conn in reader.connections:
                if conn.msgtype in ('sensor_msgs/msg/Image', 'sensor_msgs/msg/CompressedImage'):
                    topic = conn.topic
                    # Derive camera name from topic
                    name = topic.strip('/').replace('/', '_').replace('image_raw', '').replace('image', '').strip('_')
                    if not name:
                        name = 'camera'
                    camera_topics[name] = topic

        if camera_topics:
            print(f"Auto-detected camera topics: {camera_topics}")

    only_set = set(args.only) if args.only else None

    extract(
        bag_path=args.bag_path,
        output_dir=output_dir,
        lidar_topic=args.lidar_topic,
        camera_topics=camera_topics,
        imu_topic=args.imu_topic,
        only=only_set,
        max_images=args.max_images,
        max_lidar=args.max_lidar,
        lidar_accumulate=not args.no_accumulate,
    )


if __name__ == '__main__':
    main()
