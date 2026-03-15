# FAST-Calib (ROS 2 Humble)

## LiDAR-Camera Extrinsic Calibration in One Second

Forked from [hku-mars/FAST-Calib](https://github.com/hku-mars/FAST-Calib) and ported to **ROS 2 Humble**.

FAST-Calib is an efficient target-based extrinsic calibration tool for LiDAR-camera systems.

**Key highlights:**

1. Supports solid-state and mechanical LiDAR (auto-detected via `ring` field).
2. No need for any initial extrinsic parameters.
3. Achieves highly accurate calibration results in just one second.
4. Reads ROS 2 bags (`.db3` / `.mcap`) directly — no format conversion needed.
5. Extracts camera images from the bag automatically (fallback to standalone `.jpg`).

**Related paper:**
[FAST-Calib: LiDAR-Camera Extrinsic Calibration in One Second](https://www.arxiv.org/pdf/2507.17210)

## 1. Prerequisites

- **ROS 2 Humble** (Ubuntu 22.04)
- PCL >= 1.10
- OpenCV >= 4.0
- Python 3 (for `distance_filter_tool.py`): `pip install rosbags open3d numpy`

## 2. Build

```bash
cd ~/ros2_ws/src
ln -s /path/to/FAST-Calib fast_calib   # or copy

cd ~/ros2_ws
colcon build --packages-select fast_calib
source install/setup.bash
```

## 3. Data Preparation

### 3.1 Record a ROS 2 bag (static scene, 5-10 seconds)

```bash
ros2 bag record /rslidar_points /camera/image_raw -o scene_1
```

Place the calibration target at 3 different positions and record one bag per scene.

### 3.2 Organize data

```
calib_data/
├── scene_1/              # ROS 2 bag directory
│   ├── metadata.yaml
│   └── scene_1_0.db3
├── scene_1.jpg           # Optional standalone image (fallback)
├── scene_2/
├── scene_3/
└── ...
```

If your bag contains an image topic, FAST-Calib reads it automatically — no standalone `.jpg` needed.

### 3.3 Configure

Edit `config/qr_params.yaml`:

```yaml
fast_calib:
  ros__parameters:
    # Camera intrinsics
    fx: 2478.487
    fy: 2478.585
    cx: 1231.0
    cy: 1025.7
    k1: -0.049
    k2: 0.147
    p1: -0.001
    p2: -0.002

    # Calibration target dimensions (must match your physical target)
    marker_size: 0.20
    delta_width_circles: 0.5
    delta_height_circles: 0.4
    circle_radius: 0.12

    # Distance filter (adjust per scene to isolate the target)
    x_min: 2.0
    x_max: 5.0
    y_min: -1.0
    y_max: 4.0
    z_min: 0.0
    z_max: 2.0

    # Input
    lidar_topic: "/rslidar_points"
    image_topic: "/camera/image_raw"
    bag_path: "/home/user/calib_data/scene_1"
    image_path: ""         # Optional fallback; leave empty to read from bag
    output_path: "/home/user/calib_data/output"
```

### 3.4 Get distance filter parameters (optional helper)

```bash
python3 scripts/distance_filter_tool.py /path/to/scene_1 /tmp/filter_output
```

Select 4 corner points on the target in the Open3D window, press Q. The tool prints `x/y/z_min/max` values to paste into `qr_params.yaml`.

## 4. Run Calibration

### 4.1 Single-scene calibration

```bash
# Edit qr_params.yaml with scene_1 paths and filter params, then:
ros2 launch fast_calib calib.launch.py

# Repeat for scene_2, scene_3 (update bag_path and distance filter each time)
```

Each run appends circle center data to `output/circle_center_record.txt`.

### 4.2 Multi-scene joint calibration

After running at least 3 single-scene calibrations:

```bash
ros2 launch fast_calib multi_calib.launch.py
```

### 4.3 Output files

| File | Content |
|------|---------|
| `output/single_calib_result.txt` | Single-scene T\_cam\_lidar (Rcl, Pcl) |
| `output/multi_calib_result.txt` | Multi-scene joint T\_cam\_lidar |
| `output/colored_cloud.pcd` | Point cloud colored with calibrated extrinsics |
| `output/qr_detect.png` | ArUco detection visualization |
| `output/circle_center_record.txt` | Intermediate circle centers for multi-scene |

## 5. Changes from Upstream

This fork applies the following changes to the [original FAST-Calib](https://github.com/hku-mars/FAST-Calib):

| Area | ROS 1 (upstream) | ROS 2 (this fork) |
|------|------|------|
| Build system | catkin | ament\_cmake |
| Node framework | `ros::init` / `ros::NodeHandle` | `rclcpp::init` / `rclcpp::Node` |
| Bag reader | `rosbag::Bag` (ROS 1 `.bag`) | `rosbag2_cpp::Reader` (ROS 2 `.db3`/`.mcap`) |
| Image input | `cv::imread()` from file only | From bag via `cv_bridge` + file fallback |
| Livox CustomMsg | Supported | Removed (use standard `PointCloud2`) |
| Launch files | XML `.launch` | Python `.launch.py` |
| Config YAML | `$(find ...)` substitution | Absolute paths, `ros__parameters` namespace |

Core algorithms (RANSAC plane fitting, edge detection, circle fitting, SVD) are unchanged.

## 6. Algorithm Workflow

See [workflow.md](workflow.md) for detailed algorithm documentation.

## 7. Acknowledgments

- Original authors: [Chunran Zheng](https://github.com/xuankuzcr) et al. at HKU-MARS
- Calibration target design based on [velo2cam_calibration](https://github.com/beltransen/velo2cam_calibration)
- [Jiaming Xu](https://github.com/Xujiaming1) and [Haotian Li](https://github.com/luo-xue) for support and equipment
