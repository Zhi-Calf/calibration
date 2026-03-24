# 多传感器标定系统

## 项目简介

本项目提供农业机器人多传感器融合标定工具，包括：
- **相机内参标定**（张正友棋盘格法）
- **多相机外参标定**（相机间相对位姿，四元数球面平均）
- **LiDAR-相机外参标定**（RANSAC 平面拟合 + solvePnP 链式求解）
- **IMU-相机外参标定**（静止段重力对齐，Wahba 问题 SVD 解法）

## 传感器配置

| 传感器 | 型号 / 规格 | 数量 |
|--------|------------|------|
| LiDAR  | 速腾聚创 Ruby 128 线 | 1 |
| 相机   | AR0233 2MP (1920×1080) | 6 |
| IMU    | 板载 IMU (200Hz) | 1 |

相机位置：前视、左前、右前、左后、右后、后视

## 环境要求

- Python 3.8+
- Ubuntu 20.04+ / Windows 10+

```bash
python -m venv venv
source venv/bin/activate   # Linux
# venv\Scripts\activate    # Windows

pip install -r requirements.txt
```

## 项目结构

```
calibration/
├── README.md
├── requirements.txt
├── config.yaml                    # 全局配置（含 lidar_camera 节）
├── extract_bag_data.py            # 从 ROS 2 bag 提取标定数据（见下方「从 ROS 2 Bag 提取」）
├── inspect_bag_image.py           # 查看 bag 内首帧图像编码与尺寸（调试用）
├── test_rosbag2_wsl.sh            # WSL 下测试 rosbag2 解析（list-topics + 提取）
├── camera_calibration.py          # 单相机内参标定
├── multi_camera_calibration.py    # 多相机外参标定
├── lidar_camera_calibration.py    # LiDAR-相机外参标定
├── imu_camera_calibration.py      # IMU-相机外参标定
├── calibration_utils.py          # 公共工具函数
├── complete_calibration_example.py # 完整标定流程示例
├── test_camera_calibration.py     # 单相机标定测试
├── generate_high_res_patterns.py  # 高分辨率标定板生成
├── download_calibration_patterns.py # 在线标定板下载
├── FAST-Calib/                   # LiDAR-相机精标定（ROS2 + 二维码圆孔标定板）
│   ├── launch/calib.launch.py    # 单场景启动（含 RViz2）
│   ├── config/qr_params.yaml     # bag 路径、topic、点选参数
│   ├── rviz_cfg/fast_livo2.rviz  # RViz2 配置（ROS2 插件名）
│   ├── src/main.cpp              # 主流程（含 RViz2 单击点选 ROI）
│   ├── src/lidar_detect.hpp      # LiDAR 检测（直线剔除 + 迭代 RANSAC 圆拟合）
│   ├── include/common_lib.h      # 参数、排序、几何验证
│   └── scripts/distance_filter_tool.py  # 点云距离过滤（支持 rosbag2）
└── data/                          # 数据与结果目录
    ├── camera_images/{front,front_left,...}/
    ├── lidar_points/
    ├── imu_data/
    └── calibration_results/
```

## 快速开始

### 1. 准备标定数据

- **相机**：每个相机 20–30 张不同角度的棋盘格图片，放入 `data/camera_images/<camera_name>/`
- **LiDAR-相机**：同步采集的 `.pcd`/`.ply` 点云和对应图片，按文件名排序后一一对应
- **IMU-相机**：IMU CSV 数据（timestamp, ax, ay, az, gx, gy, gz）+ 对应图片

#### 从 ROS 2 Bag 提取数据

若数据在 ROS 2 bag（`.db3` / `.mcap`）中，先用 `extract_bag_data.py` 导出为上述目录结构。依赖：`pip install rosbags`（已列入 `requirements.txt`）。

```bash
# 查看 bag 内 topic 与类型
python extract_bag_data.py /path/to/ros2_bag_dir --list-topics

# 提取到默认目录 ./extracted_data（可 -o 指定）
python extract_bag_data.py /path/to/ros2_bag_dir -o data

# 仅提取相机（或 --only lidar / imu），并限制数量
python extract_bag_data.py /path/to/ros2_bag_dir -o data --only camera --max-images 50
```

**自定义 topic 映射**（若与默认不同）：

```bash
python extract_bag_data.py /path/to/bag \\
  --lidar-topic /rslidar_points \\
  --camera-topics front=/cam0/image_raw front_left=/cam1/image_raw \\
  --imu-topic /imu/data
```

**Windows + WSL**：bag 在 Windows 盘符（如 `D:\`）时，在 WSL 中使用 `/mnt/d/` 路径，例如：

```bash
python3 extract_bag_data.py /mnt/d/rosbag2_xxx --list-topics
python3 extract_bag_data.py /mnt/d/rosbag2_xxx -o /tmp/extracted
```

脚本支持无内嵌类型定义的 ROS2 bag（自动使用 Humble 类型库）。图像编码支持：`bgr8`/`rgb8`/`mono8`/`yuv422_yuy2`/`rgba8`/`bayer_rggb8` 等，输出为标定脚本可用的图像与 PCD/IMU CSV。

### 2. 修改配置

编辑 `config.yaml`：
- 更新相机分辨率
- 设置棋盘格角点数和实际方块尺寸
- 调整 `lidar_camera` 中的 RANSAC 参数

### 3. 运行标定

```bash
# 单相机内参标定（含合成数据测试）
python test_camera_calibration.py

# 完整流程
python complete_calibration_example.py
```

## 标定原理

### LiDAR-相机标定

1. **图像侧**：检测棋盘格角点 → `solvePnP` → 得到 `T_board_cam`
2. **点云侧**：RANSAC 拟合标定板平面 → PCA 确定板面坐标轴 → 得到 `T_board_lidar`
3. **链式求解**：`T_lidar_cam = T_board_cam × inv(T_board_lidar)`
4. **多帧平均**：四元数球面平均 + 算术平均平移

### 多相机标定

同帧在参考相机和目标相机中分别检测棋盘格、`solvePnP`，通过标定板坐标系消除，
多帧结果用四元数球面平均聚合。

### IMU-相机标定

静止段估计加速度计/陀螺仪零偏（分离重力）；通过重力方向在 IMU 和相机坐标系中的
对应关系，用 SVD 求解最优旋转（Wahba 问题）。

## 标定板制作

```bash
# 生成高分辨率标定板（推荐）
python generate_high_res_patterns.py

# 或在线下载低分辨率版本
python download_calibration_patterns.py
```

打印建议：
- 600 DPI 以上打印机
- 贴在平整硬板上，避免反光材料
- 打印后实测方块尺寸并更新 `config.yaml`

## 输出

标定结果保存在 `data/calibration_results/`：
- `.npy` 文件：变换矩阵、旋转矩阵、平移向量等
- `.yaml` 文件：完整标定参数
- `*_report.txt`：可读标定报告

## FAST-Calib 使用说明

FAST-Calib 用于 LiDAR-相机精标定，基于圆孔标定板上的 4 个 ArUco 二维码和 4 个圆孔。

### 编译

需要 ROS 2 Humble、PCL、OpenCV（含 aruco 模块）。若系统默认编译器为 clang，需指定 GCC：

```bash
source /opt/ros/humble/setup.bash
CXX=/usr/bin/g++-9 CC=/usr/bin/gcc-9 colcon build --packages-select fast_calib
```

### 配置

编辑 `FAST-Calib/config/qr_params.yaml`：

| 参数 | 说明 |
|------|------|
| `bag_path` | ROS 2 bag 目录路径 |
| `lidar_topic` | 点云 topic（如 `/right/rslidar_points`） |
| `image_topic` | 图像 topic（如 `/cam3/image_raw`） |
| `circle_radius` | 标定板圆孔半径（m） |
| `delta_width_circles` | 圆心水平间距（m） |
| `delta_height_circles` | 圆心垂直间距（m） |
| `use_point_pick` | 是否启用 RViz2 点选模式 |
| `pick_padding` | 点选后 ROI 额外扩展量（m） |

### 运行

```bash
source install/setup.bash
ros2 launch fast_calib calib.launch.py
```

**点选模式**（`use_point_pick: true`）：
1. 节点加载 rosbag 点云，预过滤为 5m 半径 + 体素下采样后发布到 RViz2
2. 在 RViz2 中选择 **Publish Point** 工具，单击标定板上任意位置
3. 系统根据标定板尺寸自动计算 ROI 并继续标定流程

### 检测流程（Solid LiDAR）

1. PassThrough 过滤 → 体素下采样 → RANSAC 平面分割
2. 边界估计提取边缘点
3. **迭代 RANSAC 直线拟合**剔除标定板矩形边界
4. **迭代 RANSAC 圆拟合**（半径约束 ±0.03m）检测圆孔
5. 几何验证（圆心构成矩形，边长匹配 `delta_width/height_circles`）
6. SVD 求解 LiDAR→Camera 刚体变换

### 坐标系约定

| 传感器 | X | Y | Z |
|--------|---|---|---|
| 速腾聚创 LiDAR | 右 | 前 | 上 |
| 相机（OpenCV） | 右 | 下 | 前 |

`sortPatternCenters` 函数在排序 LiDAR 圆心时，使用上述坐标系映射确保与相机侧的排序一致。

### 输出

标定结果保存在 `output_path` 目录下：
- `single_calib_result.txt`：外参矩阵 `T_cam_lidar`（Rcl, Pcl）及相机内参
- `circle_center_record.txt`：LiDAR 和相机侧的圆心坐标
- `colored_cloud.pcd`：用标定结果投影上色的点云（相机坐标系）
- `qr_detect.png`：二维码检测结果图

## 参考

- Zhang, Z. (2000). "A flexible new technique for camera calibration"
- Olson, E. (2011). "AprilTag: A robust and flexible visual fiducial system"
- **FAST-Calib**：https://github.com/hku-mars/FAST-Calib — 本仓库内 `FAST-Calib/` 为 ROS2 移植版，支持 RViz2 点选 ROI、改进的圆孔检测算法、速腾聚创 LiDAR 坐标系适配。
- **rosbags**：Python 读写 ROS1/ROS2 bag，本流程使用其 Highlevel API（AnyReader）解析 rosbag2。
