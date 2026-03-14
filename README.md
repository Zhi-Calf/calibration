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
├── camera_calibration.py          # 单相机内参标定
├── multi_camera_calibration.py    # 多相机外参标定
├── lidar_camera_calibration.py    # LiDAR-相机外参标定
├── imu_camera_calibration.py      # IMU-相机外参标定
├── calibration_utils.py           # 公共工具函数
├── complete_calibration_example.py # 完整标定流程示例
├── test_camera_calibration.py     # 单相机标定测试
├── generate_high_res_patterns.py  # 高分辨率标定板生成
├── download_calibration_patterns.py # 在线标定板下载
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

## 参考

- Zhang, Z. (2000). "A flexible new technique for camera calibration"
- Olson, E. (2011). "AprilTag: A robust and flexible visual fiducial system"
- FAST-Calib: https://github.com/hku-mars/FAST-Calib （推荐用于 LiDAR-Camera 精标定）
