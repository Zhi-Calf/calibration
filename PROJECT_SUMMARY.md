# 自动驾驶多传感器标定系统 - 项目总结

## 项目概述

本项目为自动驾驶系统提供了完整的多传感器标定解决方案，支持6个相机、1个LiDAR和1个IMU的标定。

## 文件结构

```
calibration_system/
├── README.md                          # 项目说明和使用指南
├── requirements.txt                    # Python依赖库
├── config.yaml                        # 配置文件
├── PROJECT_SUMMARY.md                 # 本文件
├── calibration_utils.py               # 标定工具函数
├── camera_calibration.py              # 相机内参标定
├── multi_camera_calibration.py        # 多相机外参标定
├── lidar_camera_calibration.py       # LiDAR-相机标定
├── imu_camera_calibration.py          # IMU-相机标定
├── test_camera_calibration.py         # 相机标定测试
└── complete_calibration_example.py    # 完整标定流程示例
```

## 核心模块说明

### 1. calibration_utils.py
**功能**: 标定工具函数库
**主要函数**:
- `load_config()` - 加载配置文件
- `save_calibration_results()` - 保存标定结果
- `find_checkerboard_corners()` - 检测棋盘格角点
- `compute_reprojection_error()` - 计算重投影误差
- `plot_reprojection_errors()` - 绘制误差分布图
- `visualize_calibration()` - 可视化标定结果

### 2. camera_calibration.py
**功能**: 单相机内参标定
**主要类**: `CameraCalibration`
**主要方法**:
- `calibrate()` - 执行相机标定
- `undistort_image()` - 畸变矫正
- `visualize_corners()` - 可视化检测的角点
- `visualize_reprojection()` - 可视化重投影结果

**输出**: 相机内参矩阵、畸变系数、重投影误差

### 3. multi_camera_calibration.py
**功能**: 6个相机的外参标定
**主要类**: `MultiCameraCalibration`
**主要方法**:
- `calibrate_all_cameras()` - 标定所有相机内参
- `calculate_extrinsics()` - 计算相机之间的外参
- `global_optimization()` - 全局优化
- `transform_point()` - 点坐标转换

**输出**: 
- 每个相机的内参
- 相对于参考相机的外参（R, T）

### 4. lidar_camera_calibration.py
**功能**: LiDAR-相机外参标定
**主要类**: `LidarCameraCalibration`
**主要方法**:
- `detect_apriltag_in_image()` - 检测AprilTag
- `load_point_cloud()` - 加载LiDAR点云
- `preprocess_point_cloud()` - 点云预处理
- `calibrate()` - 执行LiDAR-相机标定
- `transform_point_cloud()` - 点云坐标转换
- `project_lidar_to_image()` - LiDAR点投影到图像

**输出**: LiDAR到相机的4x4变换矩阵

### 5. imu_camera_calibration.py
**功能**: IMU-相机外参标定
**主要类**: `IMUCameraCalibration`
**主要方法**:
- `load_imu_data()` - 加载IMU数据
- `detect_static_periods()` - 检测静止时间段
- `estimate_imu_bias()` - 估计IMU零偏
- `compute_gravity_direction_from_imu()` - 从IMU计算重力方向
- `calibrate()` - 执行IMU-相机标定

**输出**: 
- IMU到相机的变换矩阵
- IMU零偏（加速度计和陀螺仪）

### 6. test_camera_calibration.py
**功能**: 相机标定测试样例
**主要功能**:
- 生成合成棋盘格图像
- 测试相机标定流程
- 验证标定结果

### 7. complete_calibration_example.py
**功能**: 完整标定流程示例
**主要功能**:
- 步骤1: 标定所有相机内参
- 步骤2: 标定LiDAR到相机外参
- 步骤3: 标定IMU到相机外参
- 生成标定结果摘要

## 传感器配置

### 6个相机
1. **front** - 前视相机
2. **front_left** - 左前相机
3. **front_right** - 右前相机
4. **rear_left** - 左后相机
5. **rear_right** - 右后相机
6. **rear** - 后视相机

### LiDAR
- 类型: Velodyne 64线或32线
- 最大探测距离: 120米
- 最小探测距离: 0.5米

### IMU
- 类型: 高频惯性测量单元
- 采样频率: 200Hz

## 标定板下载链接

### 1. 棋盘格标定板（相机标定）
- **标准棋盘格 (9x6, 20mm)**
  - 下载: https://opencv.org/wp-content/uploads/2020/04/camera_calibration_pattern.png
  - 打印: A4纸张

- **高精度棋盘格 (12x8, 30mm)**
  - 下载: https://raw.githubusercontent.com/opencv/opencv/master/doc/pattern.png
  - 打印: A3纸张

### 2. AprilTag标定板（LiDAR-相机标定）
- **AprilTag 36h11**
  - 下载: https://github.com/AprilRobotics/apriltag-imgs/raw/master/tag36h11/tag36_11_00000.png
  - 推荐: 4x4网格，间距20cm

### 3. ArUco标定板（可选）
- **ArUco生成器**
  - 在线生成: https://chev.me/arucogen/
  - 推荐: DICT_6X6_250字典

## 使用流程

### 快速开始

1. **安装依赖**
```bash
pip install -r requirements.txt
```

2. **准备数据**
```bash
# 创建数据目录结构
mkdir -p data/camera_images/{front,front_left,front_right,rear_left,rear_right,rear}
mkdir -p data/lidar_points
mkdir -p data/imu_data
mkdir -p data/calibration_results
```

3. **运行完整标定**
```bash
python complete_calibration_example.py
```

### 分步标定

#### 步骤1: 相机内参标定
```bash
# 为每个相机采集20-30张棋盘格图像
python test_camera_calibration.py  # 测试用
# 或
python camera_calibration.py       # 实际使用
```

#### 步骤2: 多相机外参标定
```bash
python multi_camera_calibration.py
```

#### 步骤3: LiDAR-相机标定
```bash
python lidar_camera_calibration.py
```

#### 步骤4: IMU-相机标定
```bash
python imu_camera_calibration.py
```

## 输出结果

### 相机内参
- `camera_matrix.npy` - 相机内参矩阵 (3x3)
- `distortion_coeffs.npy` - 畸变系数 (5x1)
- `reprojection_error` - 重投影误差

### 相机外参
- `R.npy` - 旋转矩阵 (3x3)
- `T.npy` - 平移向量 (3x1)
- 相对于参考相机的外参

### LiDAR-相机外参
- `transformation_matrix.npy` - 4x4变换矩阵
- `R.npy` - 旋转矩阵
- `T.npy` - 平移向量

### IMU-相机外参
- `transformation_matrix.npy` - 4x4变换矩阵
- `accel_bias.npy` - 加速度计零偏
- `gyro_bias.npy` - 陀螺仪零偏

## 标定精度评估

### 重投影误差
- **优秀**: < 0.5 像素
- **良好**: 0.5 - 1.0 像素
- **可接受**: 1.0 - 2.0 像素
- **需要重新标定**: > 2.0 像素

### 提高精度的方法
1. 增加标定图像数量
2. 确保标定板覆盖整个视场
3. 使用更高精度的标定板
4. 精确测量标定板实际尺寸
5. 改善光照条件
6. 避免标定板反光

## 常见问题

### Q1: 标定失败怎么办？
**A**: 
1. 检查图像质量和数量
2. 确认标定板尺寸配置正确
3. 验证数据文件格式
4. 查看错误日志

### Q2: 标定精度不够？
**A**:
1. 增加标定图像数量（推荐30+张）
2. 改善光照条件
3. 使用更高分辨率的标定板
4. 确保标定板平整

### Q3: LiDAR-相机标定失败？
**A**:
1. 检查时间同步
2. 确认AprilTag在LiDAR中可见
3. 增加AprilTag尺寸
4. 预处理点云（降采样、滤波）

### Q4: IMU-相机标定误差大？
**A**:
1. 确保IMU数据时间同步
2. 增加静止时间段
3. 在多个方向采集数据
4. 检查IMU零偏估计

## 技术原理

### 相机标定
- **方法**: 张正友标定法
- **原理**: 通过多视角棋盘格图像计算内参和畸变系数
- **优化**: 最小化重投影误差

### LiDAR-相机标定
- **方法**: AprilTag特征点匹配
- **原理**: 建立LiDAR点云与相机图像的对应关系
- **求解**: PnP算法求解外参

### IMU-相机标定
- **方法**: 重力方向对齐
- **原理**: 通过静止时间段估计零偏，使用重力方向计算旋转
- **算法**: Kabsch算法

## 扩展功能

### 1. 在线标定
- 实时更新标定参数
- 适应温度变化和机械振动

### 2. 自动化标定
- 机器人自动移动标定板
- 自动采集和处理数据

### 3. 联合优化
- 同时优化所有传感器外参
- 使用Bundle Adjustment

### 4. 精度评估
- 交叉验证
- 不确定性分析
- 置信区间估计

## 参考文献

1. Zhang, Z. (2000). "A flexible new technique for camera calibration"
2. Olson, E. (2011). "AprilTag: A robust and flexible visual fiducial system"
3. OpenCV Camera Calibration Tutorial
4. Kabsch, W. (1976). "A solution for the best rotation to relate two sets of vectors"

## 许可证

MIT License

## 联系方式

如有问题或建议，请提交Issue或Pull Request。

## 更新日志

### Version 1.0.0 (2024-03-05)
- 初始版本发布
- 支持6个相机、LiDAR、IMU标定
- 完整的文档和示例
- 测试样例

---

**祝您标定顺利！**