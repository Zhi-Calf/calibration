# 自动驾驶多相机和LiDAR标定系统

## 项目简介

本项目提供了完整的自动驾驶多传感器融合标定解决方案，包括：
- **6个相机的内参标定**（使用棋盘格）
- **多相机外参标定**（相机之间的相对位姿）
- **LiDAR-相机外参标定**（LiDAR到相机的变换）
- **IMU-相机外参标定**（IMU到相机的变换）
- 可视化工具和测试样例

## 传感器配置

本项目针对以下传感器配置设计：
- **6个相机**：前视、左前、右前、左后、右后、后视
- **1个LiDAR**：64线或32线激光雷达
- **1个IMU**：惯性测量单元

## 环境要求

### 系统要求
- Ubuntu 18.04+ / macOS / Windows 10+
- Python 3.7+

### 依赖库安装

创建虚拟环境（推荐）：
```bash
python -m venv venv
source venv/bin/activate  # Linux/macOS
# 或
venv\Scripts\activate  # Windows
```

安装依赖：
```bash
pip install -r requirements.txt
```

### requirements.txt
```
numpy>=1.19.0
opencv-python>=4.5.0
scipy>=1.5.0
matplotlib>=3.3.0
open3d>=0.13.0
scikit-learn>=0.24.0
pyyaml>=5.4.0
tqdm>=4.60.0
```

手动安装：
```bash
pip install numpy opencv-python scipy matplotlib open3d scikit-learn pyyaml tqdm
```

## 标定板获取

### 方法1: 生成高分辨率标定板（推荐）

**推荐使用生成脚本，可以获得高分辨率的标定板图像：**

```bash
# 生成高分辨率标定板
python generate_high_res_patterns.py
```

生成的标定板保存在 `calibration_patterns/high_res/` 目录，包括：

1. **标准棋盘格 (9x6, 20mm)**
   - 文件: `checkerboard_9x6_20mm_600dpi.png`
   - 分辨率: 7014 x 4962 像素 (600 DPI)
   - 打印: A4纸张

2. **高精度棋盘格 (12x8, 30mm)**
   - 文件: `checkerboard_12x8_30mm_600dpi.png`
   - 分辨率: 9924 x 7014 像素 (600 DPI)
   - 打印: A3纸张

3. **超高精度棋盘格 (14x10, 25mm)**
   - 文件: `checkerboard_14x10_25mm_1200dpi.png`
   - 分辨率: 19848 x 14028 像素 (1200 DPI)
   - 打印: A3纸张

4. **AprilTag标定板布局 (4x4网格)**
   - 文件: `apriltag_layout_4x4_600dpi.png`
   - 分辨率: 9924 x 7014 像素 (600 DPI)
   - 标签尺寸: 160mm
   - 标签间距: 200mm

### 方法2: 在线下载（分辨率较低）

如果生成脚本不可用，可以从以下链接下载（注意：在线下载的图像分辨率较低）：

#### 1. 棋盘格标定板（相机标定）
- **标准棋盘格 (9x6, 20mm)**
  - 下载: https://opencv.org/wp-content/uploads/2020/04/camera_calibration_pattern.png
  - 打印: A4纸张
  - 注意: 分辨率较低，仅用于测试

- **高精度棋盘格 (12x8, 30mm)**
  - 下载: https://raw.githubusercontent.com/opencv/opencv/master/doc/pattern.png
  - 打印: A3纸张
  - 注意: 分辨率较低，仅用于测试

#### 2. AprilTag标定板（LiDAR-相机标定）
- **AprilTag 36h11**
  - 下载: https://github.com/AprilRobotics/apriltag-imgs/raw/master/tag36h11/tag36_11_00000.png
  - 推荐: 4x4网格，间距20cm
  - 注意: 单个标签，需要手动组合

#### 3. 下载脚本

也可以使用下载脚本获取在线标定板：

```bash
# 下载在线标定板
python download_calibration_patterns.py
```

**注意**: 在线下载的图像分辨率较低，建议使用生成脚本创建高分辨率标定板。

### 方法3: 在线生成器（可选）

#### ArUco标定板
- **ArUco生成器**
  - 在线生成: https://chev.me/arucogen/
  - 推荐: DICT_6X6_250字典
  - 可以自定义尺寸和分辨率

### 打印建议

1. **打印机要求**：
   - 使用高分辨率打印机（至少600dpi）
   - 使用厚纸或硬纸板避免变形

2. **固定方法**：
   - 将打印的标定板贴在平整的硬板上
   - 使用无反光材料
   - 确保标定板平整不弯曲

3. **棋盘格测量**：
   - 打印后精确测量实际方块尺寸
   - 将实际尺寸填入配置文件

## 项目结构

```
calibration_system/
├── README.md                    # 本文件
├── requirements.txt            # Python依赖
├── config.yaml                  # 配置文件
├── camera_calibration.py        # 相机内参标定
├── stereo_calibration.py        # 双相机外参标定
├── lidar_camera_calibration.py  # LiDAR-相机标定
├── calibration_utils.py         # 标定工具函数
├── test_camera_calibration.py   # 相机标定测试
├── test_stereo_calibration.py   # 双相机标定测试
├── test_lidar_camera.py         # LiDAR-相机标定测试
├── data/                        # 数据目录
│   ├── camera_images/           # 相机标定图像
│   ├── stereo_images/           # 双相机图像对
│   ├── lidar_points/            # LiDAR点云数据
│   └── calibration_results/     # 标定结果输出
└── calibration_patterns/        # 标定板图像
    ├── checkerboard_9x6.png     # 棋盘格
    └── apriltag_board.png       # AprilTag板
```

## 快速开始

### 1. 相机内参标定

```bash
python test_camera_calibration.py
```

### 2. 双相机外参标定

```bash
python test_stereo_calibration.py
```

### 3. LiDAR-相机外参标定

```bash
python test_lidar_camera.py
```

## 使用说明

### 准备标定数据

1. **相机标定**：
   - 准备20-30张不同角度的棋盘格图像
   - 图像应覆盖相机视场的各个区域
   - 将图像放入 `data/camera_images/` 目录

2. **双相机标定**：
   - 准备10-20对同步的图像
   - 左右相机图像需要同名（如left_001.jpg, right_001.jpg）
   - 将图像对放入 `data/stereo_images/` 目录

3. **LiDAR-相机标定**：
   - 准备同时采集的LiDAR点云和相机图像
   - 使用AprilTag标定板
   - 点云文件格式：.pcd 或 .ply
   - 图像和点云需要时间同步

### 运行标定

修改 `config.yaml` 文件中的参数：
- 相机分辨率
- 棋盘格尺寸
- 标定板实际尺寸

运行对应的测试脚本即可完成标定。

## 输出结果

标定完成后，结果保存在 `data/calibration_results/` 目录：
- `camera_matrix.npy` - 相机内参矩阵
- `dist_coeffs.npy` - 畸变系数
- `stereo_R.npy` - 双相机旋转矩阵
- `stereo_T.npy` - 双相机平移向量
- `lidar_to_camera_extrinsics.npy` - LiDAR到相机的外参

## 技术细节

### 相机标定原理
使用张正友标定法，通过多视角的棋盘格图像计算相机内参和畸变系数。

### LiDAR-相机标定原理
使用AprilTag标记作为特征点，建立LiDAR点云与相机图像之间的对应关系，求解外参矩阵。

## 常见问题

### Q: 标定精度不够怎么办？
A: 
1. 增加标定图像数量
2. 确保标定板覆盖整个视场
3. 使用更高精度的标定板
4. 精确测量标定板实际尺寸

### Q: 标定速度慢？
A:
1. 减少图像分辨率
2. 减少使用的图像数量
3. 使用更高效的硬件

### Q: LiDAR-相机标定失败？
A:
1. 检查时间同步是否正确
2. 确保AprilTag在LiDAR和相机中都能清晰识别
3. 增加AprilTag的尺寸

## 参考文献

- Zhang, Z. (2000). "A flexible new technique for camera calibration"
- Olson, E. (2011). "AprilTag: A robust and flexible visual fiducial system"
- OpenCV Camera Calibration Tutorial

## 许可证

MIT License

## 联系方式

如有问题，请提交Issue或联系项目维护者。