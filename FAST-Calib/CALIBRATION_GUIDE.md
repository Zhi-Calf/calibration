# FAST-Calib 标定操作流程指南

本文档面向实习生，详细说明如何使用 FAST-Calib 完成 LiDAR-相机外参标定。

---

## 一、概述

FAST-Calib 用于标定 **LiDAR 与相机之间的外参（旋转矩阵 R 和平移向量 t）**。

- 支持固态雷达（Livox Mid360、Avia）和机械雷达（Ouster、Hesai）
- 不需要初始外参猜测
- 标定流程约 1 秒完成

标定结果以 FAST-LIVO2 格式输出，可直接用于 FAST-LIVO2 系统。

---

## 二、环境准备

### 2.1 依赖

| 依赖 | 最低版本 |
|------|---------|
| ROS 1 (Noetic 推荐) | — |
| PCL | >= 1.8 |
| OpenCV | >= 4.0 |
| livox_ros_driver | 如使用 Livox 雷达 |

辅助工具脚本额外依赖：
- Python 3、`rosbag`、`open3d`、`numpy`

### 2.2 编译

```bash
cd ~/catkin_ws/src
git clone <本仓库地址> fast_calib
cd ~/catkin_ws
catkin_make -DCMAKE_BUILD_TYPE=Release
source devel/setup.bash
```

---

## 三、标定靶准备

### 3.1 标定靶说明

标定靶由 **4 个 ArUco 标记** 和 **4 个圆形孔洞** 组成。靶板的 CAD 模型可从 [这里](https://pan.baidu.com/s/14Q2zmEfY6Z2O5Cq4wgVljQ?pwd=2hhn) 下载后加工。

关键尺寸参数（需在配置文件中设置，务必与实物一致）：

| 参数 | 含义 | 默认值 |
|------|------|--------|
| `marker_size` | ArUco 标记边长（米） | 0.20 |
| `delta_width_qr_center` | 两个 ArUco 水平方向中心距的一半 | 0.55 |
| `delta_height_qr_center` | 两个 ArUco 垂直方向中心距的一半 | 0.35 |
| `delta_width_circles` | 两个圆孔中心的水平距离 | 0.50 |
| `delta_height_circles` | 两个圆孔中心的垂直距离 | 0.40 |
| `circle_radius` | 圆孔半径 | 0.12 |

> **注意：** 如果你使用的标定靶与默认尺寸不同，**必须** 修改 `config/qr_params.yaml` 中的对应参数，否则标定会失败。

### 3.2 标定靶放置

多场景标定需要至少 **3 个不同位置/角度** 放置标定靶：

1. **正面朝前** — 靶板正对传感器
2. **偏向右侧** — 靶板转向右方
3. **偏向左侧** — 靶板转向左方

确保标定靶在相机和 LiDAR 的共同视野 (FoV) 内，且距离适中（一般 2~5 米）。

---

## 四、数据采集

### 4.1 采集要求

每个场景需要：
1. **一个 rosbag** — 包含静态点云数据（雷达保持不动，录制几秒即可）
2. **一张对应图片** — 与 rosbag 同时刻拍摄的相机图像（`.jpg`）

### 4.2 采集步骤

```bash
# 1. 启动雷达驱动（以 Livox 为例）
roslaunch livox_ros_driver livox_lidar_msg.launch

# 2. 将标定靶放好，确保静止，然后录制 rosbag
rosbag record /livox/lidar -O scene_1.bag --duration=5

# 3. 同时用相机拍一张照片，保存为 scene_1.jpg
```

> **提示：**
> - 录制时传感器和标定靶都要保持静止
> - Topic 名称根据实际雷达修改（Ouster 用 `/ouster/points`，Hesai 用 `/hesai/pandar`）
> - 多场景标定需重复此步骤至少 3 次，每次变换靶板位置

### 4.3 数据存放

将 bag 文件和图片放入 `calib_data/` 目录。推荐按场景组织：

```
calib_data/
├── scene_1.bag
├── scene_1.jpg
├── scene_2.bag
├── scene_2.jpg
├── scene_3.bag
└── scene_3.jpg
```

---

## 五、配置文件修改

编辑 `config/qr_params.yaml`，按如下顺序检查和修改：

### 5.1 相机内参

填写你相机的标定内参（如果还没有内参，先用 OpenCV 或 MATLAB 做相机标定）：

```yaml
fx: 1215.31801774424
fy: 1214.72961288138
cx: 1047.86571859677
cy: 745.068353101898
k1: -0.33574781188503
k2: 0.10996870793601
p1: 0.000157303079833973
p2: 0.000544930726278493
```

### 5.2 标定靶参数

根据你的标定靶实物尺寸填写：

```yaml
marker_size: 0.20
delta_width_qr_center: 0.55
delta_height_qr_center: 0.35
delta_width_circles: 0.5
delta_height_circles: 0.4
circle_radius: 0.12
```

### 5.3 距离滤波参数

设置点云的裁剪范围，只保留标定靶附近的点云。可以手动估计，也可以用工具辅助：

```yaml
x_min: 2.0
x_max: 5.0
y_min: -1.0
y_max: 1.0
z_min: 0.0
z_max: 2.0
```

**使用辅助工具快速获取滤波参数：**

```bash
python scripts/distance_filter_tool.py /path/to/scene_1.bag /path/to/output_dir
```

工具会打开 Open3D 可视化窗口：
1. 按住 **Shift + 鼠标左键** 在标定靶的 4 个角上选点
2. 按 **Q** 关闭窗口
3. 工具自动计算包围盒（含 0.2m 余量）并保存为 `.txt` 文件
4. 将输出的范围值填入 `qr_params.yaml`

> **注意：** 滤波范围不需要非常精确，包含标定靶即可，多一些额外点云也没关系。

### 5.4 输入输出路径

```yaml
lidar_topic: "/livox/lidar"       # 根据你的雷达修改
bag_path: "$(find fast_calib)/calib_data/scene_1.bag"
image_path: "$(find fast_calib)/calib_data/scene_1.jpg"
output_path: "$(find fast_calib)/output"
```

> `lidar_topic` 常见值：
> - Livox: `/livox/lidar`
> - Ouster: `/ouster/points`
> - Hesai: `/hesai/pandar`

---

## 六、运行标定

### 6.1 单场景标定

```bash
roslaunch fast_calib calib.launch
```

程序会：
1. 从 rosbag 加载点云，从图片检测 ArUco 标记
2. 对点云做距离滤波 → RANSAC 平面分割 → 边缘检测 → 聚类圆拟合
3. 从图片中通过 ArUco 定位 4 个圆心在相机坐标系下的坐标
4. 用 SVD 计算 LiDAR→Camera 的刚体变换
5. 输出结果

**输出文件（在 `output/` 目录下）：**

| 文件 | 内容 |
|------|------|
| `single_calib_result.txt` | 标定结果（R, t，FAST-LIVO2 格式） |
| `colored_cloud.pcd` | 用标定外参着色的点云（可用 CloudCompare 查看） |
| `qr_detect.png` | ArUco 检测可视化图 |
| `circle_center_record.txt` | 圆心坐标记录（供多场景使用） |

### 6.2 多场景联合标定（推荐）

**先对至少 3 个不同场景分别执行单场景标定（每次修改 `bag_path`、`image_path` 和距离滤波参数）：**

```bash
# 场景 1
# 修改 qr_params.yaml 中的 bag_path、image_path、距离滤波参数
roslaunch fast_calib calib.launch

# 场景 2
# 再次修改 qr_params.yaml
roslaunch fast_calib calib.launch

# 场景 3
# 再次修改 qr_params.yaml
roslaunch fast_calib calib.launch
```

每次单场景标定完成后，圆心数据会自动追加到 `circle_center_record.txt`。

**然后运行多场景联合标定：**

```bash
roslaunch fast_calib multi_calib.launch
```

输出文件：`output/multi_calib_result.txt`（包含联合优化后的 R, t）。

> 多场景联合标定精度通常优于单场景，建议正式使用时采用此方式。

---

## 七、验证标定结果

### 7.1 视觉验证

打开 `output/colored_cloud.pcd`（用 CloudCompare 或 PCL viewer），检查点云着色是否与真实场景颜色一致。如果墙壁、地面的颜色对应正确，说明标定结果良好。

### 7.2 RViz 验证

标定时会自动启动 RViz（如果 launch 中 `rviz` 参数为 `true`），可以在 RViz 中直观查看：
- 原始点云
- 检测到的平面和圆心
- 投影效果

### 7.3 常见失败原因及排查

| 现象 | 可能原因 | 解决方法 |
|------|---------|---------|
| ArUco 检测失败 | 图片模糊/曝光不当/标记被遮挡 | 重新拍照，确保 4 个标记清晰可见 |
| 圆拟合失败 | 距离滤波范围不对，标定靶点云太少 | 调整 `x/y/z_min/max`，靶板靠近一些 |
| 几何检查失败 | 标定靶尺寸参数与实物不匹配 | 实测靶板尺寸，修正 `qr_params.yaml` |
| 点云着色偏移明显 | 单场景精度不足 | 使用多场景联合标定 |
| 找不到点云消息 | `lidar_topic` 设置错误 | 用 `rosbag info xxx.bag` 确认 topic 名 |

---

## 八、完整操作清单（快速参考）

1. [ ] 制作/获取标定靶，量好实际尺寸
2. [ ] 安装依赖，编译 FAST-Calib
3. [ ] 将标定靶放置在传感器共同视野内
4. [ ] 录制 rosbag + 拍照（场景 1）
5. [ ] 变换靶板位置，重复录制（场景 2、3）
6. [ ] 将数据放入 `calib_data/`
7. [ ] 修改 `config/qr_params.yaml`：内参、靶板尺寸、Topic
8. [ ] 运行 `distance_filter_tool.py` 获取滤波参数（可选）
9. [ ] 设置场景 1 的路径和滤波参数，运行 `roslaunch fast_calib calib.launch`
10. [ ] 设置场景 2 的路径和滤波参数，运行 `roslaunch fast_calib calib.launch`
11. [ ] 设置场景 3 的路径和滤波参数，运行 `roslaunch fast_calib calib.launch`
12. [ ] 运行 `roslaunch fast_calib multi_calib.launch` 进行联合标定
13. [ ] 检查 `output/` 下的结果，用 `colored_cloud.pcd` 验证

---

## 九、文件结构速查

```
FAST-Calib/
├── config/
│   └── qr_params.yaml          ← 主配置文件（你需要修改这个）
├── calib_data/                  ← 放标定数据（bag + 图片）
├── output/                      ← 标定输出结果
├── scripts/
│   └── distance_filter_tool.py  ← 距离滤波辅助工具
├── launch/
│   ├── calib.launch             ← 单场景标定启动文件
│   └── multi_calib.launch       ← 多场景标定启动文件
├── src/                         ← 源码（一般不需要修改）
├── include/                     ← 头文件
└── rviz_cfg/                    ← RViz 配置
```
