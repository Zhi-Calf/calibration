# FAST-Calib 标定数据采集操作说明

本文档说明如何在 **ROS 2 Humble** 环境下，使用 **速腾 Ruby 128 线雷达** 为 FAST-Calib 采集标定数据。

> **说明：** 本项目后续会将标定代码迁移到 ROS 2，届时可直接读取 ROS 2 bag（`.db3` / `.mcap`）。当前过渡阶段如仍需使用 ROS 1 版标定程序，请参考附录 A 将 ROS 2 bag 转换为 ROS 1 格式。

---

## 一、采集前准备

### 1.1 硬件清单

| 物品 | 说明 |
|------|------|
| LiDAR | **速腾 Ruby（RS-Ruby-128）** |
| 相机 | 已完成内参标定的 RGB 相机 |
| 标定靶 | 带 4 个 ArUco 标记 + 4 个圆形孔洞的标定板 |
| 支架/三脚架 | 固定标定靶，采集期间不能晃动 |
| 电脑 | Ubuntu 22.04 + ROS 2 Humble，已安装 `rslidar_sdk` |

### 1.2 安装速腾雷达 ROS 2 驱动

```bash
cd ~/ros2_ws/src
git clone https://github.com/RoboSense-LiDAR/rslidar_sdk.git
cd rslidar_sdk
git submodule init && git submodule update

cd ~/ros2_ws
colcon build --packages-select rslidar_sdk
source install/setup.bash
```

### 1.3 配置雷达驱动

编辑 `rslidar_sdk/config/config.yaml`：

```yaml
common:
  msg_source: 1                    # 1 = 在线雷达
  send_point_cloud_ros: true       # 发布 ROS 2 PointCloud2

lidar:
  - driver:
      lidar_type: RS128            # Ruby 128 线
      msop_port: 6699              # 数据端口，根据实际网络配置修改
      difop_port: 7788             # 设备信息端口
    ros:
      ros_frame_id: rslidar
      ros_send_point_cloud_topic: /rslidar_points
```

> **关键点：**
> - `lidar_type` 必须设为 `RS128`（对应 Ruby 128 线）
> - `msop_port` 和 `difop_port` 要与雷达实际网络配置一致
> - 速腾驱动发布的是标准 `sensor_msgs/msg/PointCloud2`，点云包含 `ring` 字段（线号），FAST-Calib 会自动识别为机械雷达模式

### 1.4 确认传感器安装

- LiDAR 和相机 **刚性固连**，采集前后不能有任何相对位移
- 两个传感器的 **视场 (FoV) 必须有重叠区域**
- 传感器安装完成后，**整个标定流程结束前不要动传感器**

### 1.5 确认相机内参

外参标定之前必须已有相机内参。如果还没有，先用棋盘格做内参标定。

需要的参数：`fx`, `fy`, `cx`, `cy`, `k1`, `k2`, `p1`, `p2`

### 1.6 确认雷达 Topic 正常

启动驱动并检查 topic：

```bash
# 终端 1：启动雷达驱动
ros2 launch rslidar_sdk start.py

# 终端 2：检查 topic
ros2 topic list
ros2 topic info /rslidar_points
ros2 topic hz /rslidar_points
```

期望输出：

```
Type: sensor_msgs/msg/PointCloud2
Publisher count: 1
```

频率应与雷达帧率一致（Ruby 通常 10 Hz）。

> 如果看不到 topic 或频率为 0，检查网络连接和端口配置。

---

## 二、标定靶放置规则

### 2.1 基本要求

- 标定靶在 **LiDAR 和相机共同可见** 的区域内
- 距离传感器 **2~5 米**（Ruby 128 线分辨率高，3~4 米效果最佳）
- **4 个 ArUco 标记和 4 个圆孔全部完整可见**，不能被遮挡
- 靶板 **竖直放置**，用支架/三脚架固定，不要手持

### 2.2 多场景放置方案（至少 3 个位置）

| 场景 | 靶板位置 | 说明 |
|------|---------|------|
| 场景 1 | 正前方 | 靶板正对传感器，居中放置 |
| 场景 2 | 偏右方 | 靶板向右移动并略微转向传感器 |
| 场景 3 | 偏左方 | 靶板向左移动并略微转向传感器 |

三个场景位置要有 **明显差异**，位置分散得越开，联合标定精度越高。

### 2.3 放置注意事项

- 靶板背后 **不要紧贴墙壁**，避免 RANSAC 把墙面也当成目标平面
- 靶板周围不要有干扰平面检测的物体
- 靶板不能有明显形变或弯曲

---

## 三、数据采集步骤

每个场景需要采集 **两样东西**：一个 rosbag（点云 + 图像）和一张独立的 `.jpg` 图片。

> **为什么既要录进 bag 又要单独保存 jpg？**
> - 把图像录进 bag 可以保留完整的原始数据，后续代码迁移到 ROS 2 后可以直接从 bag 读取图像，不再需要单独的 jpg
> - 当前标定程序用 `cv::imread()` 从文件加载图片，所以暂时还需要一张独立的 `.jpg`

### 3.1 Step 1 — 启动传感器

**终端 1 — 启动雷达：**

```bash
ros2 launch rslidar_sdk start.py
```

确认终端没有报错，且 `/rslidar_points` topic 正在发布。

**终端 2 — 启动相机（如果相机走 ROS 2）：**

```bash
# 根据你的相机驱动修改，常见示例：
ros2 launch usb_cam usb_cam.launch.py
# 或
ros2 launch realsense2_camera rs_launch.py
```

启动后确认相机 topic 正常：

```bash
ros2 topic list | grep image
ros2 topic hz /camera/image_raw
```

### 3.2 Step 2 — 放置标定靶

将标定靶放在场景 1 的位置（正前方），用支架固定好。确认：

- [ ] 靶板稳定，不会晃动
- [ ] 靶板正面朝向传感器
- [ ] 4 个 ArUco 标记清晰可见
- [ ] 4 个圆孔完整在视野内

### 3.3 Step 3 — 录制 rosbag（点云 + 图像一起录）

确认靶板和传感器都 **保持静止** 后，在新终端中同时录制点云和图像：

```bash
# 同时录制雷达点云和相机图像
ros2 bag record /rslidar_points /camera/image_raw -o scene_1
```

等待 **5~10 秒**，按 **Ctrl+C** 停止录制。

> - 将 `/camera/image_raw` 替换为你相机实际的图像 topic
> - 如果不确定 topic 名称，先用 `ros2 topic list | grep image` 查看

如果相机 **不走 ROS 2**，则只录雷达：

```bash
ros2 bag record /rslidar_points -o scene_1
```

> **重要：** 录制全程传感器和标定靶都 **不能动**！这是静态标定，任何运动都会导致失败。

录制完成后生成一个目录：

```
scene_1/
├── metadata.yaml
└── scene_1_0.db3
```

### 3.4 Step 4 — 保存一张独立的 jpg 图片

虽然图像已经录进了 bag，但当前标定程序还需要一张独立的 `.jpg` 文件。

**方式 A：从刚录的 bag 中提取（推荐，确保数据一致）**

```bash
# 终端 1：回放 bag 中的图像 topic
ros2 bag play scene_1 --topics /camera/image_raw

# 终端 2：保存一帧图像
ros2 run image_view image_saver --ros-args \
  -r image:=/camera/image_raw \
  -p filename_format:="scene_1.jpg" \
  -p save_all_image:=false
```

保存成功后按 Ctrl+C 停止两个终端。

**方式 B：录制 bag 期间直接拍照（相机走 ROS 2）**

在录制 bag 的同时，在另一个终端保存一帧：

```bash
ros2 run image_view image_saver --ros-args \
  -r image:=/camera/image_raw \
  -p filename_format:="scene_1.jpg" \
  -p save_all_image:=false
```

**方式 C：相机不走 ROS，手动拍照**

在录 bag 期间用相机拍照，保存为 `scene_1.jpg`。拍照时 **不要移动相机**。

**图片质量要求：**

- [ ] 曝光正常，不过曝也不欠曝
- [ ] 对焦清晰，ArUco 标记要清楚
- [ ] 4 个 ArUco 标记 **全部完整** 出现在画面中
- [ ] 没有运动模糊

### 3.5 Step 5 — 验证数据

录完后立即验证：

```bash
ros2 bag info scene_1
```

确认输出类似：

```
Files:             scene_1_0.db3
Bag size:          xx.x MiB
Duration:          5.xs
Messages:          150
Topic information:
  Topic: /rslidar_points   | Type: sensor_msgs/msg/PointCloud2      | Count: 50
  Topic: /camera/image_raw | Type: sensor_msgs/msg/Image            | Count: 100
```

核查：
- `/rslidar_points` topic 存在，消息数量 > 0（10 Hz 录 5 秒约 50 条）
- `/camera/image_raw` topic 存在且有消息（如果录了图像的话）
- 时长约 5~10 秒

用图片查看器打开 `scene_1.jpg`，确认 4 个 ArUco 标记完整可见且清晰。

### 3.6 Step 6 — 换位置重复

1. 将标定靶移到 **场景 2**（偏右方），重复 Step 2 ~ Step 5

```bash
ros2 bag record /rslidar_points /camera/image_raw -o scene_2
# 并保存 scene_2.jpg
```

2. 将标定靶移到 **场景 3**（偏左方），同样操作

```bash
ros2 bag record /rslidar_points /camera/image_raw -o scene_3
# 并保存 scene_3.jpg
```

---

## 四、数据整理

### 4.1 目录结构

将数据放入 `calib_data/`：

```
calib_data/
├── scene_1/                 # ROS 2 bag 目录
│   ├── metadata.yaml
│   └── scene_1_0.db3
├── scene_1.jpg              # 场景 1 图片
├── scene_2/
│   ├── metadata.yaml
│   └── scene_2_0.db3
├── scene_2.jpg
├── scene_3/
│   ├── metadata.yaml
│   └── scene_3_0.db3
└── scene_3.jpg
```

### 4.2 文件命名规范

- bag 目录和图片使用 **相同的场景编号**
- 文件名中 **不要包含中文或空格**

### 4.3 配置文件中的路径设置

采集完成后，在 `config/qr_params.yaml` 中填写路径：

```yaml
lidar_topic: "/rslidar_points"
bag_path: "$(find fast_calib)/calib_data/scene_1"      # ROS 2 bag 目录路径
image_path: "$(find fast_calib)/calib_data/scene_1.jpg"
output_path: "$(find fast_calib)/output"
```

> 代码迁移到 ROS 2 后，`bag_path` 指向 ROS 2 bag 目录即可，无需转换。

---

## 五、获取距离滤波参数

每个场景标定靶位置不同，距离滤波范围也不同。

### 5.1 用 RViz2 目测估算

```bash
# 终端 1：回放 bag
ros2 bag play scene_1

# 终端 2：打开 RViz2
rviz2
```

在 RViz2 中添加 PointCloud2 显示，topic 设为 `/rslidar_points`，观察标定靶在点云中的位置，估算其 x / y / z 范围。

### 5.2 填入配置

在 `config/qr_params.yaml` 中设置，范围宽松一些没关系，包含标定靶即可：

```yaml
x_min: 2.0
x_max: 5.0
y_min: -1.0
y_max: 1.0
z_min: -1.0
z_max: 1.5
```

> **Ruby 雷达坐标系参考：** x 前方，y 左方，z 上方。根据实际安装方向调整。

> 每个场景需要各自的滤波参数。多场景标定时每次切换场景都要更新 `qr_params.yaml`。

---

## 六、采集检查清单

**每个场景录完后逐项确认：**

- [ ] `ros2 bag info` 显示 `/rslidar_points` 且消息数量 > 0
- [ ] `ros2 bag info` 显示 `/camera/image_raw` 且消息数量 > 0（如录了图像）
- [ ] bag 目录大小合理（不是空的）
- [ ] 独立的 `.jpg` 图片文件存在且清晰
- [ ] 图片中 4 个 ArUco 标记完整、清晰
- [ ] 采集时传感器和标定靶均保持静止
- [ ] bag 和图片是同一时刻

**全部场景录完后确认：**

- [ ] 至少采集了 3 个不同位置的场景
- [ ] 3 个场景的靶板位置有明显区分
- [ ] 所有数据已放入 `calib_data/`
- [ ] 相机内参已准备好

---

## 七、常见问题

### Q: 录多长时间的 bag 合适？

5~10 秒。Ruby 128 线 10 Hz，5 秒即可获得约 50 帧点云，累加后密度足够。录太长只会增大文件体积，没有额外收益。

### Q: Ruby 在 FAST-Calib 中被当作什么类型的雷达？

**机械雷达。** 速腾 Ruby 发布的 `PointCloud2` 包含 `ring` 字段，FAST-Calib 会自动识别为机械式多线雷达（`LiDARType::Mech`），使用基于 ring 的逐线边缘检测 + 迭代 RANSAC 圆拟合算法。

### Q: 速腾驱动有 XYZIRT 和 XYZI 两种点类型，用哪个？

用 **XYZIRT**（默认）。因为 FAST-Calib 需要 `ring` 字段来判断雷达类型和做逐线处理。XYZI 没有 ring 字段，程序会把它当成固态雷达处理，算法不匹配。

### Q: 可以只录一个场景吗？

可以。单场景也能出结果，但精度不如多场景联合标定。正式使用建议至少 3 个场景。

### Q: 为什么图像录进了 bag 还要单独存一张 jpg？

当前标定程序用 `cv::imread()` 从文件路径加载图片，不是从 bag 读图像 topic。所以暂时还需要一张独立的 `.jpg`。后续代码迁移到 ROS 2 后，会改成直接从 bag 读取图像，届时一个 bag 就够了，不再需要单独的 jpg。

### Q: 图片必须和 rosbag 同时采集吗？

是的。但因为是 **静态采集**（什么都不动），只要在录 bag 期间拍照即可。推荐从 bag 中回放提取图片（Step 4 方式 A），这样数据一致性最好。关键是录制期间传感器和标定靶都 **保持不动**。

### Q: 标定靶可以放在地上吗？

不推荐。竖直放置在支架上效果更好，避免与地面平面混淆。

### Q: `ros2 bag record` 时终端没有任何输出？

正常。ROS 2 Humble 的 bag record 默认不打印每条消息。按 Ctrl+C 停止后会显示录制摘要。也可以加 `--log-level debug` 查看详细输出。

### Q: 录制时意外多录了其他 topic 怎么办？

没关系。FAST-Calib 只读取配置中 `lidar_topic` 指定的 topic，其他 topic 会被忽略。但建议只录需要的 topic 以节省磁盘空间。

---

## 八、命令速查表

| 操作 | 命令 |
|------|------|
| 启动速腾雷达 | `ros2 launch rslidar_sdk start.py` |
| 列出 topic | `ros2 topic list` |
| 检查 topic 频率 | `ros2 topic hz /rslidar_points` |
| 检查 topic 类型 | `ros2 topic info /rslidar_points` |
| 录制 bag（点云+图像） | `ros2 bag record /rslidar_points /camera/image_raw -o scene_1` |
| 录制 bag（仅点云） | `ros2 bag record /rslidar_points -o scene_1` |
| 查看 bag 信息 | `ros2 bag info scene_1` |
| 回放 bag | `ros2 bag play scene_1` |
| 回放 bag 中图像 topic | `ros2 bag play scene_1 --topics /camera/image_raw` |
| 从 bag 提取一帧图片 | `ros2 run image_view image_saver --ros-args -r image:=/camera/image_raw -p filename_format:="scene_1.jpg"` |
| 在 RViz2 中查看点云 | `rviz2`，添加 PointCloud2，topic 选 `/rslidar_points` |

---

## 附录 A：ROS 2 bag 转 ROS 1 bag（过渡期使用）

在代码完成 ROS 2 迁移之前，如需使用当前 ROS 1 版标定程序，需要将 bag 转换。

### 安装转换工具

```bash
pip install rosbags
```

### 执行转换

```bash
rosbags-convert scene_1 --dst scene_1.bag
rosbags-convert scene_2 --dst scene_2.bag
rosbags-convert scene_3 --dst scene_3.bag
```

速腾 Ruby 发布的是标准 `sensor_msgs/msg/PointCloud2`，`rosbags-convert` 可以直接转换，不存在自定义消息类型的问题。

### 验证

```bash
python3 -c "
from rosbags.rosbag1 import Reader
with Reader('scene_1.bag') as reader:
    for conn in reader.connections:
        print(f'Topic: {conn.topic}  Type: {conn.msgtype}  Count: {conn.msgcount}')
"
```

转换后将 `.bag` 文件和图片放入 `calib_data/`，配置 `qr_params.yaml` 中 `bag_path` 指向 `.bag` 文件即可。
