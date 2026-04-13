# 外参标定手册（6相机，cam->lidar，fisheye）

## 1. 目标
对 6 个相机分别标定 `T_cam_lidar` 外参（旋转 R + 平移 t）。

当前数据路径：`data/rosbag/outdata`

---

## 2. 数据结构（当前约定）
- `0000` -> cam0
- `1111` -> cam1
- `2222` -> cam2
- `3333` -> cam3
- `4444` -> cam4
- `5555` -> cam5

每个目录下：
- `3m-front/rosbag2_...`
- 话题包含：
  - `/camX/image_raw`
  - `/left/rslidar_points`（当前使用）

---

## 3. 原理（简版）
1. 图像侧得到板上 4 个目标点（相机坐标）
2. 点云侧在 RViz 点击 4 个对应点（雷达坐标）
3. 用 SVD 解刚体变换，得到 `T_cam_lidar`
4. 计算 RMSE，并通过门限筛选

---

## 4. 当前实现特性（你项目已改）
- 支持 `camera_model: "fisheye"`
- 若一帧 tag 不足 4 个，自动尝试 bag 下一帧
- 只有检测到 `>=4` tag 才进入图像点选流程
- 支持手动图像 4 点 + 手动点云 4 点

---

## 5. 配置文件
使用：
- `data/qr_params_cam0_outdata.yaml`
- ...
- `data/qr_params_cam5_outdata.yaml`

关键参数：
- `camera_model: "fisheye"`
- `target_mode: "apriltag_grid"`
- `use_image_pick: true`
- `use_point_pick: true`
- `pick_num_points: 4`
- `lidar_topic: "/left/rslidar_points"`
- `image_topic: "/camX/image_raw"`
- `bag_path: ".../outdata/XXXX/3m-front/rosbag2_..."`

---

## 6. 编译
```bash
cd FAST-Calib
source /opt/ros/humble/setup.bash
colcon build --packages-select fast_calib
source install/setup.bash
```

---

## 7. 运行命令

### 单相机（例：cam3）
```bash
ros2 launch fast_calib calib.launch.py \
  rviz:=false \
  params_file:=data/qr_params_cam3_outdata.yaml
```

### 六相机批量
```bash
for i in 0 1 2 3 4 5; do
  echo "========== calibrating cam$i =========="
  ros2 launch fast_calib calib.launch.py \
    rviz:=false \
    params_file:=data/qr_params_cam${i}_outdata.yaml
done
```

建议另开一个终端启动 RViz：
```bash
rviz2 -d FAST-Calib/rviz_cfg/fast_livo2.rviz
```

---

## 8. 每轮人工操作
1. 等待程序读取 bag 并检测到至少 4 个 tag
2. 图像窗口按顺序点 4 个 tag
3. RViz 按同顺序点 4 个点云点
4. 查看输出 `RMSE` 与 `T_cam_lidar`
5. 确认 `Round accepted`

---

## 9. 输出文件
每个相机目录（例 cam3）：
- `data/output/2026_04_10_outdata/cam3_external/single_calib_result.txt`
- `data/output/2026_04_10_outdata/cam3_external/colored_cloud.pcd`
- `data/output/2026_04_10_outdata/cam3_external/circle_center_record.txt`

---

## 10. 常见问题
- 只识别到 2 个 tag：
  - 已支持自动跳后续帧
  - 检查光照、遮挡、字典配置
- `camera_model` 仍显示 pinhole：
  - 重新编译并 `source install/setup.bash`
  - 确认 launch 使用了 `params_file`
- 循环跑不进下一个相机：
  - 不要在循环里 `rviz:=true`，否则会等待 rviz 退出

