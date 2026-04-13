# 内参标定手册（6相机，fisheye）

## 1. 目标
标定每个相机的 fisheye 内参，输出：
- `intrinsics_cam0.yaml ... intrinsics_cam5.yaml`
- `intrinsics_summary.json`

输出目录：`internal_calibration/output_cam_fisheye_2026_04_11`

---

## 2. 数据要求
- 数据根目录：`data/rosbag/indata/cam`
- 每个相机有棋盘格多姿态图像（角度、距离、位置尽量覆盖）
- 棋盘规格（你当前项目）：内角点 `8 x 12`，格子边长 `0.02m`

---

## 3. 原理（简版）
- 使用棋盘角点建立 3D-2D 约束
- 最小化重投影误差（RMS）
- fisheye 模型求解 `K + D`
  - `K`: `fx fy cx cy`
  - `D`: `k1 k2 k3 k4`

---

## 4. 运行命令

### PowerShell
```powershell
python internal_calibration/calibrate_fisheye.py `
  --data-root data/rosbag/indata/cam `
  --rows 8 `
  --cols 12 `
  --square-size 0.02 `
  --out-dir internal_calibration/output_cam_fisheye_2026_04_11
```

### WSL
```bash
python internal_calibration/calibrate_fisheye.py \
  --data-root data/rosbag/indata/cam \
  --rows 8 \
  --cols 12 \
  --square-size 0.02 \
  --out-dir internal_calibration/output_cam_fisheye_2026_04_11
```

---

## 5. 结果判读
- 关注每个相机 RMS，越小越好
- 实操经验：
  - `< 0.6`：很好
  - `0.6 ~ 1.0`：可用
  - `> 1.5`：建议复采

---

## 6. 常见问题
- 检测到角点但 RMS 很大：
  - 多补边缘姿态、倾斜姿态
  - 避免全是“正对棋盘”数据
- 个别相机桶形畸变大：
  - 优先 fisheye（你项目 cam0 已验证明显改善）

---

## 7. 给外参使用
外参配置会读取这些 fisheye 内参并写入：
- `camera_model: "fisheye"`
- `fx fy cx cy k1 k2 p1 p2`（其中 `p1,p2` 映射 fisheye 的 `k3,k4`）

