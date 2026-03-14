"""
标定工具函数模块

提供相机和LiDAR标定过程中常用的辅助函数，包括：
- 角点检测
- 坐标变换
- 数据加载和保存
- 可视化工具
"""

import cv2
import numpy as np
import yaml
import os
from pathlib import Path
from typing import List, Tuple, Optional
import matplotlib.pyplot as plt


def load_config(config_path: str = 'config.yaml') -> dict:
    """
    加载配置文件
    
    Args:
        config_path: 配置文件路径
        
    Returns:
        配置字典
    """
    with open(config_path, 'r') as f:
        config = yaml.safe_load(f)
    return config


def ensure_dir(directory: str):
    """
    确保目录存在，如果不存在则创建
    
    Args:
        directory: 目录路径
    """
    Path(directory).mkdir(parents=True, exist_ok=True)


def save_calibration_results(results: dict, output_dir: str, prefix: str = ''):
    """
    保存标定结果到文件
    
    Args:
        results: 标定结果字典
        output_dir: 输出目录
        prefix: 文件名前缀
    """
    ensure_dir(output_dir)
    
    # 保存为numpy格式
    for key, value in results.items():
        if isinstance(value, np.ndarray):
            filename = os.path.join(output_dir, f'{prefix}{key}.npy')
            np.save(filename, value)
            print(f'保存 {key} 到 {filename}')
    
    # 保存为YAML格式
    yaml_filename = os.path.join(output_dir, f'{prefix}results.yaml')

    def _to_serializable(obj):
        if isinstance(obj, np.ndarray):
            return obj.tolist()
        if isinstance(obj, dict):
            return {k: _to_serializable(v) for k, v in obj.items()}
        if isinstance(obj, (list, tuple)):
            return [_to_serializable(v) for v in obj]
        if isinstance(obj, (np.floating, np.integer)):
            return float(obj) if isinstance(obj, np.floating) else int(obj)
        return obj

    with open(yaml_filename, 'w') as f:
        yaml.dump(_to_serializable(results), f, default_flow_style=False)
    print(f'保存完整结果到 {yaml_filename}')


def load_calibration_results(input_dir: str, prefix: str = '') -> dict:
    """
    从文件加载标定结果
    
    Args:
        input_dir: 输入目录
        prefix: 文件名前缀
        
    Returns:
        标定结果字典
    """
    results = {}
    for filename in os.listdir(input_dir):
        if filename.startswith(prefix) and filename.endswith('.npy'):
            key = filename[len(prefix):-4]
            filepath = os.path.join(input_dir, filename)
            results[key] = np.load(filepath)
    return results


def find_checkerboard_corners(image: np.ndarray, 
                               pattern_size: Tuple[int, int],
                               flags: Optional[int] = None) -> Tuple[bool, Optional[np.ndarray]]:
    """
    在图像中检测棋盘格角点
    
    Args:
        image: 输入图像（灰度图）
        pattern_size: 棋盘格内角点数量 (columns, rows)
        flags: cv2.findChessboardCorners的标志位
        
    Returns:
        (success, corners): 是否成功检测到角点，角点坐标
    """
    if flags is None:
        flags = cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE
    
    # 检测角点
    ret, corners = cv2.findChessboardCorners(image, pattern_size, flags)
    
    if ret:
        # 亚像素精度优化
        criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
        corners = cv2.cornerSubPix(image, corners, (11, 11), (-1, -1), criteria)
    
    return ret, corners


def generate_object_points(pattern_size: Tuple[int, int], 
                          square_size: float,
                          num_images: int) -> List[np.ndarray]:
    """
    生成标定板的世界坐标系点
    
    Args:
        pattern_size: 棋盘格内角点数量 (columns, rows)
        square_size: 方块实际尺寸（米）
        num_images: 图像数量
        
    Returns:
        世界坐标系点列表
    """
    # 创建棋盘格的世界坐标点 (Z=0平面)
    objp = np.zeros((pattern_size[0] * pattern_size[1], 3), np.float32)
    objp[:, :2] = np.mgrid[0:pattern_size[0], 0:pattern_size[1]].T.reshape(-1, 2)
    objp *= square_size
    
    return [objp] * num_images


def draw_checkerboard_corners(image: np.ndarray, 
                             corners: np.ndarray,
                             pattern_size: Tuple[int, int]) -> np.ndarray:
    """
    在图像上绘制检测到的棋盘格角点
    
    Args:
        image: 输入图像
        corners: 角点坐标
        pattern_size: 棋盘格内角点数量
        
    Returns:
        绘制了角点的图像
    """
    img_copy = image.copy()
    cv2.drawChessboardCorners(img_copy, pattern_size, corners, True)
    return img_copy


def undistort_image(image: np.ndarray,
                   camera_matrix: np.ndarray,
                   dist_coeffs: np.ndarray) -> np.ndarray:
    """
    对图像进行畸变矫正
    
    Args:
        image: 输入图像
        camera_matrix: 相机内参矩阵
        dist_coeffs: 畸变系数
        
    Returns:
        矫正后的图像
    """
    return cv2.undistort(image, camera_matrix, dist_coeffs)


def project_points(object_points: np.ndarray,
                   rvec: np.ndarray,
                   tvec: np.ndarray,
                   camera_matrix: np.ndarray,
                   dist_coeffs: np.ndarray) -> np.ndarray:
    """
    将3D点投影到2D图像平面
    
    Args:
        object_points: 3D世界坐标点
        rvec: 旋转向量
        tvec: 平移向量
        camera_matrix: 相机内参矩阵
        dist_coeffs: 畸变系数
        
    Returns:
        2D图像坐标点
    """
    projected_points, _ = cv2.projectPoints(object_points, rvec, tvec, 
                                           camera_matrix, dist_coeffs)
    return projected_points.reshape(-1, 2)


def compute_reprojection_error(object_points: List[np.ndarray],
                               image_points: List[np.ndarray],
                               rvecs: List[np.ndarray],
                               tvecs: List[np.ndarray],
                               camera_matrix: np.ndarray,
                               dist_coeffs: np.ndarray) -> float:
    """
    计算重投影误差
    
    Args:
        object_points: 3D世界坐标点列表
        image_points: 2D图像坐标点列表
        rvecs: 旋转向量列表
        tvecs: 平移向量列表
        camera_matrix: 相机内参矩阵
        dist_coeffs: 畸变系数
        
    Returns:
        平均重投影误差（像素）
    """
    total_sq_error = 0.0
    total_points = 0
    
    for i in range(len(object_points)):
        projected_points = project_points(object_points[i], rvecs[i], tvecs[i],
                                        camera_matrix, dist_coeffs)
        
        img_pts = image_points[i].reshape(-1, 2)
        diff = img_pts - projected_points
        sq_errors = np.sum(diff ** 2, axis=1)
        total_sq_error += np.sum(sq_errors)
        total_points += len(sq_errors)
    
    if total_points == 0:
        return float('inf')
    return np.sqrt(total_sq_error / total_points)


def stereo_rectify(camera_matrix1: np.ndarray,
                   dist_coeffs1: np.ndarray,
                   camera_matrix2: np.ndarray,
                   dist_coeffs2: np.ndarray,
                   image_size: Tuple[int, int],
                   R: np.ndarray,
                   T: np.ndarray) -> dict:
    """
    立体相机校正
    
    Args:
        camera_matrix1: 左相机内参矩阵
        dist_coeffs1: 左相机畸变系数
        camera_matrix2: 右相机内参矩阵
        dist_coeffs2: 右相机畸变系数
        image_size: 图像尺寸 (width, height)
        R: 双相机旋转矩阵
        T: 双相机平移向量
        
    Returns:
        校正参数字典
    """
    R1, R2, P1, P2, Q, roi1, roi2 = cv2.stereoRectify(
        camera_matrix1, dist_coeffs1,
        camera_matrix2, dist_coeffs2,
        image_size, R, T,
        flags=cv2.CALIB_ZERO_DISPARITY,
        alpha=0
    )
    
    return {
        'R1': R1,
        'R2': R2,
        'P1': P1,
        'P2': P2,
        'Q': Q,
        'roi1': roi1,
        'roi2': roi2
    }


def plot_reprojection_errors(errors: List[float], 
                            output_path: Optional[str] = None):
    """
    绘制重投影误差分布图
    
    Args:
        errors: 重投影误差列表
        output_path: 输出图像路径（可选）
    """
    plt.figure(figsize=(10, 6))
    plt.bar(range(len(errors)), errors)
    plt.xlabel('Image Index')
    plt.ylabel('Reprojection Error (pixels)')
    plt.title('Reprojection Error Distribution')
    plt.axhline(y=np.mean(errors), color='r', linestyle='--', 
                label=f'Mean: {np.mean(errors):.4f}')
    plt.legend()
    plt.grid(True, alpha=0.3)
    
    if output_path:
        plt.savefig(output_path, dpi=300, bbox_inches='tight')
        print(f'保存重投影误差图到 {output_path}')
    else:
        plt.show()
    
    plt.close()


def visualize_calibration(image: np.ndarray,
                         corners: np.ndarray,
                         object_points: np.ndarray,
                         rvec: np.ndarray,
                         tvec: np.ndarray,
                         camera_matrix: np.ndarray,
                         dist_coeffs: np.ndarray,
                         output_path: Optional[str] = None):
    """
    可视化标定结果
    
    Args:
        image: 输入图像
        corners: 检测到的角点
        object_points: 3D世界坐标点
        rvec: 旋转向量
        tvec: 平移向量
        camera_matrix: 相机内参矩阵
        dist_coeffs: 畸变系数
        output_path: 输出图像路径（可选）
    """
    # 投影3D点
    projected_points = project_points(object_points, rvec, tvec,
                                      camera_matrix, dist_coeffs)
    
    # 绘制
    img_copy = image.copy()
    for i in range(len(projected_points)):
        # 绘制实际角点（绿色）
        cv2.circle(img_copy, tuple(corners[i].astype(int)), 5, (0, 255, 0), -1)
        # 绘制投影点（红色）
        cv2.circle(img_copy, tuple(projected_points[i].astype(int)), 5, (0, 0, 255), -1)
        # 绘制连接线
        cv2.line(img_copy, tuple(corners[i].astype(int)), 
                tuple(projected_points[i].astype(int)), (255, 255, 0), 2)
    
    # 计算误差
    error = np.mean(np.linalg.norm(corners - projected_points, axis=1))
    
    # 添加文本
    cv2.putText(img_copy, f'Mean Error: {error:.4f} px', (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
    cv2.putText(img_copy, 'Green: Detected, Red: Reprojected', (10, 70),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)
    
    if output_path:
        cv2.imwrite(output_path, img_copy)
        print(f'保存可视化结果到 {output_path}')
    else:
        cv2.imshow('Calibration Visualization', img_copy)
        cv2.waitKey(0)
        cv2.destroyAllWindows()


def get_image_files(directory: str, 
                   extensions: Tuple[str, ...] = ('.jpg', '.jpeg', '.png', '.bmp')) -> List[str]:
    """
    获取目录中的所有图像文件
    
    Args:
        directory: 图像目录
        extensions: 图像扩展名
        
    Returns:
        图像文件路径列表
    """
    image_files = []
    for filename in os.listdir(directory):
        if filename.lower().endswith(extensions):
            image_files.append(os.path.join(directory, filename))
    return sorted(image_files)


def resize_image(image: np.ndarray, 
                max_width: int = 1280,
                max_height: int = 720) -> np.ndarray:
    """
    调整图像大小
    
    Args:
        image: 输入图像
        max_width: 最大宽度
        max_height: 最大高度
        
    Returns:
        调整大小后的图像
    """
    h, w = image.shape[:2]
    
    if w > max_width or h > max_height:
        scale = min(max_width / w, max_height / h)
        new_w = int(w * scale)
        new_h = int(h * scale)
        image = cv2.resize(image, (new_w, new_h), interpolation=cv2.INTER_AREA)
    
    return image


def compute_rotation_matrix(rvec: np.ndarray) -> np.ndarray:
    """
    从旋转向量计算旋转矩阵
    
    Args:
        rvec: 旋转向量 (3x1)
        
    Returns:
        旋转矩阵 (3x3)
    """
    return cv2.Rodrigues(rvec)[0]


def rotation_matrix_to_angles(R: np.ndarray) -> Tuple[float, float, float]:
    """
    从旋转矩阵提取欧拉角 (roll, pitch, yaw)
    
    Args:
        R: 旋转矩阵 (3x3)
        
    Returns:
        (roll, pitch, yaw) 弧度
    """
    sy = np.sqrt(R[0, 0] * R[0, 0] + R[1, 0] * R[1, 0])
    
    singular = sy < 1e-6
    
    if not singular:
        roll = np.arctan2(R[2, 1], R[2, 2])
        pitch = np.arctan2(-R[2, 0], sy)
        yaw = np.arctan2(R[1, 0], R[0, 0])
    else:
        roll = np.arctan2(-R[1, 2], R[1, 1])
        pitch = np.arctan2(-R[2, 0], sy)
        yaw = 0
    
    return roll, pitch, yaw