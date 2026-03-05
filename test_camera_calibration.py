"""
相机内参标定测试脚本

演示如何使用相机内参标定功能。
"""

import numpy as np
import cv2
import os
from camera_calibration import CameraCalibration
from calibration_utils import ensure_dir


def generate_synthetic_checkerboard_images(output_dir: str, num_images: int = 20):
    """
    生成合成的棋盘格图像用于测试
    
    Args:
        output_dir: 输出目录
        num_images: 生成图像的数量
    """
    ensure_dir(output_dir)
    
    print(f'生成 {num_images} 张合成棋盘格图像到 {output_dir}')
    
    # 棋盘格参数
    pattern_size = (9, 6)
    square_size = 0.02  # 20mm
    board_width = pattern_size[0] * square_size
    board_height = pattern_size[1] * square_size
    
    # 相机参数
    fx, fy = 1000, 1000
    cx, cy = 640, 360
    
    # 生成3D点
    objp = np.zeros((pattern_size[0] * pattern_size[1], 3), np.float32)
    objp[:, :2] = np.mgrid[0:pattern_size[0], 0:pattern_size[1]].T.reshape(-1, 2)
    objp *= square_size
    
    for i in range(num_images):
        # 随机旋转和平移
        angle_x = np.random.uniform(-0.5, 0.5)
        angle_y = np.random.uniform(-0.5, 0.5)
        angle_z = np.random.uniform(-0.5, 0.5)
        
        # 旋转向量
        rvec = np.array([angle_x, angle_y, angle_z], dtype=np.float32)
        
        # 平移向量
        tvec = np.array([
            np.random.uniform(-0.1, 0.1),
            np.random.uniform(-0.1, 0.1),
            np.random.uniform(0.5, 1.5)
        ], dtype=np.float32)
        
        # 投影点
        camera_matrix = np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1]], dtype=np.float32)
        dist_coeffs = np.zeros(5, dtype=np.float32)
        
        projected, _ = cv2.projectPoints(objp, rvec, tvec, camera_matrix, dist_coeffs)
        projected = projected.reshape(-1, 2)
        
        # 创建空白图像
        img = np.zeros((720, 1280, 3), dtype=np.uint8)
        
        # 绘制棋盘格
        for row in range(pattern_size[1]):
            for col in range(pattern_size[0]):
                idx = row * pattern_size[0] + col
                pt = projected[idx]
                
                # 确定点在图像范围内
                if 0 <= pt[0] < 1280 and 0 <= pt[1] < 720:
                    # 根据位置决定颜色
                    if (row + col) % 2 == 0:
                        color = (255, 255, 255)  # 白色
                    else:
                        color = (0, 0, 0)  # 黑色
                    
                    # 绘制方块
                    if idx < len(projected) - 1:
                        pt2 = projected[idx + 1]
                        if row < pattern_size[1] - 1 and col < pattern_size[0] - 1:
                            # 绘制矩形
                            pt3 = projected[idx + pattern_size[0]]
                            pt4 = projected[idx + pattern_size[0] + 1]
                            pts = np.array([pt, pt2, pt4, pt3], dtype=np.int32)
                            cv2.fillPoly(img, [pts], color)
        
        # 保存图像
        output_path = os.path.join(output_dir, f'synthetic_{i:03d}.jpg')
        cv2.imwrite(output_path, img)
        
        if (i + 1) % 5 == 0:
            print(f'已生成 {i + 1}/{num_images} 张图像')
    
    print(f'合成图像生成完成!')


def test_camera_calibration():
    """
    测试相机内参标定
    """
    print('=' * 60)
    print('相机内参标定测试')
    print('=' * 60)
    
    # 1. 生成测试数据
    test_data_dir = 'data/test_camera_images'
    generate_synthetic_checkerboard_images(test_data_dir, num_images=20)
    
    # 2. 创建标定器
    calibrator = CameraCalibration('config.yaml')
    
    # 3. 执行标定
    print('\n开始标定...')
    results = calibrator.calibrate(test_data_dir)
    
    # 4. 保存结果
    output_dir = 'data/calibration_results/test_camera'
    calibrator.save_results(output_dir)
    
    # 5. 打印结果
    print('\n' + '=' * 60)
    print('标定结果摘要')
    print('=' * 60)
    print(f'重投影误差: {calibrator.reprojection_error:.6f} 像素')
    print(f'\n相机内参矩阵:\n{calibrator.camera_matrix}')
    print(f'\n畸变系数: {calibrator.dist_coeffs.ravel()}')
    
    # 6. 可视化
    if calibrator.config['calibration']['save_visualization']:
        viz_dir = os.path.join(output_dir, 'visualization')
        print(f'\n保存可视化结果到 {viz_dir}')
        calibrator.visualize_corners(viz_dir, show=False)
        calibrator.plot_error_distribution(os.path.join(viz_dir, 'error_distribution.png'))
    
    print('\n测试完成!')


if __name__ == '__main__':
    test_camera_calibration()