"""
相机内参标定模块

实现单相机的内参标定功能，使用张正友标定法。
主要功能：
- 加载标定图像
- 检测棋盘格角点
- 计算相机内参矩阵和畸变系数
- 评估标定质量
- 畸变矫正
"""

import cv2
import numpy as np
import os
from typing import List, Tuple, Optional
from tqdm import tqdm
from calibration_utils import (
    load_config,
    ensure_dir,
    save_calibration_results,
    find_checkerboard_corners,
    generate_object_points,
    draw_checkerboard_corners,
    compute_reprojection_error,
    plot_reprojection_errors,
    visualize_calibration,
    get_image_files,
    resize_image,
    rotation_matrix_to_angles
)


class CameraCalibration:
    """
    相机内参标定类
    
    使用棋盘格标定板进行相机内参标定，计算：
    - 相机内参矩阵 (camera_matrix)
    - 畸变系数 (distortion_coeffs)
    - 每张图像的外参 (rvecs, tvecs)
    """
    
    def __init__(self, config_path: str = 'config.yaml'):
        """
        初始化标定器
        
        Args:
            config_path: 配置文件路径
        """
        self.config = load_config(config_path)
        self.pattern_size = tuple(self.config['checkerboard']['inner_corners'])
        self.square_size = self.config['checkerboard']['square_size']
        
        # 标定结果
        self.camera_matrix = None
        self.dist_coeffs = None
        self.rvecs = []
        self.tvecs = []
        self.object_points = []
        self.image_points = []
        self.reprojection_error = None
        
        # 成功标定的图像
        self.valid_images = []
        self.valid_corners = []
        
    def prepare_calibration_data(self, image_dir: str) -> Tuple[List[np.ndarray], List[np.ndarray]]:
        """
        准备标定数据：检测图像中的棋盘格角点
        
        Args:
            image_dir: 图像目录路径
            
        Returns:
            (object_points, image_points): 3D世界坐标点和2D图像坐标点列表
        """
        print(f'正在准备标定数据...')
        print(f'图像目录: {image_dir}')
        print(f'棋盘格尺寸: {self.pattern_size}')
        print(f'方块尺寸: {self.square_size}m')
        
        # 获取图像文件列表
        image_files = get_image_files(image_dir)
        
        if len(image_files) == 0:
            raise ValueError(f'在目录 {image_dir} 中未找到图像文件')
        
        print(f'找到 {len(image_files)} 张图像')
        
        object_points = []  # 3D世界坐标点
        image_points = []   # 2D图像坐标点
        
        # 遍历所有图像
        for image_file in tqdm(image_files, desc='检测棋盘格角点'):
            # 读取图像
            img = cv2.imread(image_file)
            if img is None:
                print(f'警告: 无法读取图像 {image_file}')
                continue
            
            # 转换为灰度图
            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
            
            # 检测棋盘格角点
            ret, corners = find_checkerboard_corners(gray, self.pattern_size)
            
            if ret:
                # 保存成功的检测结果
                object_points.append(self._generate_object_points())
                image_points.append(corners)
                
                self.valid_images.append(img)
                self.valid_corners.append(corners)
                
                if self.config['calibration']['verbose']:
                    print(f'成功检测角点: {os.path.basename(image_file)}')
            else:
                if self.config['calibration']['verbose']:
                    print(f'未检测到角点: {os.path.basename(image_file)}')
        
        self.object_points = object_points
        self.image_points = image_points
        
        if len(object_points) < self.config['calibration']['min_images']:
            raise ValueError(
                f'成功检测的图像数量不足! 需要至少 {self.config["calibration"]["min_images"]} 张, '
                f'实际检测到 {len(object_points)} 张'
            )
        
        print(f'成功检测 {len(object_points)}/{len(image_files)} 张图像')
        return object_points, image_points
    
    def _generate_object_points(self) -> np.ndarray:
        """
        生成标定板的世界坐标系点
        
        Returns:
            3D世界坐标点数组
        """
        objp = np.zeros((self.pattern_size[0] * self.pattern_size[1], 3), np.float32)
        objp[:, :2] = np.mgrid[0:self.pattern_size[0], 0:self.pattern_size[1]].T.reshape(-1, 2)
        objp *= self.square_size
        return objp
    
    def calibrate(self, image_dir: str, max_images: Optional[int] = None) -> dict:
        """
        执行相机标定
        
        Args:
            image_dir: 图像目录路径
            max_images: 最大使用的图像数量（可选）
            
        Returns:
            标定结果字典
        """
        # 准备标定数据
        object_points, image_points = self.prepare_calibration_data(image_dir)
        
        # 限制使用的图像数量
        if max_images is not None and len(object_points) > max_images:
            object_points = object_points[:max_images]
            image_points = image_points[:max_images]
            print(f'使用前 {max_images} 张图像进行标定')
        
        # 获取图像尺寸
        img_shape = self.valid_images[0].shape[:2][::-1]  # (width, height)
        print(f'图像尺寸: {img_shape}')
        
        # 执行标定
        print('正在执行相机标定...')
        flags = cv2.CALIB_FIX_K3  # 固定k3畸变系数
        
        ret, camera_matrix, dist_coeffs, rvecs, tvecs = cv2.calibrateCamera(
            object_points,
            image_points,
            img_shape,
            None,
            None,
            flags=flags
        )
        
        if not ret:
            raise RuntimeError('相机标定失败!')
        
        # 保存结果
        self.camera_matrix = camera_matrix
        self.dist_coeffs = dist_coeffs
        self.rvecs = rvecs
        self.tvecs = tvecs
        
        # 计算重投影误差
        self.reprojection_error = compute_reprojection_error(
            object_points, image_points, rvecs, tvecs, camera_matrix, dist_coeffs
        )
        
        print(f'\n标定完成!')
        print(f'重投影误差: {self.reprojection_error:.4f} 像素')
        print(f'相机内参矩阵:\n{camera_matrix}')
        print(f'畸变系数: {dist_coeffs.ravel()}')
        
        return self.get_calibration_results()
    
    def get_calibration_results(self) -> dict:
        """
        获取标定结果
        
        Returns:
            标定结果字典
        """
        return {
            'camera_matrix': self.camera_matrix,
            'distortion_coeffs': self.dist_coeffs,
            'rvecs': self.rvecs,
            'tvecs': self.tvecs,
            'reprojection_error': self.reprojection_error
        }
    
    def save_results(self, output_dir: str):
        """
        保存标定结果到文件
        
        Args:
            output_dir: 输出目录
        """
        ensure_dir(output_dir)
        
        results = self.get_calibration_results()
        
        # 保存标定参数
        save_calibration_results(results, output_dir)
        
        # 保存详细报告
        report_path = os.path.join(output_dir, 'calibration_report.txt')
        self._save_report(report_path)
        
        print(f'标定结果已保存到 {output_dir}')
    
    def _save_report(self, report_path: str):
        """
        保存标定报告
        
        Args:
            report_path: 报告文件路径
        """
        with open(report_path, 'w') as f:
            f.write('相机内参标定报告\n')
            f.write('=' * 50 + '\n\n')
            
            f.write(f'重投影误差: {self.reprojection_error:.6f} 像素\n')
            f.write(f'有效图像数量: {len(self.object_points)}\n')
            f.write(f'棋盘格尺寸: {self.pattern_size}\n')
            f.write(f'方块尺寸: {self.square_size}m\n\n')
            
            f.write('相机内参矩阵:\n')
            f.write(str(self.camera_matrix) + '\n\n')
            
            f.write('畸变系数:\n')
            f.write(str(self.dist_coeffs.ravel()) + '\n\n')
            
            # 提取焦距和主点
            fx = self.camera_matrix[0, 0]
            fy = self.camera_matrix[1, 1]
            cx = self.camera_matrix[0, 2]
            cy = self.camera_matrix[1, 2]
            
            f.write(f'焦距 fx: {fx:.2f} 像素\n')
            f.write(f'焦距 fy: {fy:.2f} 像素\n')
            f.write(f'主点 cx: {cx:.2f} 像素\n')
            f.write(f'主点 cy: {cy:.2f} 像素\n')
    
    def visualize_corners(self, output_dir: Optional[str] = None, show: bool = True):
        """
        可视化检测到的角点
        
        Args:
            output_dir: 输出目录（如果为None则不保存）
            show: 是否显示图像
        """
        if output_dir:
            ensure_dir(output_dir)
        
        for i, (img, corners) in enumerate(zip(self.valid_images, self.valid_corners)):
            # 绘制角点
            img_with_corners = draw_checkerboard_corners(img, corners, self.pattern_size)
            
            if output_dir:
                output_path = os.path.join(output_dir, f'corners_{i:03d}.jpg')
                cv2.imwrite(output_path, img_with_corners)
            
            if show:
                cv2.imshow(f'Corners {i}', img_with_corners)
                cv2.waitKey(500)
        
        if show:
            cv2.destroyAllWindows()
    
    def visualize_reprojection(self, output_dir: Optional[str] = None, show: bool = True):
        """
        可视化重投影结果
        
        Args:
            output_dir: 输出目录（如果为None则不保存）
            show: 是否显示图像
        """
        if output_dir:
            ensure_dir(output_dir)
        
        for i in range(len(self.object_points)):
            img = self.valid_images[i]
            corners = self.valid_corners[i]
            obj_points = self.object_points[i]
            rvec = self.rvecs[i]
            tvec = self.tvecs[i]
            
            output_path = None
            if output_dir:
                output_path = os.path.join(output_dir, f'reprojection_{i:03d}.jpg')
            
            visualize_calibration(img, corners, obj_points, rvec, tvec,
                                 self.camera_matrix, self.dist_coeffs, output_path)
            
            if show:
                cv2.waitKey(500)
        
        if show:
            cv2.destroyAllWindows()
    
    def plot_error_distribution(self, output_path: Optional[str] = None):
        """
        绘制重投影误差分布图
        
        Args:
            output_path: 输出图像路径（可选）
        """
        # 计算每张图像的重投影误差
        errors = []
        for i in range(len(self.object_points)):
            # 投影3D点到2D图像平面
            projected, _ = cv2.projectPoints(
                self.object_points[i], self.rvecs[i], self.tvecs[i],
                self.camera_matrix, self.dist_coeffs
            )
            projected = projected.reshape(-1, 2)
            
            # 计算误差
            error = np.mean(np.linalg.norm(self.image_points[i] - projected, axis=1))
            errors.append(error)
        
        plot_reprojection_errors(errors, output_path)
    
    def undistort_image(self, image: np.ndarray) -> np.ndarray:
        """
        对图像进行畸变矫正
        
        Args:
            image: 输入图像
            
        Returns:
            矫正后的图像
        """
        return cv2.undistort(image, self.camera_matrix, self.dist_coeffs)
    
    def compare_distortion(self, image: np.ndarray, output_path: Optional[str] = None):
        """
        比较原始图像和矫正后的图像
        
        Args:
            image: 输入图像
            output_path: 输出图像路径（可选）
        """
        undistorted = self.undistort_image(image)
        
        # 拼接图像
        h, w = image.shape[:2]
        comparison = np.zeros((h, w * 2, 3), dtype=np.uint8)
        comparison[:, :w] = image
        comparison[:, w:] = undistorted
        
        # 添加标签
        cv2.putText(comparison, 'Original', (10, 30),
                   cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        cv2.putText(comparison, 'Undistorted', (w + 10, 30),
                   cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        
        if output_path:
            cv2.imwrite(output_path, comparison)
            print(f'保存畸变对比图到 {output_path}')
        else:
            cv2.imshow('Distortion Comparison', comparison)
            cv2.waitKey(0)
            cv2.destroyAllWindows()


def main():
    """
    主函数：演示相机标定流程
    """
    # 创建标定器
    calibrator = CameraCalibration('config.yaml')
    
    # 执行标定
    image_dir = 'data/camera_images'
    results = calibrator.calibrate(image_dir)
    
    # 保存结果
    output_dir = 'data/calibration_results'
    calibrator.save_results(output_dir)
    
    # 可视化
    if calibrator.config['calibration']['save_visualization']:
        viz_dir = os.path.join(output_dir, 'visualization')
        
        # 可视化角点
        calibrator.visualize_corners(viz_dir, show=False)
        
        # 可视化重投影
        calibrator.visualize_reprojection(viz_dir, show=False)
        
        # 绘制误差分布
        error_plot_path = os.path.join(viz_dir, 'reprojection_error.png')
        calibrator.plot_error_distribution(error_plot_path)
    
    print('\n标定完成!')


if __name__ == '__main__':
    main()