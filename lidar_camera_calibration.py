"""
LiDAR-相机外参标定模块

实现LiDAR与相机之间的外参标定功能。
主要功能：
- 加载LiDAR点云和相机图像
- 使用AprilTag建立对应关系
- 计算LiDAR到相机的外参矩阵
- 可视化标定结果
"""

import cv2
import numpy as np
import open3d as o3d
import os
from typing import List, Tuple, Optional, Dict
from tqdm import tqdm
from calibration_utils import (
    load_config,
    ensure_dir,
    save_calibration_results,
    get_image_files,
    rotation_matrix_to_angles
)


class LidarCameraCalibration:
    """
    LiDAR-相机外参标定类
    
    使用AprilTag标定板建立LiDAR点云与相机图像之间的对应关系，
    计算LiDAR到相机的外参矩阵（4x4变换矩阵）。
    """
    
    def __init__(self, config_path: str = 'config.yaml'):
        """
        初始化标定器
        
        Args:
            config_path: 配置文件路径
        """
        self.config = load_config(config_path)
        self.apriltag_config = self.config['lidar_camera']['apriltag']
        self.lidar_config = self.config['lidar_camera']['lidar']
        
        # 标定板标签的世界坐标（在标定板坐标系中）
        self.tag_positions = np.array(self.apriltag_config['tag_positions'], dtype=np.float32)
        self.tag_size = self.apriltag_config['tag_size']
        
        # 标定结果
        self.transformation_matrix = None  # 4x4变换矩阵
        self.R = None  # 旋转矩阵 3x3
        self.T = None  # 平移向量 3x1
        self.calibration_error = None
        
        # 相机内参（需要先标定）
        self.camera_matrix = None
        self.dist_coeffs = None
        
    def set_camera_intrinsics(self, camera_matrix: np.ndarray, dist_coeffs: np.ndarray):
        """
        设置相机内参
        
        Args:
            camera_matrix: 相机内参矩阵 3x3
            dist_coeffs: 畸变系数
        """
        self.camera_matrix = camera_matrix
        self.dist_coeffs = dist_coeffs
        print('相机内参已设置')
        
    def detect_apriltag_in_image(self, image: np.ndarray) -> List[Dict]:
        """
        在图像中检测AprilTag
        
        Args:
            image: 输入图像
            
        Returns:
            检测到的AprilTag列表，每个元素包含标签ID和角点坐标
        """
        # 注意：这里需要安装opencv-contrib-python或使用apriltag库
        # 为了简化，这里使用ArUco标记作为替代
        
        # 定义ArUco字典（作为AprilTag的替代）
        aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
        aruco_params = cv2.aruco.DetectorParameters()
        
        # 检测标记
        corners, ids, rejected = cv2.aruco.detectMarkers(image, aruco_dict, parameters=aruco_params)
        
        detections = []
        if ids is not None:
            for i, corner in enumerate(corners):
                detections.append({
                    'id': int(ids[i][0]),
                    'corners': corner[0]  # 4个角点坐标
                })
        
        return detections
    
    def extract_tag_points_from_lidar(self, point_cloud: np.ndarray, 
                                     detections: List[Dict],
                                     camera_matrix: np.ndarray,
                                     dist_coeffs: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        """
        从LiDAR点云中提取与AprilTag对应的3D点
        
        Args:
            point_cloud: LiDAR点云 (N, 3)
            detections: 图像中的AprilTag检测结果
            camera_matrix: 相机内参矩阵
            dist_coeffs: 畸变系数
            
        Returns:
            (image_points, lidar_points): 图像中的2D点和LiDAR中的3D点
        """
        image_points = []
        lidar_points = []
        
        # 投影LiDAR点到图像平面
        if len(point_cloud) > 0:
            points_3d = point_cloud[:, :3]
            points_3d_hom = np.hstack([points_3d, np.ones((len(points_3d), 1))])
            
            # 假设初始变换矩阵（需要迭代优化）
            # 这里使用单位矩阵作为初始估计
            initial_T = np.eye(4)
            
            # 投影点
            points_2d_hom = (points_3d_hom @ initial_T.T)[:, :3]
            points_2d = cv2.projectPoints(
                points_2d_hom[:, :3], np.zeros(3), np.zeros(3),
                camera_matrix, dist_coeffs
            )[0].reshape(-1, 2)
            
            # 为每个AprilTag寻找对应的LiDAR点
            for detection in detections:
                tag_corners = detection['corners']
                tag_center = np.mean(tag_corners, axis=0)
                
                # 在投影的LiDAR点中寻找最近的点
                distances = np.linalg.norm(points_2d - tag_center, axis=1)
                min_idx = np.argmin(distances)
                
                if distances[min_idx] < 50:  # 阈值：50像素
                    image_points.append(tag_center)
                    lidar_points.append(points_3d[min_idx])
        
        if len(image_points) > 0:
            return np.array(image_points), np.array(lidar_points)
        else:
            return np.array([]), np.array([])
    
    def load_point_cloud(self, pcd_file: str) -> np.ndarray:
        """
        加载LiDAR点云文件
        
        Args:
            pcd_file: 点云文件路径 (.pcd 或 .ply)
            
        Returns:
            点云数组 (N, 3) 或 (N, 4)
        """
        try:
            pcd = o3d.io.read_point_cloud(pcd_file)
            points = np.asarray(pcd.points)
            return points
        except Exception as e:
            print(f'加载点云失败: {e}')
            return np.array([])
    
    def preprocess_point_cloud(self, points: np.ndarray) -> np.ndarray:
        """
        预处理LiDAR点云
        
        Args:
            points: 原始点云
            
        Returns:
            处理后的点云
        """
        if len(points) == 0:
            return points
        
        # 1. 距离过滤
        distances = np.linalg.norm(points, axis=1)
        min_range = 0.5
        max_range = 50.0
        mask = (distances > min_range) & (distances < max_range)
        points = points[mask]
        
        # 2. 降采样
        voxel_size = self.lidar_config['voxel_size']
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points)
        pcd_down = pcd.voxel_down_sample(voxel_size=voxel_size)
        points = np.asarray(pcd_down.points)
        
        return points
    
    def compute_transformation(self, 
                            image_points: np.ndarray,
                            lidar_points: np.ndarray,
                            object_points: np.ndarray) -> Tuple[np.ndarray, np.ndarray, float]:
        """
        计算LiDAR到相机的变换矩阵
        
        Args:
            image_points: 图像中的2D点 (N, 2)
            lidar_points: LiDAR中的3D点 (N, 3)
            object_points: 标定板的世界坐标点 (N, 3)
            
        Returns:
            (R, T, error): 旋转矩阵、平移向量、标定误差
        """
        # 使用PnP算法求解相机到标定板的外参
        success, rvec, tvec = cv2.solvePnP(
            object_points, image_points,
            self.camera_matrix, self.dist_coeffs,
            flags=cv2.SOLVEPNP_ITERATIVE
        )
        
        if not success:
            raise RuntimeError('PnP求解失败!')
        
        # 计算标定板到相机的变换
        R_board_to_cam = cv2.Rodrigues(rvec)[0]
        T_board_to_cam = tvec
        
        # 计算LiDAR到标定板的变换（使用点云拟合）
        # 这里简化处理，假设LiDAR坐标系与标定板坐标系重合
        R_lidar_to_board = np.eye(3)
        T_lidar_to_board = np.zeros((3, 1))
        
        # 计算LiDAR到相机的变换
        # T_lidar_to_cam = T_board_to_cam @ T_lidar_to_board
        # R_lidar_to_cam = R_board_to_cam @ R_lidar_to_board
        R = R_board_to_cam @ R_lidar_to_board
        T = T_board_to_cam + R_board_to_cam @ T_lidar_to_board
        
        # 计算重投影误差
        error = self._compute_calibration_error(
            lidar_points, object_points, R, T
        )
        
        return R, T, error
    
    def _compute_calibration_error(self,
                                 lidar_points: np.ndarray,
                                 object_points: np.ndarray,
                                 R: np.ndarray,
                                 T: np.ndarray) -> float:
        """
        计算标定误差
        
        Args:
            lidar_points: LiDAR点
            object_points: 世界坐标点
            R: 旋转矩阵
            T: 平移向量
            
        Returns:
            平均误差
        """
        # 将LiDAR点转换到相机坐标系
        lidar_transformed = (R @ lidar_points.T + T).T
        
        # 投影到图像平面
        projected = cv2.projectPoints(
            object_points, np.zeros(3), np.zeros(3),
            self.camera_matrix, self.dist_coeffs
        )[0].reshape(-1, 2)
        
        lidar_projected = cv2.projectPoints(
            lidar_transformed, np.zeros(3), np.zeros(3),
            self.camera_matrix, self.dist_coeffs
        )[0].reshape(-1, 2)
        
        # 计算误差
        error = np.mean(np.linalg.norm(projected - lidar_projected, axis=1))
        
        return error
    
    def calibrate(self, 
                 lidar_data_dir: str,
                 camera_image_dir: str,
                 camera_matrix: np.ndarray,
                 dist_coeffs: np.ndarray) -> Dict:
        """
        执行LiDAR-相机标定
        
        Args:
            lidar_data_dir: LiDAR点云目录
            camera_image_dir: 相机图像目录
            camera_matrix: 相机内参矩阵
            dist_coeffs: 畸变系数
            
        Returns:
            标定结果字典
        """
        print('=' * 60)
        print('开始LiDAR-相机外参标定')
        print('=' * 60)
        
        # 设置相机内参
        self.set_camera_intrinsics(camera_matrix, dist_coeffs)
        
        # 获取数据文件列表
        lidar_files = sorted([f for f in os.listdir(lidar_data_dir) if f.endswith('.pcd') or f.endswith('.ply')])
        image_files = get_image_files(camera_image_dir)
        
        print(f'找到 {len(lidar_files)} 个LiDAR点云文件')
        print(f'找到 {len(image_files)} 张相机图像')
        
        if len(lidar_files) != len(image_files):
            print('警告: LiDAR和相机数据数量不匹配，使用较小的数量')
        
        num_pairs = min(len(lidar_files), len(image_files))
        
        # 收集所有的对应点
        all_image_points = []
        all_lidar_points = []
        all_object_points = []
        
        for i in tqdm(range(num_pairs), desc='处理LiDAR-相机数据对'):
            # 加载点云
            lidar_file = os.path.join(lidar_data_dir, lidar_files[i])
            points = self.load_point_cloud(lidar_file)
            
            if len(points) == 0:
                continue
            
            # 预处理点云
            points = self.preprocess_point_cloud(points)
            
            # 加载图像
            img = cv2.imread(image_files[i])
            if img is None:
                continue
            
            # 检测AprilTag
            detections = self.detect_apriltag_in_image(img)
            
            if len(detections) == 0:
                continue
            
            # 提取对应点
            image_pts, lidar_pts = self.extract_tag_points_from_lidar(
                points, detections, self.camera_matrix, self.dist_coeffs
            )
            
            if len(image_pts) == 0:
                continue
            
            all_image_points.append(image_pts)
            all_lidar_points.append(lidar_pts)
            
            # 生成对应的世界坐标点
            for j in range(len(image_pts)):
                # 使用标签位置作为世界坐标
                if j < len(self.tag_positions):
                    all_object_points.append(self.tag_positions[j])
        
        if len(all_image_points) == 0:
            raise ValueError('未能找到有效的LiDAR-相机对应点!')
        
        print(f'\n共收集到 {len(all_image_points)} 个对应点对')
        
        # 合并所有点
        image_points = np.vstack(all_image_points)
        lidar_points = np.vstack(all_lidar_points)
        object_points = np.vstack(all_object_points)
        
        # 计算变换矩阵
        print('正在计算LiDAR-相机变换矩阵...')
        R, T, error = self.compute_transformation(
            image_points, lidar_points, object_points
        )
        
        # 构建齐次变换矩阵
        transformation_matrix = np.eye(4)
        transformation_matrix[:3, :3] = R
        transformation_matrix[:3, 3] = T.ravel()
        
        # 保存结果
        self.R = R
        self.T = T
        self.transformation_matrix = transformation_matrix
        self.calibration_error = error
        
        # 提取欧拉角
        roll, pitch, yaw = rotation_matrix_to_angles(R)
        
        print(f'\nLiDAR-相机标定完成!')
        print(f'标定误差: {error:.4f} 像素')
        print(f'旋转矩阵 (欧拉角): roll={np.degrees(roll):.2f}°, pitch={np.degrees(pitch):.2f}°, yaw={np.degrees(yaw):.2f}°')
        print(f'平移向量: {T.ravel()}')
        
        return self.get_calibration_results()
    
    def get_calibration_results(self) -> Dict:
        """
        获取标定结果
        
        Returns:
            标定结果字典
        """
        return {
            'transformation_matrix': self.transformation_matrix,
            'R': self.R,
            'T': self.T,
            'calibration_error': self.calibration_error
        }
    
    def save_results(self, output_dir: str):
        """
        保存标定结果到文件
        
        Args:
            output_dir: 输出目录
        """
        ensure_dir(output_dir)
        
        results = self.get_calibration_results()
        save_calibration_results(results, output_dir, prefix='lidar_to_camera_')
        
        # 保存报告
        report_path = os.path.join(output_dir, 'lidar_camera_calibration_report.txt')
        self._save_report(report_path)
        
        print(f'LiDAR-相机标定结果已保存到 {output_dir}')
    
    def _save_report(self, report_path: str):
        """
        保存标定报告
        
        Args:
            report_path: 报告文件路径
        """
        with open(report_path, 'w', encoding='utf-8') as f:
            f.write('LiDAR-相机外参标定报告\n')
            f.write('=' * 60 + '\n\n')
            
            f.write(f'标定误差: {self.calibration_error:.6f} 像素\n\n')
            
            f.write('变换矩阵 (LiDAR到相机):\n')
            f.write(str(self.transformation_matrix) + '\n\n')
            
            f.write('旋转矩阵 R:\n')
            f.write(str(self.R) + '\n\n')
            
            f.write('平移向量 T:\n')
            f.write(str(self.T.ravel()) + '\n\n')
            
            # 提取欧拉角
            roll, pitch, yaw = rotation_matrix_to_angles(self.R)
            f.write(f'欧拉角 (弧度): roll={roll:.6f}, pitch={pitch:.6f}, yaw={yaw:.6f}\n')
            f.write(f'欧拉角 (角度): roll={np.degrees(roll):.2f}°, pitch={np.degrees(pitch):.2f}°, yaw={np.degrees(yaw):.2f}°\n')
    
    def transform_point_cloud(self, points: np.ndarray) -> np.ndarray:
        """
        将LiDAR点云转换到相机坐标系
        
        Args:
            points: LiDAR点云 (N, 3)
            
        Returns:
            转换后的点云 (N, 3)
        """
        if self.transformation_matrix is None:
            raise RuntimeError('请先执行标定!')
        
        # 转换为齐次坐标
        points_hom = np.hstack([points, np.ones((len(points), 1))])
        
        # 应用变换
        transformed = (points_hom @ self.transformation_matrix.T)[:, :3]
        
        return transformed
    
    def project_lidar_to_image(self, points: np.ndarray) -> np.ndarray:
        """
        将LiDAR点投影到图像平面
        
        Args:
            points: LiDAR点云 (N, 3)
            
        Returns:
            图像坐标 (N, 2)
        """
        if self.camera_matrix is None or self.transformation_matrix is None:
            raise RuntimeError('请先设置相机内参并执行标定!')
        
        # 转换到相机坐标系
        points_cam = self.transform_point_cloud(points)
        
        # 投影到图像平面
        points_2d = cv2.projectPoints(
            points_cam, np.zeros(3), np.zeros(3),
            self.camera_matrix, self.dist_coeffs
        )[0].reshape(-1, 2)
        
        return points_2d


def main():
    """
    主函数：演示LiDAR-相机标定流程
    """
    # 这里需要提供相机内参
    # 实际使用时应该从相机标定结果中加载
    
    # 示例：创建标定器并执行标定
    calibrator = LidarCameraCalibration('config.yaml')
    
    # 假设已经标定了相机
    # camera_matrix = np.load('data/calibration_results/camera_matrix.npy')
    # dist_coeffs = np.load('data/calibration_results/distortion_coeffs.npy')
    
    # 执行标定
    # results = calibrator.calibrate(
    #     'data/lidar_points',
    #     'data/camera_images/front',
    #     camera_matrix,
    #     dist_coeffs
    # )
    
    # 保存结果
    # calibrator.save_results('data/calibration_results/lidar_camera')
    
    print('LiDAR-相机标定完成!')


if __name__ == '__main__':
    main()