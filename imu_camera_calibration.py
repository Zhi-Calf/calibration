"""
IMU-相机外参标定模块

实现IMU与相机之间的外参标定功能。
主要功能：
- 加载IMU数据和相机图像
- 估计IMU的零偏和比例因子
- 计算IMU到相机的外参矩阵
- 使用静止时间段进行标定
"""

import cv2
import numpy as np
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


class IMUCameraCalibration:
    """
    IMU-相机外参标定类
    
    使用静止时间段和AprilTag标定板建立IMU与相机之间的对应关系，
    计算IMU到相机的外参矩阵（4x4变换矩阵）和IMU的零偏。
    """
    
    def __init__(self, config_path: str = 'config.yaml'):
        """
        初始化标定器
        
        Args:
            config_path: 配置文件路径
        """
        self.config = load_config(config_path)
        self.imu_config = self.config['imu_camera']['imu']
        
        # 标定结果
        self.transformation_matrix = None  # 4x4变换矩阵
        self.R = None  # 旋转矩阵 3x3
        self.T = None  # 平移向量 3x1
        self.calibration_error = None
        
        # IMU参数
        self.accel_bias = None  # 加速度计零偏
        self.gyro_bias = None   # 陀螺仪零偏
        self.gravity = np.array([0, 0, -9.81])  # 重力向量 (m/s²)
        
        # 相机内参
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
        
    def load_imu_data(self, imu_file: str) -> Dict:
        """
        加载IMU CSV 数据。

        格式要求：第一行为表头，后续每行包含
          timestamp, accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z
        timestamp 单位为秒（float64），与图像时间戳对齐。
        """
        try:
            data = np.loadtxt(imu_file, delimiter=',', skiprows=1)
            if data.ndim == 1:
                data = data.reshape(1, -1)
            if data.shape[1] < 7:
                raise ValueError(f'IMU 数据列数不足: 期望 >=7, 实际 {data.shape[1]}')

            imu_data = {
                'timestamp': data[:, 0],
                'accel': data[:, 1:4],
                'gyro': data[:, 4:7]
            }

            print(f'加载IMU数据: {len(imu_data["timestamp"])} 个数据点, '
                  f'时间范围: [{data[0, 0]:.3f}, {data[-1, 0]:.3f}]')
            return imu_data

        except Exception as e:
            print(f'加载IMU数据失败: {e}')
            return {}
    
    def detect_static_periods(self, imu_data: Dict) -> List[Tuple[int, int]]:
        """
        检测静止时间段
        
        通过分析加速度计和陀螺仪的方差来检测静止状态。
        
        Args:
            imu_data: IMU数据字典
            
        Returns:
            静止时间段列表 [(start_idx, end_idx), ...]
        """
        accel = imu_data['accel']
        gyro = imu_data['gyro']
        
        threshold = self.imu_config['static_threshold']
        window_size = self.imu_config['window_size']
        
        static_periods = []
        in_static = False
        start_idx = 0
        
        # 滑动窗口检测
        for i in range(len(accel) - window_size + 1):
            window_accel = accel[i:i+window_size]
            window_gyro = gyro[i:i+window_size]
            
            # 计算方差
            accel_var = np.var(window_accel, axis=0)
            gyro_var = np.var(window_gyro, axis=0)
            
            # 检查是否静止
            if np.all(accel_var < threshold**2) and np.all(gyro_var < threshold**2):
                if not in_static:
                    start_idx = i
                    in_static = True
            else:
                if in_static:
                    end_idx = i - 1
                    if end_idx - start_idx >= window_size:
                        static_periods.append((start_idx, end_idx))
                    in_static = False
        
        # 添加最后一个静止段
        if in_static:
            static_periods.append((start_idx, len(accel) - 1))
        
        print(f'检测到 {len(static_periods)} 个静止时间段')
        return static_periods
    
    def estimate_imu_bias(self, imu_data: Dict, static_periods: List[Tuple[int, int]]):
        """
        估计IMU零偏
        
        使用静止时间段的数据估计加速度计和陀螺仪的零偏。
        
        Args:
            imu_data: IMU数据字典
            static_periods: 静止时间段列表
        """
        accel_bias_samples = []
        gyro_bias_samples = []
        
        for start_idx, end_idx in static_periods:
            # 提取静止时间段的数据
            static_accel = imu_data['accel'][start_idx:end_idx+1]
            static_gyro = imu_data['gyro'][start_idx:end_idx+1]
            
            # 计算平均值
            accel_bias_samples.append(np.mean(static_accel, axis=0))
            gyro_bias_samples.append(np.mean(static_gyro, axis=0))
        
        accel_mean = np.mean(accel_bias_samples, axis=0)
        self.gyro_bias = np.mean(gyro_bias_samples, axis=0)

        accel_norm = np.linalg.norm(accel_mean)
        if accel_norm < 1e-6:
            self.accel_bias = accel_mean
        else:
            gravity_direction = accel_mean / accel_norm
            gravity_in_imu = gravity_direction * 9.81
            self.accel_bias = accel_mean - gravity_in_imu

        print(f'估计加速度计零偏 (去重力): {self.accel_bias}')
        print(f'估计陀螺仪零偏: {self.gyro_bias}')
    
    def detect_apriltag_pose(self, image: np.ndarray) -> Optional[Tuple[np.ndarray, np.ndarray]]:
        """
        检测AprilTag在图像中的位姿
        
        Args:
            image: 输入图像
            
        Returns:
            (rvec, tvec): 旋转向量和平移向量，如果检测失败则返回None
        """
        # 使用ArUco标记（作为AprilTag的替代）
        aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
        aruco_params = cv2.aruco.DetectorParameters()
        
        # 检测标记
        corners, ids, rejected = cv2.aruco.detectMarkers(image, aruco_dict, parameters=aruco_params)
        
        if ids is not None and len(ids) > 0:
            # 定义标记的3D坐标（假设标记在平面上）
            marker_size = self.config['imu_camera']['apriltag']['tag_size']
            obj_points = np.array([
                [-marker_size/2, -marker_size/2, 0],
                [marker_size/2, -marker_size/2, 0],
                [marker_size/2, marker_size/2, 0],
                [-marker_size/2, marker_size/2, 0]
            ], dtype=np.float32)
            
            # 计算标记相对于相机的位姿
            success, rvec, tvec = cv2.solvePnP(
                obj_points, corners[0],
                self.camera_matrix, self.dist_coeffs
            )
            
            if success:
                return rvec, tvec
        
        return None
    
    def compute_gravity_direction_from_imu(self, imu_data: Dict, image_idx: int) -> np.ndarray:
        """
        从IMU数据计算重力方向
        
        Args:
            imu_data: IMU数据字典
            image_idx: 图像对应的IMU数据索引
            
        Returns:
            重力方向向量（在IMU坐标系中）
        """
        # 使用最近的IMU数据
        accel = imu_data['accel'][image_idx]
        
        # 去除零偏
        accel_corrected = accel - self.accel_bias
        
        # 重力方向就是加速度的方向（归一化）
        gravity_imu = -accel_corrected / np.linalg.norm(accel_corrected)
        
        return gravity_imu
    
    def compute_gravity_direction_from_camera(self, rvec: np.ndarray, tvec: np.ndarray) -> np.ndarray:
        """
        从相机位姿计算重力方向
        
        假设标定板水平放置，重力垂直于标定板向下。
        
        Args:
            rvec: 相机相对于标定板的旋转向量
            tvec: 相机相对于标定板的平移向量
            
        Returns:
            重力方向向量（在相机坐标系中）
        """
        # 计算旋转矩阵
        R_cam_board = cv2.Rodrigues(rvec)[0]
        
        # 标定板的法向量在世界坐标系中是 [0, 0, 1]
        # 重力在世界坐标系中是 [0, 0, -1]
        gravity_world = np.array([0, 0, -1])
        
        # 将重力方向转换到相机坐标系
        gravity_camera = R_cam_board @ gravity_world
        
        return gravity_camera
    
    def calibrate(self,
                 imu_data_dir: str,
                 camera_image_dir: str,
                 camera_matrix: np.ndarray,
                 dist_coeffs: np.ndarray) -> Dict:
        """
        执行IMU-相机标定
        
        Args:
            imu_data_dir: IMU数据目录
            camera_image_dir: 相机图像目录
            camera_matrix: 相机内参矩阵
            dist_coeffs: 畸变系数
            
        Returns:
            标定结果字典
        """
        print('=' * 60)
        print('开始IMU-相机外参标定')
        print('=' * 60)
        
        # 设置相机内参
        self.set_camera_intrinsics(camera_matrix, dist_coeffs)
        
        # 加载IMU数据
        imu_files = sorted([f for f in os.listdir(imu_data_dir) if f.endswith('.csv')])
        if len(imu_files) == 0:
            raise ValueError('未找到IMU数据文件!')
        
        imu_file = os.path.join(imu_data_dir, imu_files[0])
        imu_data = self.load_imu_data(imu_file)
        
        if len(imu_data) == 0:
            raise ValueError('IMU数据加载失败!')
        
        # 检测静止时间段
        if self.imu_config['estimate_bias']:
            print('检测静止时间段...')
            static_periods = self.detect_static_periods(imu_data)
            self.estimate_imu_bias(imu_data, static_periods)
        else:
            print('跳过零偏估计')
        
        # 获取相机图像
        image_files = get_image_files(camera_image_dir)
        print(f'找到 {len(image_files)} 张相机图像')
        
        # 收集重力方向对
        gravity_imu_list = []
        gravity_camera_list = []
        
        imu_timestamps = imu_data['timestamp']

        for i, image_file in enumerate(tqdm(image_files, desc='处理IMU-相机数据对')):
            img = cv2.imread(image_file)
            if img is None:
                continue

            pose = self.detect_apriltag_pose(img)
            if pose is None:
                continue

            rvec, tvec = pose

            image_ts = self._extract_timestamp_from_filename(image_file)
            if image_ts is not None:
                imu_idx = int(np.argmin(np.abs(imu_timestamps - image_ts)))
                if abs(imu_timestamps[imu_idx] - image_ts) > 0.1:
                    print(f'  帧 {i}: 时间差过大 ({abs(imu_timestamps[imu_idx] - image_ts):.3f}s), 跳过')
                    continue
            else:
                imu_idx = int(i * len(imu_timestamps) / len(image_files))
                imu_idx = min(imu_idx, len(imu_timestamps) - 1)

            gravity_imu = self.compute_gravity_direction_from_imu(imu_data, imu_idx)
            gravity_camera = self.compute_gravity_direction_from_camera(rvec, tvec)

            gravity_imu_list.append(gravity_imu)
            gravity_camera_list.append(gravity_camera)
        
        if len(gravity_imu_list) == 0:
            raise ValueError('未能找到有效的IMU-相机数据对!')
        
        print(f'\n共收集到 {len(gravity_imu_list)} 个重力方向对')
        
        # 转换为numpy数组
        gravity_imu = np.array(gravity_imu_list)
        gravity_camera = np.array(gravity_camera_list)
        
        # 计算IMU到相机的旋转矩阵
        print('正在计算IMU-相机旋转矩阵...')
        R = self._compute_rotation_from_vectors(gravity_imu, gravity_camera)
        
        # 平移向量通常难以精确估计，这里设为0
        # 实际应用中可能需要额外的标定板来确定平移
        T = np.zeros((3, 1))
        
        # 构建齐次变换矩阵
        transformation_matrix = np.eye(4)
        transformation_matrix[:3, :3] = R
        transformation_matrix[:3, 3] = T.ravel()
        
        # 保存结果
        self.R = R
        self.T = T
        self.transformation_matrix = transformation_matrix
        
        # 计算标定误差
        self.calibration_error = self._compute_calibration_error(
            gravity_imu, gravity_camera, R
        )
        
        # 提取欧拉角
        roll, pitch, yaw = rotation_matrix_to_angles(R)
        
        print(f'\nIMU-相机标定完成!')
        print(f'标定误差: {self.calibration_error:.4f} 弧度')
        print(f'旋转矩阵 (欧拉角): roll={np.degrees(roll):.2f}°, pitch={np.degrees(pitch):.2f}°, yaw={np.degrees(yaw):.2f}°')
        print(f'平移向量: {T.ravel()} (未精确标定)')
        
        return self.get_calibration_results()
    
    @staticmethod
    def _extract_timestamp_from_filename(filepath: str) -> Optional[float]:
        """
        从文件名中提取时间戳（如 1672531200.123.png → 1672531200.123）。
        如果文件名不包含有效时间戳则返回 None，此时退化到线性索引对齐。
        """
        basename = os.path.splitext(os.path.basename(filepath))[0]
        try:
            return float(basename)
        except ValueError:
            return None

    def _compute_rotation_from_vectors(self,
                                      vectors_imu: np.ndarray,
                                      vectors_camera: np.ndarray) -> np.ndarray:
        """
        从方向向量对计算最优旋转矩阵 R 使得 R @ v_imu ≈ v_cam。
        使用 SVD（Wahba 问题的解法），不做中心化（因为输入是方向，不是点）。
        """
        H = vectors_imu.T @ vectors_camera

        U, S, Vt = np.linalg.svd(H)

        d = np.linalg.det(Vt.T @ U.T)
        D = np.diag([1.0, 1.0, d])
        R = Vt.T @ D @ U.T

        return R
    
    def _compute_calibration_error(self,
                                  gravity_imu: np.ndarray,
                                  gravity_camera: np.ndarray,
                                  R: np.ndarray) -> float:
        """
        计算标定误差
        
        Args:
            gravity_imu: IMU坐标系中的重力方向
            gravity_camera: 相机坐标系中的重力方向
            R: 旋转矩阵
            
        Returns:
            平均误差（弧度）
        """
        # 将IMU中的重力方向转换到相机坐标系
        gravity_imu_transformed = (R @ gravity_imu.T).T
        
        # 计算角度误差
        errors = []
        for i in range(len(gravity_imu)):
            # 计算两个向量之间的夹角
            cos_angle = np.dot(gravity_imu_transformed[i], gravity_camera[i])
            cos_angle = np.clip(cos_angle, -1.0, 1.0)
            angle = np.arccos(cos_angle)
            errors.append(angle)
        
        return np.mean(errors)
    
    def get_calibration_results(self) -> Dict:
        """
        获取标定结果
        
        Returns:
            标定结果字典
        """
        results = {
            'transformation_matrix': self.transformation_matrix,
            'R': self.R,
            'T': self.T,
            'accel_bias': self.accel_bias,
            'gyro_bias': self.gyro_bias,
            'calibration_error': self.calibration_error
        }
        return results
    
    def save_results(self, output_dir: str):
        """
        保存标定结果到文件
        
        Args:
            output_dir: 输出目录
        """
        ensure_dir(output_dir)
        
        results = self.get_calibration_results()
        save_calibration_results(results, output_dir, prefix='imu_to_camera_')
        
        # 保存报告
        report_path = os.path.join(output_dir, 'imu_camera_calibration_report.txt')
        self._save_report(report_path)
        
        print(f'IMU-相机标定结果已保存到 {output_dir}')
    
    def _save_report(self, report_path: str):
        """
        保存标定报告
        
        Args:
            report_path: 报告文件路径
        """
        with open(report_path, 'w', encoding='utf-8') as f:
            f.write('IMU-相机外参标定报告\n')
            f.write('=' * 60 + '\n\n')
            
            f.write(f'标定误差: {self.calibration_error:.6f} 弧度\n')
            f.write(f'标定误差: {np.degrees(self.calibration_error):.2f}°\n\n')
            
            if self.accel_bias is not None:
                f.write(f'加速度计零偏: {self.accel_bias}\n')
                f.write(f'陀螺仪零偏: {self.gyro_bias}\n\n')
            
            f.write('变换矩阵 (IMU到相机):\n')
            f.write(str(self.transformation_matrix) + '\n\n')
            
            f.write('旋转矩阵 R:\n')
            f.write(str(self.R) + '\n\n')
            
            f.write('平移向量 T:\n')
            f.write(str(self.T.ravel()) + '\n\n')
            
            # 提取欧拉角
            roll, pitch, yaw = rotation_matrix_to_angles(self.R)
            f.write(f'欧拉角 (弧度): roll={roll:.6f}, pitch={pitch:.6f}, yaw={yaw:.6f}\n')
            f.write(f'欧拉角 (角度): roll={np.degrees(roll):.2f}°, pitch={np.degrees(pitch):.2f}°, yaw={np.degrees(yaw):.2f}°\n')
            f.write('\n注意: 平移向量未精确标定，需要额外的标定步骤\n')


def main():
    """
    主函数：演示IMU-相机标定流程
    """
    # 示例：创建标定器并执行标定
    calibrator = IMUCameraCalibration('config.yaml')
    
    # 假设已经标定了相机
    # camera_matrix = np.load('data/calibration_results/camera_matrix.npy')
    # dist_coeffs = np.load('data/calibration_results/distortion_coeffs.npy')
    
    # 执行标定
    # results = calibrator.calibrate(
    #     'data/imu_data',
    #     'data/camera_images/front',
    #     camera_matrix,
    #     dist_coeffs
    # )
    
    # 保存结果
    # calibrator.save_results('data/calibration_results/imu_camera')
    
    print('IMU-相机标定完成!')


if __name__ == '__main__':
    main()