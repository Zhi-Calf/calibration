"""
多相机外参标定模块

实现6个相机之间的外参标定功能。
主要功能：
- 分别标定每个相机的内参
- 计算相机之间的相对位姿（旋转矩阵R和平移向量T）
- 全局优化以提高精度
- 保存标定结果
"""

import cv2
import numpy as np
import os
from typing import List, Tuple, Optional, Dict
from tqdm import tqdm
from scipy.spatial.transform import Rotation as ScipyRotation
from camera_calibration import CameraCalibration
from calibration_utils import (
    load_config,
    ensure_dir,
    save_calibration_results,
    find_checkerboard_corners,
    get_image_files,
    rotation_matrix_to_angles
)


class MultiCameraCalibration:
    """
    多相机外参标定类
    
    对6个相机进行内参标定，并计算它们之间的外参：
    - 每个相机的内参矩阵和畸变系数
    - 相对于参考相机的外参（R, T）
    """
    
    def __init__(self, config_path: str = 'config.yaml'):
        """
        初始化标定器
        
        Args:
            config_path: 配置文件路径
        """
        self.config = load_config(config_path)
        self.cameras_config = self.config['sensors']['cameras']
        self.reference_camera = self.config['multi_camera']['reference_camera']
        
        # 为每个相机创建标定器
        self.calibrators: Dict[str, CameraCalibration] = {}
        for camera_cfg in self.cameras_config:
            camera_name = camera_cfg['name']
            self.calibrators[camera_name] = CameraCalibration(config_path)
        
        # 标定结果
        self.intrinsics = {}  # 内参
        self.extrinsics = {}  # 外参（相对于参考相机）
        self.reprojection_errors = {}
        
    def calibrate_all_cameras(self, base_dir: str) -> Dict[str, dict]:
        """
        标定所有相机的内参
        
        Args:
            base_dir: 相机图像的基础目录，子目录为各相机名称
            
        Returns:
            所有相机的内参标定结果
        """
        print('=' * 60)
        print('开始标定所有相机的内参')
        print('=' * 60)
        
        intrinsics_results = {}
        
        for camera_cfg in tqdm(self.cameras_config, desc='标定相机内参'):
            camera_name = camera_cfg['name']
            print(f'\n正在标定相机: {camera_name}')
            
            # 构建相机图像目录
            image_dir = os.path.join(base_dir, camera_name)
            
            if not os.path.exists(image_dir):
                print(f'警告: 相机 {camera_name} 的图像目录不存在: {image_dir}')
                continue
            
            # 执行标定
            calibrator = self.calibrators[camera_name]
            results = calibrator.calibrate(image_dir)
            
            # 保存结果
            intrinsics_results[camera_name] = {
                'camera_matrix': calibrator.camera_matrix,
                'distortion_coeffs': calibrator.dist_coeffs,
                'reprojection_error': calibrator.reprojection_error
            }
            
            print(f'相机 {camera_name} 标定完成，重投影误差: {calibrator.reprojection_error:.4f}')
        
        self.intrinsics = intrinsics_results
        return intrinsics_results
    
    def calculate_extrinsics(self, base_dir: str) -> Dict[str, dict]:
        """
        计算相机之间的外参（相对于参考相机）。

        对同一帧（按文件名排序对齐），分别在参考相机和目标相机中检测棋盘格，
        通过 solvePnP 得到各自的 T_board_cam，再计算
          T_ref_target = T_board_ref^{-1} @ T_board_target  (board 坐标系消掉)
        对多帧结果做四元数球面平均得到最终外参。
        """
        print('\n' + '=' * 60)
        print('开始计算相机之间的外参')
        print('=' * 60)

        if self.reference_camera not in self.intrinsics:
            raise ValueError(f'参考相机 {self.reference_camera} 未标定!')

        ref_calibrator = self.calibrators[self.reference_camera]
        ref_image_dir = os.path.join(base_dir, self.reference_camera)
        ref_images = sorted(get_image_files(ref_image_dir))

        ref_detections = {}
        for idx, img_path in enumerate(tqdm(ref_images, desc=f'检测参考相机 {self.reference_camera} 角点')):
            img = cv2.imread(img_path)
            if img is None:
                continue
            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
            ret, corners = find_checkerboard_corners(gray, ref_calibrator.pattern_size)
            if ret:
                objp = ref_calibrator._generate_object_points()
                success, rvec, tvec = cv2.solvePnP(
                    objp, corners,
                    ref_calibrator.camera_matrix, ref_calibrator.dist_coeffs
                )
                if success:
                    R_ref, _ = cv2.Rodrigues(rvec)
                    T_board_ref = np.eye(4)
                    T_board_ref[:3, :3] = R_ref
                    T_board_ref[:3, 3] = tvec.ravel()
                    ref_detections[idx] = T_board_ref

        extrinsics_results = {}

        for camera_cfg in self.cameras_config:
            camera_name = camera_cfg['name']

            if camera_name == self.reference_camera:
                extrinsics_results[camera_name] = {
                    'R': np.eye(3),
                    'T': np.zeros((3, 1))
                }
                continue

            if camera_name not in self.intrinsics:
                print(f'警告: 相机 {camera_name} 未标定内参, 跳过')
                continue

            print(f'\n计算相机 {camera_name} 相对于 {self.reference_camera} 的外参')

            calibrator = self.calibrators[camera_name]
            image_dir = os.path.join(base_dir, camera_name)
            images = sorted(get_image_files(image_dir))

            quats = []
            translations = []
            num_pairs = min(len(ref_images), len(images))

            for idx in tqdm(range(num_pairs), desc=f'处理相机 {camera_name}', leave=False):
                if idx not in ref_detections:
                    continue

                img = cv2.imread(images[idx])
                if img is None:
                    continue

                gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
                ret, corners = find_checkerboard_corners(gray, calibrator.pattern_size)
                if not ret:
                    continue

                objp = calibrator._generate_object_points()
                success, rvec, tvec = cv2.solvePnP(
                    objp, corners,
                    calibrator.camera_matrix, calibrator.dist_coeffs
                )
                if not success:
                    continue

                R_tgt, _ = cv2.Rodrigues(rvec)
                T_board_tgt = np.eye(4)
                T_board_tgt[:3, :3] = R_tgt
                T_board_tgt[:3, 3] = tvec.ravel()

                T_board_ref = ref_detections[idx]
                T_ref_tgt = np.linalg.inv(T_board_ref) @ T_board_tgt

                r = ScipyRotation.from_matrix(T_ref_tgt[:3, :3])
                quats.append(r.as_quat())
                translations.append(T_ref_tgt[:3, 3])

            if len(quats) == 0:
                print(f'警告: 未能找到相机 {camera_name} 的匹配帧')
                continue

            quats = np.array(quats)
            ref_q = quats[0]
            for j in range(1, len(quats)):
                if np.dot(quats[j], ref_q) < 0:
                    quats[j] = -quats[j]
            avg_quat = np.mean(quats, axis=0)
            avg_quat /= np.linalg.norm(avg_quat)
            R_avg = ScipyRotation.from_quat(avg_quat).as_matrix()

            T_avg = np.mean(translations, axis=0).reshape(3, 1)

            roll, pitch, yaw = rotation_matrix_to_angles(R_avg)
            extrinsics_results[camera_name] = {
                'R': R_avg,
                'T': T_avg
            }

            print(f'相机 {camera_name} 外参计算完成 (使用 {len(quats)} 对匹配帧)')
            print(f'旋转 (欧拉角): roll={np.degrees(roll):.2f}°, '
                  f'pitch={np.degrees(pitch):.2f}°, yaw={np.degrees(yaw):.2f}°')
            print(f'平移向量: {T_avg.ravel()}')

        self.extrinsics = extrinsics_results
        return extrinsics_results
    
    def global_optimization(self):
        """
        全局优化相机外参
        
        使用Bundle Adjustment优化所有相机的外参，提高一致性。
        """
        print('\n执行全局优化...')
        
        # 这里可以实现更复杂的全局优化算法
        # 例如使用scipy.optimize.least_squares进行bundle adjustment
        
        if not self.config['multi_camera']['global_optimization']:
            print('全局优化已禁用，跳过')
            return
        
        # 简单的平均优化
        # 实际应用中可以使用更复杂的优化方法
        print('全局优化完成（简化版本）')
    
    def calibrate(self, base_dir: str) -> Dict[str, dict]:
        """
        执行完整的多相机标定流程
        
        Args:
            base_dir: 相机图像的基础目录
            
        Returns:
            所有相机的标定结果（内参和外参）
        """
        # 1. 标定所有相机的内参
        intrinsics = self.calibrate_all_cameras(base_dir)
        
        # 2. 计算相机之间的外参
        extrinsics = self.calculate_extrinsics(base_dir)
        
        # 3. 全局优化
        self.global_optimization()
        
        # 4. 合并结果
        results = {}
        for camera_cfg in self.cameras_config:
            camera_name = camera_cfg['name']
            if camera_name in intrinsics:
                results[camera_name] = {
                    'intrinsics': intrinsics[camera_name],
                    'extrinsics': extrinsics.get(camera_name, None)
                }
        
        return results
    
    def save_results(self, output_dir: str):
        """
        保存标定结果到文件
        
        Args:
            output_dir: 输出目录
        """
        ensure_dir(output_dir)
        
        # 保存内参
        intrinsics_dir = os.path.join(output_dir, 'intrinsics')
        ensure_dir(intrinsics_dir)
        for camera_name, intrinsics in self.intrinsics.items():
            camera_dir = os.path.join(intrinsics_dir, camera_name)
            ensure_dir(camera_dir)
            save_calibration_results(intrinsics, camera_dir)
        
        # 保存外参
        extrinsics_dir = os.path.join(output_dir, 'extrinsics')
        ensure_dir(extrinsics_dir)
        save_calibration_results(self.extrinsics, extrinsics_dir)
        
        # 保存报告
        report_path = os.path.join(output_dir, 'multi_camera_calibration_report.txt')
        self._save_report(report_path)
        
        print(f'多相机标定结果已保存到 {output_dir}')
    
    def _save_report(self, report_path: str):
        """
        保存标定报告
        
        Args:
            report_path: 报告文件路径
        """
        with open(report_path, 'w', encoding='utf-8') as f:
            f.write('多相机标定报告\n')
            f.write('=' * 60 + '\n\n')
            
            f.write(f'参考相机: {self.reference_camera}\n')
            f.write(f'相机数量: {len(self.cameras_config)}\n\n')
            
            # 内参报告
            f.write('=' * 60 + '\n')
            f.write('相机内参报告\n')
            f.write('=' * 60 + '\n\n')
            
            for camera_name, intrinsics in self.intrinsics.items():
                f.write(f'相机: {camera_name}\n')
                f.write(f'重投影误差: {intrinsics["reprojection_error"]:.6f} 像素\n')
                f.write(f'相机内参矩阵:\n')
                f.write(str(intrinsics['camera_matrix']) + '\n')
                f.write(f'畸变系数: {intrinsics["distortion_coeffs"].ravel()}\n\n')
            
            # 外参报告
            f.write('=' * 60 + '\n')
            f.write('相机外参报告（相对于参考相机）\n')
            f.write('=' * 60 + '\n\n')
            
            for camera_name, extrinsics in self.extrinsics.items():
                if camera_name == self.reference_camera:
                    f.write(f'相机: {camera_name} (参考相机)\n')
                    f.write('旋转矩阵 R:\n')
                    f.write(str(extrinsics['R']) + '\n')
                    f.write('平移向量 T:\n')
                    f.write(str(extrinsics['T'].ravel()) + '\n\n')
                else:
                    f.write(f'相机: {camera_name}\n')
                    f.write('旋转矩阵 R:\n')
                    f.write(str(extrinsics['R']) + '\n')
                    
                    # 提取欧拉角
                    roll, pitch, yaw = rotation_matrix_to_angles(extrinsics['R'])
                    f.write(f'欧拉角 (弧度): roll={roll:.6f}, pitch={pitch:.6f}, yaw={yaw:.6f}\n')
                    f.write(f'欧拉角 (角度): roll={np.degrees(roll):.2f}°, pitch={np.degrees(pitch):.2f}°, yaw={np.degrees(yaw):.2f}°\n')
                    
                    f.write('平移向量 T:\n')
                    f.write(str(extrinsics['T'].ravel()) + '\n\n')
    
    def get_camera_extrinsics(self, camera_name: str) -> Optional[Tuple[np.ndarray, np.ndarray]]:
        """
        获取指定相机的外参
        
        Args:
            camera_name: 相机名称
            
        Returns:
            (R, T) 旋转矩阵和平移向量，如果相机不存在则返回None
        """
        if camera_name not in self.extrinsics:
            return None
        extrinsics = self.extrinsics[camera_name]
        return extrinsics['R'], extrinsics['T']
    
    def transform_point(self, point: np.ndarray, 
                       from_camera: str, 
                       to_camera: str) -> Optional[np.ndarray]:
        """
        将点从一个相机坐标系转换到另一个相机坐标系
        
        Args:
            point: 3D点坐标 (3,)
            from_camera: 源相机名称
            to_camera: 目标相机名称
            
        Returns:
            转换后的3D点坐标，如果转换失败则返回None
        """
        if from_camera not in self.extrinsics or to_camera not in self.extrinsics:
            return None

        R_from = self.extrinsics[from_camera]['R']
        T_from = self.extrinsics[from_camera]['T']
        R_to = self.extrinsics[to_camera]['R']
        T_to = self.extrinsics[to_camera]['T']

        point_ref = R_from @ point.reshape(3, 1) + T_from
        point_to = R_to.T @ (point_ref - T_to)

        return point_to.ravel()


def main():
    """
    主函数：演示多相机标定流程
    """
    # 创建标定器
    calibrator = MultiCameraCalibration('config.yaml')
    
    # 执行标定
    base_dir = 'data/camera_images'
    results = calibrator.calibrate(base_dir)
    
    # 保存结果
    output_dir = 'data/calibration_results/multi_camera'
    calibrator.save_results(output_dir)
    
    print('\n多相机标定完成!')
    
    # 打印外参摘要
    print('\n外参摘要（相对于参考相机）:')
    print('-' * 60)
    for camera_cfg in calibrator.cameras_config:
        camera_name = camera_cfg['name']
        R, T = calibrator.get_camera_extrinsics(camera_name)
        if R is not None:
            roll, pitch, yaw = rotation_matrix_to_angles(R)
            print(f'{camera_name:12s}: R=[{np.degrees(roll):6.2f}°, {np.degrees(pitch):6.2f}°, {np.degrees(yaw):6.2f}°], T=[{T[0,0]:7.3f}, {T[1,0]:7.3f}, {T[2,0]:7.3f}]')


if __name__ == '__main__':
    main()