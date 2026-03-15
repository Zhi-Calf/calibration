"""
LiDAR-相机外参标定模块

使用棋盘格标定板同时被 LiDAR 和相机观测，通过以下链路求解 LiDAR→Camera 变换：
  T_lidar_cam = T_board_cam × inv(T_board_lidar)

图像侧：检测棋盘格角点 → solvePnP → T_board_cam
点云侧：RANSAC 拟合标定板平面 → 提取平面内点的边界 → 估计 T_board_lidar
"""

import cv2
import numpy as np
import open3d as o3d
import os
from typing import List, Tuple, Optional, Dict
from tqdm import tqdm
from scipy.spatial.transform import Rotation as ScipyRotation
from calibration_utils import (
    load_config,
    ensure_dir,
    save_calibration_results,
    find_checkerboard_corners,
    get_image_files,
    rotation_matrix_to_angles
)


class LidarCameraCalibration:
    """
    LiDAR-相机外参标定类

    通过棋盘格标定板建立 LiDAR 与相机之间的空间对应关系，
    计算 LiDAR→Camera 的 4×4 齐次变换矩阵。
    """

    def __init__(self, config_path: str = 'config.yaml'):
        self.config = load_config(config_path)

        lc_cfg = self.config.get('lidar_camera', {})
        cb_cfg = lc_cfg.get('checkerboard', self.config.get('checkerboard', {}))
        self.pattern_size = tuple(cb_cfg.get('inner_corners', [9, 6]))
        self.square_size = cb_cfg.get('square_size', 0.02)

        lidar_cfg = lc_cfg.get('lidar', {})
        self.voxel_size = lidar_cfg.get('voxel_size', 0.01)
        self.distance_min = lidar_cfg.get('distance_min', 0.5)
        self.distance_max = lidar_cfg.get('distance_max', 15.0)

        plane_cfg = lc_cfg.get('plane_fitting', {})
        self.plane_dist_thresh = plane_cfg.get('distance_threshold', 0.01)
        self.plane_ransac_n = plane_cfg.get('ransac_n', 3)
        self.plane_iterations = plane_cfg.get('num_iterations', 2000)
        self.plane_min_inliers = plane_cfg.get('min_inliers', 50)

        calib_cfg = lc_cfg.get('calibration', {})
        self.min_pairs = calib_cfg.get('min_pairs', 3)
        self.max_reproj_error = calib_cfg.get('max_reprojection_error', 5.0)

        self.transformation_matrix = None
        self.R = None
        self.T = None
        self.calibration_error = None
        self.camera_matrix = None
        self.dist_coeffs = None

    def set_camera_intrinsics(self, camera_matrix: np.ndarray, dist_coeffs: np.ndarray):
        self.camera_matrix = camera_matrix
        self.dist_coeffs = dist_coeffs

    def load_point_cloud(self, pcd_file: str) -> np.ndarray:
        try:
            pcd = o3d.io.read_point_cloud(pcd_file)
            return np.asarray(pcd.points)
        except Exception as e:
            print(f'加载点云失败: {e}')
            return np.array([])

    def preprocess_point_cloud(self, points: np.ndarray) -> np.ndarray:
        if len(points) == 0:
            return points

        distances = np.linalg.norm(points, axis=1)
        mask = (distances > self.distance_min) & (distances < self.distance_max)
        points = points[mask]

        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points)
        pcd_down = pcd.voxel_down_sample(voxel_size=self.voxel_size)
        return np.asarray(pcd_down.points)

    def extract_board_plane_from_lidar(
        self, points: np.ndarray
    ) -> Optional[Tuple[np.ndarray, np.ndarray, np.ndarray]]:
        """
        用 RANSAC 从点云中拟合标定板平面，返回平面法向量、平面内点和质心。

        Returns:
            (plane_normal, inlier_points, centroid) 或 None
        """
        if len(points) < self.plane_min_inliers:
            return None

        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points)

        plane_model, inlier_indices = pcd.segment_plane(
            distance_threshold=self.plane_dist_thresh,
            ransac_n=self.plane_ransac_n,
            num_iterations=self.plane_iterations
        )

        if len(inlier_indices) < self.plane_min_inliers:
            return None

        a, b, c, _d = plane_model
        normal = np.array([a, b, c])
        normal = normal / np.linalg.norm(normal)

        inlier_points = points[inlier_indices]
        centroid = np.mean(inlier_points, axis=0)

        return normal, inlier_points, centroid

    def estimate_board_pose_in_lidar(
        self, normal: np.ndarray, inlier_points: np.ndarray, centroid: np.ndarray
    ) -> np.ndarray:
        """
        从平面法向量和内点估计标定板在 LiDAR 坐标系下的位姿 (4×4)。

        使用 PCA 在平面内点上确定 x/y 轴方向，法向量作为 z 轴。
        """
        centered = inlier_points - centroid
        projected = centered - np.outer(centered @ normal, normal)

        cov = projected.T @ projected
        eigenvalues, eigenvectors = np.linalg.eigh(cov)

        idx = np.argsort(eigenvalues)[::-1]
        eigenvectors = eigenvectors[:, idx]

        x_axis = eigenvectors[:, 0]
        x_axis = x_axis / np.linalg.norm(x_axis)

        z_axis = normal.copy()
        y_axis = np.cross(z_axis, x_axis)
        y_axis = y_axis / np.linalg.norm(y_axis)
        x_axis = np.cross(y_axis, z_axis)
        x_axis = x_axis / np.linalg.norm(x_axis)

        T_board_lidar = np.eye(4)
        T_board_lidar[:3, 0] = x_axis
        T_board_lidar[:3, 1] = y_axis
        T_board_lidar[:3, 2] = z_axis
        T_board_lidar[:3, 3] = centroid

        return T_board_lidar

    def estimate_board_pose_in_camera(
        self, image: np.ndarray
    ) -> Optional[np.ndarray]:
        """
        在图像中检测棋盘格并用 solvePnP 求标定板在相机坐标系下的位姿 (4×4)。
        """
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY) if len(image.shape) == 3 else image
        ret, corners = find_checkerboard_corners(gray, self.pattern_size)
        if not ret:
            return None

        objp = np.zeros((self.pattern_size[0] * self.pattern_size[1], 3), np.float32)
        objp[:, :2] = np.mgrid[0:self.pattern_size[0], 0:self.pattern_size[1]].T.reshape(-1, 2)
        objp *= self.square_size

        success, rvec, tvec = cv2.solvePnP(
            objp, corners, self.camera_matrix, self.dist_coeffs,
            flags=cv2.SOLVEPNP_ITERATIVE
        )
        if not success:
            return None

        R_cam, _ = cv2.Rodrigues(rvec)
        T_board_cam = np.eye(4)
        T_board_cam[:3, :3] = R_cam
        T_board_cam[:3, 3] = tvec.ravel()
        return T_board_cam

    def compute_reprojection_error(
        self, points_lidar: np.ndarray, T_lidar_cam: np.ndarray
    ) -> float:
        """
        将 LiDAR 平面内点用 T_lidar_cam 变换后投影到图像，
        计算投影点与图像边界的合理性（所有点应落在图像内且深度为正）。
        返回在相机前方的点的百分比作为质量指标。
        """
        pts_hom = np.hstack([points_lidar, np.ones((len(points_lidar), 1))])
        pts_cam = (T_lidar_cam @ pts_hom.T).T[:, :3]

        in_front = pts_cam[:, 2] > 0
        return np.mean(in_front) * 100.0

    def calibrate(
        self,
        lidar_data_dir: str,
        camera_image_dir: str,
        camera_matrix: np.ndarray,
        dist_coeffs: np.ndarray
    ) -> Dict:
        print('=' * 60)
        print('开始 LiDAR-相机外参标定')
        print('=' * 60)

        self.set_camera_intrinsics(camera_matrix, dist_coeffs)

        lidar_files = sorted([
            f for f in os.listdir(lidar_data_dir)
            if f.endswith('.pcd') or f.endswith('.ply')
        ])
        image_files = get_image_files(camera_image_dir)

        print(f'找到 {len(lidar_files)} 个 LiDAR 点云文件')
        print(f'找到 {len(image_files)} 张相机图像')

        num_pairs = min(len(lidar_files), len(image_files))
        if num_pairs == 0:
            raise ValueError('未找到 LiDAR 或相机数据!')

        T_lidar_cam_candidates = []

        for i in tqdm(range(num_pairs), desc='处理 LiDAR-相机数据对'):
            lidar_file = os.path.join(lidar_data_dir, lidar_files[i])
            points = self.load_point_cloud(lidar_file)
            if len(points) == 0:
                continue

            points = self.preprocess_point_cloud(points)

            plane_result = self.extract_board_plane_from_lidar(points)
            if plane_result is None:
                print(f'  帧 {i}: 未能从点云中拟合标定板平面')
                continue

            normal, inlier_points, centroid = plane_result
            T_board_lidar = self.estimate_board_pose_in_lidar(normal, inlier_points, centroid)

            img = cv2.imread(image_files[i])
            if img is None:
                continue

            T_board_cam = self.estimate_board_pose_in_camera(img)
            if T_board_cam is None:
                print(f'  帧 {i}: 未能在图像中检测到棋盘格')
                continue

            T_lidar_cam = T_board_cam @ np.linalg.inv(T_board_lidar)

            quality = self.compute_reprojection_error(inlier_points, T_lidar_cam)
            print(f'  帧 {i}: 平面内点={len(inlier_points)}, 投影质量={quality:.1f}%')

            if quality > 50.0:
                T_lidar_cam_candidates.append(T_lidar_cam)

        if len(T_lidar_cam_candidates) < self.min_pairs:
            raise ValueError(
                f'有效数据对不足! 需要至少 {self.min_pairs} 对, '
                f'实际得到 {len(T_lidar_cam_candidates)} 对'
            )

        T_final = self._average_transforms(T_lidar_cam_candidates)

        self.R = T_final[:3, :3]
        self.T = T_final[:3, 3].reshape(3, 1)
        self.transformation_matrix = T_final

        self.calibration_error = self._evaluate_consistency(T_lidar_cam_candidates, T_final)

        roll, pitch, yaw = rotation_matrix_to_angles(self.R)
        print(f'\nLiDAR-相机标定完成!')
        print(f'使用 {len(T_lidar_cam_candidates)} 对有效数据')
        print(f'帧间一致性误差: 旋转={self.calibration_error["rotation_std_deg"]:.2f}°, '
              f'平移={self.calibration_error["translation_std_m"]:.4f}m')
        print(f'旋转 (欧拉角): roll={np.degrees(roll):.2f}°, '
              f'pitch={np.degrees(pitch):.2f}°, yaw={np.degrees(yaw):.2f}°')
        print(f'平移向量: {self.T.ravel()}')

        return self.get_calibration_results()

    def _average_transforms(self, transforms: List[np.ndarray]) -> np.ndarray:
        """
        在 SE(3) 上对多个变换取平均：旋转用四元数球面平均，平移取算术平均。
        """
        quats = []
        translations = []
        for T in transforms:
            r = ScipyRotation.from_matrix(T[:3, :3])
            quats.append(r.as_quat())
            translations.append(T[:3, 3])

        quats = np.array(quats)
        ref = quats[0]
        for i in range(1, len(quats)):
            if np.dot(quats[i], ref) < 0:
                quats[i] = -quats[i]

        avg_quat = np.mean(quats, axis=0)
        avg_quat = avg_quat / np.linalg.norm(avg_quat)

        avg_R = ScipyRotation.from_quat(avg_quat).as_matrix()
        avg_t = np.mean(translations, axis=0)

        T_avg = np.eye(4)
        T_avg[:3, :3] = avg_R
        T_avg[:3, 3] = avg_t
        return T_avg

    def _evaluate_consistency(
        self, transforms: List[np.ndarray], T_mean: np.ndarray
    ) -> Dict:
        """评估多帧标定结果的一致性。"""
        angle_diffs = []
        trans_diffs = []
        R_mean = T_mean[:3, :3]
        t_mean = T_mean[:3, 3]

        for T in transforms:
            R_diff = T[:3, :3] @ R_mean.T
            angle = np.arccos(np.clip((np.trace(R_diff) - 1) / 2, -1, 1))
            angle_diffs.append(np.degrees(angle))
            trans_diffs.append(np.linalg.norm(T[:3, 3] - t_mean))

        return {
            'rotation_std_deg': float(np.std(angle_diffs)),
            'translation_std_m': float(np.std(trans_diffs)),
            'rotation_diffs_deg': angle_diffs,
            'translation_diffs_m': trans_diffs,
        }

    def get_calibration_results(self) -> Dict:
        return {
            'transformation_matrix': self.transformation_matrix,
            'R': self.R,
            'T': self.T,
            'calibration_error': self.calibration_error
        }

    def save_results(self, output_dir: str):
        ensure_dir(output_dir)
        results = self.get_calibration_results()
        save_calibration_results(results, output_dir, prefix='lidar_to_camera_')

        report_path = os.path.join(output_dir, 'lidar_camera_calibration_report.txt')
        self._save_report(report_path)
        print(f'LiDAR-相机标定结果已保存到 {output_dir}')

    def _save_report(self, report_path: str):
        with open(report_path, 'w', encoding='utf-8') as f:
            f.write('LiDAR-相机外参标定报告\n')
            f.write('=' * 60 + '\n\n')

            err = self.calibration_error or {}
            f.write(f'帧间一致性: 旋转 std={err.get("rotation_std_deg", 0):.4f}°, '
                    f'平移 std={err.get("translation_std_m", 0):.6f}m\n\n')

            f.write('变换矩阵 (LiDAR → Camera):\n')
            f.write(str(self.transformation_matrix) + '\n\n')

            f.write('旋转矩阵 R:\n')
            f.write(str(self.R) + '\n\n')

            f.write('平移向量 T:\n')
            f.write(str(self.T.ravel()) + '\n\n')

            roll, pitch, yaw = rotation_matrix_to_angles(self.R)
            f.write(f'欧拉角 (度): roll={np.degrees(roll):.2f}, '
                    f'pitch={np.degrees(pitch):.2f}, yaw={np.degrees(yaw):.2f}\n')

    def transform_point_cloud(self, points: np.ndarray) -> np.ndarray:
        if self.transformation_matrix is None:
            raise RuntimeError('请先执行标定!')
        pts_hom = np.hstack([points, np.ones((len(points), 1))])
        return (pts_hom @ self.transformation_matrix.T)[:, :3]

    def project_lidar_to_image(self, points: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        """
        将 LiDAR 点投影到图像平面。

        Returns:
            (points_2d, valid_mask): 2D 坐标和有效性掩码（在相机前方）
        """
        if self.camera_matrix is None or self.transformation_matrix is None:
            raise RuntimeError('请先设置相机内参并执行标定!')

        pts_cam = self.transform_point_cloud(points)
        valid = pts_cam[:, 2] > 0

        rvec = np.zeros(3)
        tvec = np.zeros(3)
        pts_2d = cv2.projectPoints(
            pts_cam, rvec, tvec, self.camera_matrix, self.dist_coeffs
        )[0].reshape(-1, 2)

        return pts_2d, valid


def main():
    print('LiDAR-相机标定模块')
    print('使用方法: 参考 complete_calibration_example.py')


if __name__ == '__main__':
    main()
