"""
完整的多传感器标定示例脚本

演示如何使用整个标定系统完成6个相机、LiDAR和IMU的标定流程。
"""

import os
import numpy as np
from calibration_utils import ensure_dir, load_config
from camera_calibration import CameraCalibration
from multi_camera_calibration import MultiCameraCalibration
from lidar_camera_calibration import LidarCameraCalibration
from imu_camera_calibration import IMUCameraCalibration


def print_section(title: str):
    """
    打印分隔线
    
    Args:
        title: 章节标题
    """
    print('\n' + '=' * 80)
    print(title.center(80))
    print('=' * 80 + '\n')


def step1_calibrate_all_cameras():
    """
    步骤1: 标定所有相机的内参
    """
    print_section('步骤 1: 标定所有相机内参')
    
    # 创建多相机标定器
    multi_calibrator = MultiCameraCalibration('config.yaml')
    
    # 标定所有相机
    camera_images_dir = 'data/camera_images'
    
    print('注意: 请确保以下目录存在并包含标定图像:')
    for camera_cfg in multi_calibrator.cameras_config:
        camera_name = camera_cfg['name']
        camera_dir = os.path.join(camera_images_dir, camera_name)
        print(f'  - {camera_dir}')
    
    print('\n如果没有真实数据，可以运行 test_camera_calibration.py 生成测试数据\n')
    
    # 执行标定
    results = multi_calibrator.calibrate(camera_images_dir)
    
    # 保存结果
    output_dir = 'data/calibration_results'
    multi_calibrator.save_results(output_dir)
    
    print('\n所有相机内参标定完成!')
    
    return multi_calibrator


def step2_calibrate_lidar_to_camera(multi_calibrator: MultiCameraCalibration):
    """
    步骤2: 标定LiDAR到相机的外参
    
    Args:
        multi_calibrator: 多相机标定器（包含相机内参）
    """
    print_section('步骤 2: 标定LiDAR到相机外参')
    
    # 获取参考相机的内参
    ref_camera = multi_calibrator.reference_camera
    camera_matrix = multi_calibrator.intrinsics[ref_camera]['camera_matrix']
    dist_coeffs = multi_calibrator.intrinsics[ref_camera]['distortion_coeffs']
    
    # 创建LiDAR-相机标定器
    lidar_calibrator = LidarCameraCalibration('config.yaml')
    
    # 执行标定
    lidar_data_dir = 'data/lidar_points'
    camera_image_dir = os.path.join('data/camera_images', ref_camera)
    
    print('注意: 请确保以下目录存在并包含标定数据:')
    print(f'  - {lidar_data_dir} (LiDAR点云文件 .pcd 或 .ply)')
    print(f'  - {camera_image_dir} (相机图像)')
    print('\n数据要求:')
    print('  - LiDAR点云和相机图像需要时间同步')
    print('  - 场景中需要包含AprilTag标定板')
    print('  - 推荐使用4x4 AprilTag网格，间距20cm\n')
    
    try:
        results = lidar_calibrator.calibrate(
            lidar_data_dir,
            camera_image_dir,
            camera_matrix,
            dist_coeffs
        )
        
        # 保存结果
        output_dir = 'data/calibration_results/lidar_camera'
        lidar_calibrator.save_results(output_dir)
        
        print('\nLiDAR-相机外参标定完成!')
        
        return lidar_calibrator
        
    except Exception as e:
        print(f'\n警告: LiDAR-相机标定失败: {e}')
        print('跳过此步骤，继续下一步')
        return None


def step3_calibrate_imu_to_camera(multi_calibrator: MultiCameraCalibration):
    """
    步骤3: 标定IMU到相机的外参
    
    Args:
        multi_calibrator: 多相机标定器（包含相机内参）
    """
    print_section('步骤 3: 标定IMU到相机外参')
    
    # 获取参考相机的内参
    ref_camera = multi_calibrator.reference_camera
    camera_matrix = multi_calibrator.intrinsics[ref_camera]['camera_matrix']
    dist_coeffs = multi_calibrator.intrinsics[ref_camera]['distortion_coeffs']
    
    # 创建IMU-相机标定器
    imu_calibrator = IMUCameraCalibration('config.yaml')
    
    # 执行标定
    imu_data_dir = 'data/imu_data'
    camera_image_dir = os.path.join('data/camera_images', ref_camera)
    
    print('注意: 请确保以下目录存在并包含标定数据:')
    print(f'  - {imu_data_dir} (IMU数据文件 .csv)')
    print(f'  - {camera_image_dir} (相机图像)')
    print('\n数据要求:')
    print('  - IMU数据和相机图像需要时间同步')
    print('  - CSV格式: timestamp, accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z')
    print('  - 标定过程中需要保持设备静止以估计零偏')
    print('  - 需要在多个方向采集数据以覆盖重力方向\n')
    
    try:
        results = imu_calibrator.calibrate(
            imu_data_dir,
            camera_image_dir,
            camera_matrix,
            dist_coeffs
        )
        
        # 保存结果
        output_dir = 'data/calibration_results/imu_camera'
        imu_calibrator.save_results(output_dir)
        
        print('\nIMU-相机外参标定完成!')
        
        return imu_calibrator
        
    except Exception as e:
        print(f'\n警告: IMU-相机标定失败: {e}')
        print('跳过此步骤')
        return None


def print_summary(multi_calibrator, lidar_calibrator, imu_calibrator):
    """
    打印标定结果摘要
    
    Args:
        multi_calibrator: 多相机标定器
        lidar_calibrator: LiDAR-相机标定器
        imu_calibrator: IMU-相机标定器
    """
    print_section('标定结果摘要')
    
    # 相机内参摘要
    print('1. 相机内参标定结果:')
    print('-' * 80)
    for camera_name, intrinsics in multi_calibrator.intrinsics.items():
        error = intrinsics['reprojection_error']
        fx = intrinsics['camera_matrix'][0, 0]
        fy = intrinsics['camera_matrix'][1, 1]
        print(f'  {camera_name:12s}: 误差={error:.4f}px, 焦距=({fx:.0f}, {fy:.0f})')
    
    # 相机外参摘要
    print('\n2. 相机外参标定结果（相对于参考相机）:')
    print('-' * 80)
    from calibration_utils import rotation_matrix_to_angles
    for camera_cfg in multi_calibrator.cameras_config:
        camera_name = camera_cfg['name']
        R, T = multi_calibrator.get_camera_extrinsics(camera_name)
        if R is not None:
            roll, pitch, yaw = rotation_matrix_to_angles(R)
            print(f'  {camera_name:12s}: R=[{np.degrees(roll):6.2f}°, {np.degrees(pitch):6.2f}°, {np.degrees(yaw):6.2f}°], T=[{T[0,0]:7.3f}, {T[1,0]:7.3f}, {T[2,0]:7.3f}]')
    
    # LiDAR-相机外参摘要
    if lidar_calibrator is not None:
        print('\n3. LiDAR-相机外参标定结果:')
        print('-' * 80)
        R, T = lidar_calibrator.R, lidar_calibrator.T
        error = lidar_calibrator.calibration_error
        roll, pitch, yaw = rotation_matrix_to_angles(R)
        print(f'  标定误差: {error:.4f} 像素')
        print(f'  旋转矩阵: roll={np.degrees(roll):.2f}°, pitch={np.degrees(pitch):.2f}°, yaw={np.degrees(yaw):.2f}°')
        print(f'  平移向量: {T.ravel()}')
    
    # IMU-相机外参摘要
    if imu_calibrator is not None:
        print('\n4. IMU-相机外参标定结果:')
        print('-' * 80)
        R, T = imu_calibrator.R, imu_calibrator.T
        error = imu_calibrator.calibration_error
        roll, pitch, yaw = rotation_matrix_to_angles(R)
        print(f'  标定误差: {np.degrees(error):.2f}°')
        print(f'  加速度计零偏: {imu_calibrator.accel_bias}')
        print(f'  陀螺仪零偏: {imu_calibrator.gyro_bias}')
        print(f'  旋转矩阵: roll={np.degrees(roll):.2f}°, pitch={np.degrees(pitch):.2f}°, yaw={np.degrees(yaw):.2f}°')
        print(f'  平移向量: {T.ravel()} (未精确标定)')


def main():
    """
    主函数：执行完整的多传感器标定流程
    """
    print('\n' + '=' * 80)
    print('自动驾驶多传感器标定系统'.center(80))
    print('=' * 80)
    print('\n本脚本演示完整的标定流程:')
    print('  1. 标定6个相机的内参')
    print('  2. 标定LiDAR到相机的外参')
    print('  3. 标定IMU到相机的外参')
    print('\n传感器配置:')
    print('  - 6个相机: front, front_left, front_right, rear_left, rear_right, rear')
    print('  - 1个LiDAR: Velodyne 64线')
    print('  - 1个IMU: 高频惯性测量单元')
    print()
    
    # 加载配置
    config = load_config('config.yaml')
    print(f'参考相机: {config["multi_camera"]["reference_camera"]}')
    print(f'全局优化: {"启用" if config["multi_camera"]["global_optimization"] else "禁用"}')
    
    # 步骤1: 标定所有相机
    try:
        multi_calibrator = step1_calibrate_all_cameras()
    except Exception as e:
        print(f'\n错误: 相机内参标定失败: {e}')
        print('请检查数据目录是否存在并包含有效的标定图像')
        return
    
    # 步骤2: 标定LiDAR到相机
    lidar_calibrator = step2_calibrate_lidar_to_camera(multi_calibrator)
    
    # 步骤3: 标定IMU到相机
    imu_calibrator = step3_calibrate_imu_to_camera(multi_calibrator)
    
    # 打印摘要
    print_summary(multi_calibrator, lidar_calibrator, imu_calibrator)
    
    # 完成
    print_section('标定完成')
    print('所有标定结果已保存到: data/calibration_results/')
    print('\n目录结构:')
    print('  data/calibration_results/')
    print('  ├── intrinsics/                    # 相机内参')
    print('  │   ├── front/')
    print('  │   ├── front_left/')
    print('  │   └── ...')
    print('  ├── extrinsics/                   # 相机外参')
    print('  ├── lidar_camera/                 # LiDAR-相机外参')
    print('  ├── imu_camera/                   # IMU-相机外参')
    print('  └── *.txt                       # 标定报告')
    print()
    print('下一步:')
    print('  1. 检查标定报告中的重投影误差')
    print('  2. 如果误差过大，增加标定图像数量')
    print('  3. 使用可视化工具验证标定结果')
    print('  4. 将标定参数部署到自动驾驶系统')
    print()


if __name__ == '__main__':
    main()