"""
标定板下载脚本

自动下载标定板图像文件到本地
"""

import os
import requests
from pathlib import Path
from urllib.parse import urlparse


def download_file(url: str, output_path: str):
    """
    下载文件
    
    Args:
        url: 文件URL
        output_path: 输出文件路径
    """
    print(f'正在下载: {url}')
    
    try:
        # 发送HTTP请求
        response = requests.get(url, timeout=30)
        response.raise_for_status()
        
        # 创建输出目录
        os.makedirs(os.path.dirname(output_path), exist_ok=True)
        
        # 保存文件
        with open(output_path, 'wb') as f:
            f.write(response.content)
        
        print(f'✓ 保存到: {output_path}')
        return True
        
    except requests.exceptions.RequestException as e:
        print(f'✗ 下载失败: {e}')
        return False
    except Exception as e:
        print(f'✗ 保存失败: {e}')
        return False


def main():
    """
    主函数：下载所有标定板
    """
    print('=' * 80)
    print('标定板下载工具'.center(80))
    print('=' * 80)
    print()
    print('⚠️  重要提示：')
    print('   - 在线下载的标定板分辨率较低')
    print('   - 推荐使用 generate_high_res_patterns.py 生成高分辨率标定板')
    print('   - 在线下载的标定板仅适用于测试')
    print()
    
    # 创建输出目录
    output_dir = 'calibration_patterns'
    os.makedirs(output_dir, exist_ok=True)
    
    print(f'标定板将保存到: {output_dir}/')
    print()
    
    # 标定板列表
    calibration_patterns = [
        {
            'name': '标准棋盘格 (9x6, 20mm)',
            'url': 'https://opencv.org/wp-content/uploads/2020/04/camera_calibration_pattern.png',
            'filename': 'checkerboard_9x6_20mm.png',
            'description': '用于相机内参标定，打印在A4纸上'
        },
        {
            'name': '高精度棋盘格 (12x8, 30mm)',
            'url': 'https://raw.githubusercontent.com/opencv/opencv/master/doc/pattern.png',
            'filename': 'checkerboard_12x8_30mm.png',
            'description': '用于高精度相机标定，打印在A3纸上'
        },
        {
            'name': 'AprilTag 36h11 (Tag 0)',
            'url': 'https://github.com/AprilRobotics/apriltag-imgs/raw/master/tag36h11/tag36_11_00000.png',
            'filename': 'apriltag_36h11_tag0.png',
            'description': '用于LiDAR-相机标定，需要打印多个并组成4x4网格'
        }
    ]
    
    # 下载标定板
    success_count = 0
    for i, pattern in enumerate(calibration_patterns, 1):
        print(f'\n[{i}/{len(calibration_patterns)}] {pattern["name"]}')
        print(f'说明: {pattern["description"]}')
        
        output_path = os.path.join(output_dir, pattern['filename'])
        
        if download_file(pattern['url'], output_path):
            success_count += 1
    
    # 总结
    print()
    print('=' * 80)
    print('下载完成'.center(80))
    print('=' * 80)
    print()
    print(f'成功下载: {success_count}/{len(calibration_patterns)} 个文件')
    print(f'保存位置: {os.path.abspath(output_dir)}/')
    print()
    
    if success_count > 0:
        print('下一步:')
        print('1. 检查下载的图像文件')
        print('2. 使用高分辨率打印机打印（至少600dpi）')
        print('3. 将打印的标定板贴在平整的硬板上')
        print('4. 避免使用反光材料')
        print('5. 精确测量实际方块尺寸并更新config.yaml')
        print()
    else:
        print('警告: 所有文件下载失败，请检查网络连接')
        print('您也可以手动下载:')
        for pattern in calibration_patterns:
            print(f'  - {pattern["url"]}')
        print()


def download_additional_apriltags():
    """
    下载额外的AprilTag标签（可选）
    
    对于4x4的AprilTag网格，需要下载16个标签（tag36_11_00000 到 tag36_11_00015）
    """
    print('\n' + '=' * 80)
    print('下载额外AprilTag标签（可选）'.center(80))
    print('=' * 80)
    print()
    
    print('注意: 这将下载16个AprilTag标签，用于组成4x4网格')
    print('按Ctrl+C可以取消下载\n')
    
    output_dir = 'calibration_patterns/apriltags'
    os.makedirs(output_dir, exist_ok=True)
    
    base_url = 'https://github.com/AprilRobotics/apriltag-imgs/raw/master/tag36h11/'
    
    success_count = 0
    for i in range(16):
        tag_id = i
        filename = f'tag36_11_{tag_id:05d}.png'
        url = f'{base_url}{filename}'
        output_path = os.path.join(output_dir, filename)
        
        if download_file(url, output_path):
            success_count += 1
    
    print()
    print(f'成功下载: {success_count}/16 个AprilTag标签')
    print(f'保存位置: {os.path.abspath(output_dir)}/')
    print()


if __name__ == '__main__':
    import sys
    
    # 下载主要标定板
    main()
    
    # 询问是否下载额外的AprilTag
    print('是否下载额外的AprilTag标签（16个标签组成4x4网格）？')
    print('输入 y 下载，其他键跳过: ', end='')
    
    try:
        choice = input().strip().lower()
        if choice == 'y':
            download_additional_apriltags()
    except KeyboardInterrupt:
        print('\n\n用户取消，退出程序')
        sys.exit(0)
    except Exception as e:
        print(f'\n\n错误: {e}')
        sys.exit(1)