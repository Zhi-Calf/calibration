"""
生成高分辨率标定板图像

生成高分辨率的棋盘格和AprilTag标定板
"""

import cv2
import numpy as np
import os
from pathlib import Path


def generate_checkerboard(output_path: str, 
                       pattern_size: tuple = (9, 6),
                       square_size_mm: float = 20.0,
                       dpi: int = 600,
                       paper_size: str = 'A4'):
    """
    生成高分辨率棋盘格图像
    
    Args:
        output_path: 输出文件路径
        pattern_size: 棋盘格内角点数量 (columns, rows)
        square_size_mm: 方块实际尺寸（毫米）
        dpi: 打印分辨率（每英寸点数）
        paper_size: 纸张大小 (A4, A3等）
    """
    # 纸张尺寸（英寸）
    paper_sizes = {
        'A4': (11.69, 8.27),   # 297mm x 210mm
        'A3': (16.54, 11.69),  # 420mm x 297mm
        'Letter': (11.0, 8.5),
        'Legal': (14.0, 8.5)
    }
    
    paper_width_inch, paper_height_inch = paper_sizes.get(paper_size, paper_sizes['A4'])
    
    # 图像尺寸（像素）
    img_width = int(paper_width_inch * dpi)
    img_height = int(paper_height_inch * dpi)
    
    # 方块尺寸（像素）
    square_size_inch = square_size_mm / 25.4
    square_size_px = int(square_size_inch * dpi)
    
    # 棋盘格尺寸
    num_squares_x = pattern_size[0] + 1
    num_squares_y = pattern_size[1] + 1
    
    board_width_px = num_squares_x * square_size_px
    board_height_px = num_squares_y * square_size_px
    
    # 创建空白图像
    img = np.ones((img_height, img_width), dtype=np.uint8) * 255
    
    # 计算居中位置
    start_x = (img_width - board_width_px) // 2
    start_y = (img_height - board_height_px) // 2
    
    # 绘制棋盘格
    for row in range(num_squares_y):
        for col in range(num_squares_x):
            x1 = start_x + col * square_size_px
            y1 = start_y + row * square_size_px
            x2 = x1 + square_size_px
            y2 = y1 + square_size_px
            
            # 确保在图像范围内
            if x2 > img_width or y2 > img_height:
                continue
            
            # 根据位置决定颜色
            if (row + col) % 2 == 0:
                color = 0  # 黑色
            else:
                color = 255  # 白色
            
            cv2.rectangle(img, (x1, y1), (x2, y2), color, -1)
    
    # 保存图像
    cv2.imwrite(output_path, img)
    print(f'✓ 生成棋盘格: {output_path}')
    print(f'  - 图像尺寸: {img_width} x {img_height} 像素')
    print(f'  - 棋盘格: {num_squares_x}x{num_squares_y} 方块')
    print(f'  - 方块尺寸: {square_size_mm}mm ({square_size_px}px)')
    print(f'  - 分辨率: {dpi} DPI')


def generate_apriltag_board(output_path: str,
                          tag_size_mm: float = 160.0,
                          spacing_mm: float = 200.0,
                          grid_size: tuple = (4, 4),
                          dpi: int = 600):
    """
    生成高分辨率AprilTag标定板
    
    注意：这里生成的是标定板的布局，实际的AprilTag图像需要从apriltag库生成
    
    Args:
        output_path: 输出文件路径
        tag_size_mm: 标签尺寸（毫米）
        spacing_mm: 标签间距（毫米）
        grid_size: 网格大小 (rows, cols)
        dpi: 打印分辨率
    """
    # 纸张尺寸（使用A3）
    paper_width_inch, paper_height_inch = 16.54, 11.69
    img_width = int(paper_width_inch * dpi)
    img_height = int(paper_height_inch * dpi)
    
    # 标签尺寸和间距（像素）
    tag_size_inch = tag_size_mm / 25.4
    spacing_inch = spacing_mm / 25.4
    tag_size_px = int(tag_size_inch * dpi)
    spacing_px = int(spacing_inch * dpi)
    
    # 创建空白图像
    img = np.ones((img_height, img_width), dtype=np.uint8) * 255
    
    # 计算网格总尺寸
    grid_width_px = grid_size[1] * tag_size_px + (grid_size[1] - 1) * spacing_px
    grid_height_px = grid_size[0] * tag_size_px + (grid_size[0] - 1) * spacing_px
    
    # 计算居中位置
    start_x = (img_width - grid_width_px) // 2
    start_y = (img_height - grid_height_px) // 2
    
    # 绘制标签位置（黑色方块作为占位符）
    for row in range(grid_size[0]):
        for col in range(grid_size[1]):
            x1 = start_x + col * (tag_size_px + spacing_px)
            y1 = start_y + row * (tag_size_px + spacing_px)
            x2 = x1 + tag_size_px
            y2 = y1 + tag_size_px
            
            # 绘制黑色方块（标签占位符）
            cv2.rectangle(img, (x1, y1), (x2, y2), 0, -1)
            
            # 添加标签ID文本
            tag_id = row * grid_size[1] + col
            font = cv2.FONT_HERSHEY_SIMPLEX
            font_scale = 2.0
            text = f'Tag {tag_id}'
            text_size = cv2.getTextSize(text, font, font_scale, 2)[0]
            text_x = x1 + (tag_size_px - text_size[0]) // 2
            text_y = y1 + (tag_size_px + text_size[1]) // 2
            cv2.putText(img, text, (text_x, text_y), font, font_scale, 255, 2)
    
    # 保存图像
    cv2.imwrite(output_path, img)
    print(f'✓ 生成AprilTag布局: {output_path}')
    print(f'  - 图像尺寸: {img_width} x {img_height} 像素')
    print(f'  - 网格: {grid_size[0]}x{grid_size[1]} 标签')
    print(f'  - 标签尺寸: {tag_size_mm}mm ({tag_size_px}px)')
    print(f'  - 标签间距: {spacing_mm}mm ({spacing_px}px)')
    print(f'  - 分辨率: {dpi} DPI')
    print(f'  注意: 这只是布局图，实际标签需要从apriltag库生成')


def generate_all_patterns():
    """
    生成所有高分辨率标定板
    """
    print('=' * 80)
    print('高分辨率标定板生成器'.center(80))
    print('=' * 80)
    print()
    
    # 创建输出目录
    output_dir = 'calibration_patterns/high_res'
    os.makedirs(output_dir, exist_ok=True)
    
    print(f'输出目录: {output_dir}/')
    print()
    
    # 1. 标准棋盘格 (9x6, 20mm, A4)
    print('1. 生成标准棋盘格 (9x6, 20mm, A4)')
    print('-' * 80)
    generate_checkerboard(
        os.path.join(output_dir, 'checkerboard_9x6_20mm_600dpi.png'),
        pattern_size=(9, 6),
        square_size_mm=20.0,
        dpi=600,
        paper_size='A4'
    )
    print()
    
    # 2. 高精度棋盘格 (12x8, 30mm, A3)
    print('2. 生成高精度棋盘格 (12x8, 30mm, A3)')
    print('-' * 80)
    generate_checkerboard(
        os.path.join(output_dir, 'checkerboard_12x8_30mm_600dpi.png'),
        pattern_size=(12, 8),
        square_size_mm=30.0,
        dpi=600,
        paper_size='A3'
    )
    print()
    
    # 3. 超高精度棋盘格 (14x10, 25mm, A3)
    print('3. 生成超高精度棋盘格 (14x10, 25mm, A3)')
    print('-' * 80)
    generate_checkerboard(
        os.path.join(output_dir, 'checkerboard_14x10_25mm_1200dpi.png'),
        pattern_size=(14, 10),
        square_size_mm=25.0,
        dpi=1200,
        paper_size='A3'
    )
    print()
    
    # 4. AprilTag标定板布局
    print('4. 生成AprilTag标定板布局 (4x4, 160mm标签, 200mm间距)')
    print('-' * 80)
    generate_apriltag_board(
        os.path.join(output_dir, 'apriltag_layout_4x4_600dpi.png'),
        tag_size_mm=160.0,
        spacing_mm=200.0,
        grid_size=(4, 4),
        dpi=600
    )
    print()
    
    # 总结
    print('=' * 80)
    print('生成完成'.center(80))
    print('=' * 80)
    print()
    print(f'所有标定板已保存到: {os.path.abspath(output_dir)}/')
    print()
    print('打印建议:')
    print('1. 使用高分辨率打印机（推荐600 DPI或更高）')
    print('2. 使用高质量相纸或厚纸')
    print('3. 打印后精确测量实际尺寸')
    print('4. 将打印的标定板贴在平整的硬板上')
    print('5. 避免使用反光材料')
    print('6. 确保标定板平整不弯曲')
    print()
    print('对于AprilTag标定板:')
    print('- 使用生成的布局图作为参考')
    print('- 从apriltag库下载或生成实际的标签图像')
    print('- 按照布局图将标签贴在标定板上')
    print('- 推荐使用tag36h11家族')
    print()


if __name__ == '__main__':
    generate_all_patterns()