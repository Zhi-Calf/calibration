#!/bin/bash
# Test rosbag2 parsing in WSL (e.g. bag on D:\ -> /mnt/d/).
# Usage: ./test_rosbag2_wsl.sh [bag_path]
# See calibration/README.md "从 ROS 2 Bag 提取数据".
BAG="${1:-/mnt/d/rosbag2_2026_03_12-16_58_58}"
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
cd "$SCRIPT_DIR"
echo "=== Testing rosbag2: $BAG ==="
echo ""
echo "=== List topics ==="
python3 extract_bag_data.py "$BAG" --list-topics
echo ""
echo "=== Extract test (max 2 images, 2 lidar) ==="
OUT="/tmp/rosbag2_extract_test"
python3 extract_bag_data.py "$BAG" -o "$OUT" --max-images 2 --max-lidar 2
echo ""
echo "=== Output: $OUT ==="
ls -la "$OUT" 2>/dev/null && ls -la "$OUT"/camera_images/*/ 2>/dev/null; ls -la "$OUT"/lidar_points/ 2>/dev/null
echo "Done."
