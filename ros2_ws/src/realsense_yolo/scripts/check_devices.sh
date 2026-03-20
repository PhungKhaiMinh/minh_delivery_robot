#!/bin/bash
# Kiểm tra kết nối RealSense D435i và 2D LiDAR (Hokuyo UTM-30LX)
# Chạy: ./check_devices.sh hoặc bash check_devices.sh

set -e
echo "=============================================="
echo "  Kiểm tra thiết bị RealSense + LiDAR"
echo "=============================================="
echo ""

echo "--- 1. USB thiết bị (lsusb) ---"
if command -v lsusb &>/dev/null; then
  lsusb | grep -E "Intel|RealSense|Hokuyo|Prolific|FTDI|CP210|CH340|UART" || true
  echo "(Nếu không thấy: Intel = RealSense, Hokuyo/Prolific/FTDI = LiDAR adapter)"
else
  echo "lsusb không có. Cài: sudo apt install usbutils"
fi
echo ""

echo "--- 2. Cổng serial (LiDAR Hokuyo thường dùng ttyUSB* hoặc ttyACM*) ---"
for d in /dev/ttyUSB* /dev/ttyACM* 2>/dev/null; do
  [ -e "$d" ] || continue
  echo "  Tìm thấy: $d"
  ls -la "$d" 2>/dev/null || true
done
if ! compgen -G /dev/ttyUSB* >/dev/null 2>&1 && ! compgen -G /dev/ttyACM* >/dev/null; then
  echo "  Không tìm thấy /dev/ttyUSB* hoặc /dev/ttyACM*. Kiểm tra quyền: sudo usermod -aG dialout \$USER (sau đó đăng xuất/đăng nhập lại)."
fi
echo ""

echo "--- 3. Video (RealSense D435i) ---"
for d in /dev/video* 2>/dev/null; do
  [ -e "$d" ] || continue
  echo "  $d"
done
if ! compgen -G /dev/video* >/dev/null 2>&1; then
  echo "  Không tìm thấy /dev/video*. Kiểm tra cáp USB và quyền."
fi
echo ""

echo "--- 4. Kiểm tra ROS2 topics (cần chạy camera + lidar trước) ---"
if [ -n "$ROS_DISTRO" ] && command -v ros2 &>/dev/null; then
  echo "  Các topic đang có:"
  ros2 topic list 2>/dev/null | grep -E "scan|camera|realsense_yolo|depth" || true
  echo ""
  echo "  Gợi ý: Nếu chưa có /scan, cài LiDAR driver: sudo apt install ros-foxy-urg-node"
  echo "  Sau đó: ros2 launch urg_node urg_node_launch.py (hoặc dùng launch fusion bên dưới)"
else
  echo "  Chưa source ROS2 hoặc chưa chạy node. Source: source /opt/ros/foxy/setup.bash"
fi
echo ""
echo "=============================================="
echo "  Kết thúc kiểm tra"
echo "=============================================="
