#!/bin/bash
# =============================================================================
# SUDO COMMANDS - Chạy script này với: sudo bash sudo_setup.sh
# =============================================================================
# Các lệnh cần quyền root để hoàn tất setup Minh Delivery Robot
# trên Jetson AGX Xavier.
# =============================================================================

set -e

GREEN='\033[0;32m'
CYAN='\033[0;36m'
NC='\033[0m'

echo ""
echo "============================================================"
echo "  MINH DELIVERY ROBOT - Sudo Setup Commands"
echo "============================================================"
echo ""

# --- Step 1: Clean disk ---
echo -e "${CYAN}[1/6]${NC} Dọn dẹp ổ đĩa..."
apt clean
apt autoremove -y
journalctl --vacuum-size=50M 2>/dev/null || true
echo -e "${GREEN}Done!${NC}"
echo ""

# --- Step 2: Install libopenblas (PyTorch dependency) ---
echo -e "${CYAN}[2/6]${NC} Cài libopenblas + openmpi (cho PyTorch)..."
apt install -y libopenblas-base libopenblas-dev libopenmpi-dev libomp-dev
echo -e "${GREEN}Done!${NC}"
echo ""

# --- Step 3: Install RealSense SDK + ROS2 driver ---
echo -e "${CYAN}[3/6]${NC} Cài RealSense SDK + ROS2 driver..."
apt update
apt install -y \
    ros-foxy-librealsense2 \
    ros-foxy-realsense2-camera \
    ros-foxy-realsense2-camera-msgs \
    ros-foxy-realsense2-description
echo -e "${GREEN}Done!${NC}"
echo ""

# --- Step 4: Install urg_node2 dependencies ---
echo -e "${CYAN}[4/6]${NC} Cài dependencies cho urg_node2 (LiDAR driver)..."
apt install -y \
    ros-foxy-diagnostic-updater \
    ros-foxy-diagnostic-msgs \
    ros-foxy-urg-c
echo -e "${GREEN}Done!${NC}"
echo ""

# --- Step 5: Install extra media libraries ---
echo -e "${CYAN}[5/6]${NC} Cài thư viện media (cho torchvision)..."
apt install -y \
    libjpeg-dev \
    zlib1g-dev \
    libpython3-dev \
    libavcodec-dev \
    libavformat-dev \
    libswscale-dev
echo -e "${GREEN}Done!${NC}"
echo ""

# --- Step 6: Add user to dialout group ---
echo -e "${CYAN}[6/6]${NC} Thêm user vào dialout group (cho LiDAR serial port)..."
usermod -aG dialout delivery
echo -e "${GREEN}Done! Cần logout/login lại để có hiệu lực.${NC}"
echo ""

echo "============================================================"
echo "  SUDO SETUP HOÀN TẤT!"
echo "============================================================"
echo ""
echo "Bước tiếp theo (chạy như user delivery, KHÔNG cần sudo):"
echo ""
echo "  # 1. Build urg_node2:"
echo "  cd ~/ros2_ws/src/minh_delivery_robot/ros2_ws"
echo "  source /opt/ros/foxy/setup.bash"
echo "  colcon build --packages-select urg_node2"
echo ""
echo "  # 2. Cài lại PyTorch Jetson (nếu muốn CUDA/GPU):"
echo "  pip3 install --user --no-cache-dir --force-reinstall \\"
echo "    'https://developer.download.nvidia.com/compute/redist/jp/v512/pytorch/torch-2.1.0a0+41361538.nv23.06-cp38-cp38-linux_aarch64.whl'"
echo ""
echo "  # 3. Test launch:"
echo "  cd ~/ros2_ws/src/minh_delivery_robot/ros2_ws"
echo "  source install/setup.bash"
echo "  ros2 launch realsense_yolo realsense_yolo_view.launch.py"
echo ""
