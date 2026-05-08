#!/bin/bash
# =============================================================================
# MINH DELIVERY ROBOT - Full Setup Script for Jetson AGX Xavier
# =============================================================================
# System: Jetson AGX Xavier, JetPack 5.x (L4T R35), Ubuntu 20.04, ROS2 Foxy
# Camera: Intel RealSense D435i (USB-C)
# LiDAR: Hokuyo UTM-30LX (sẽ kết nối sau)
# =============================================================================

set -e

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
CYAN='\033[0;36m'
NC='\033[0m'

ROBOT_DIR="$HOME/ros2_ws/src/minh_delivery_robot"
ROS2_WS="$ROBOT_DIR/ros2_ws"

log_info()  { echo -e "${CYAN}[INFO]${NC} $1"; }
log_ok()    { echo -e "${GREEN}[OK]${NC} $1"; }
log_warn()  { echo -e "${YELLOW}[WARN]${NC} $1"; }
log_error() { echo -e "${RED}[ERROR]${NC} $1"; }

check_disk_space() {
    local avail_kb
    avail_kb=$(df --output=avail / | tail -1 | tr -d ' ')
    local avail_gb=$((avail_kb / 1024 / 1024))
    echo "$avail_gb"
}

# =============================================================================
echo ""
echo "============================================================"
echo "  MINH DELIVERY ROBOT - Jetson AGX Xavier Setup"
echo "============================================================"
echo ""
echo "Hệ thống sẽ cài đặt:"
echo "  1. Dọn dẹp ổ đĩa (giải phóng dung lượng)"
echo "  2. RealSense SDK (librealsense2) + ROS2 driver"
echo "  3. Hokuyo LiDAR driver (urg_node2)"
echo "  4. PyTorch (GPU/CUDA) cho Jetson"
echo "  5. Ultralytics (YOLOv8)"
echo "  6. FastAPI web app dependencies"
echo "  7. Build ROS2 package realsense_yolo"
echo "  8. Cấu hình permissions (dialout group)"
echo ""
echo "Disk space hiện tại: $(check_disk_space) GB free"
echo ""
read -p "Tiếp tục? (y/n): " -n 1 -r
echo ""
if [[ ! $REPLY =~ ^[Yy]$ ]]; then
    echo "Đã hủy."
    exit 0
fi

# =============================================================================
# PHASE 1: Disk Cleanup
# =============================================================================
echo ""
echo "============================================================"
echo "  PHASE 1: Dọn dẹp ổ đĩa"
echo "============================================================"

log_info "Xóa apt cache..."
sudo apt clean

log_info "Xóa packages không dùng..."
sudo apt autoremove -y

log_info "Xóa journal logs cũ (giữ 50MB)..."
sudo journalctl --vacuum-size=50M 2>/dev/null || true

log_info "Xóa pip cache..."
pip3 cache purge 2>/dev/null || rm -rf ~/.cache/pip 2>/dev/null || true

# Xóa file .deb không dùng trong Downloads
if [ -f "$HOME/Downloads/google-chrome-stable_current_amd64.deb" ]; then
    log_info "Xóa google-chrome deb (amd64 - không dùng được trên arm64)..."
    rm -f "$HOME/Downloads/google-chrome-stable_current_amd64.deb"
fi

log_ok "Disk space sau dọn dẹp: $(check_disk_space) GB free"
echo ""

# =============================================================================
# PHASE 2: Install RealSense SDK + ROS2 Driver (via apt)
# =============================================================================
echo "============================================================"
echo "  PHASE 2: Cài đặt RealSense SDK + ROS2 Driver"
echo "============================================================"

log_info "Cài đặt ros-foxy-librealsense2 (SDK cho D435i)..."
sudo apt update
sudo apt install -y ros-foxy-librealsense2

log_info "Cài đặt ros-foxy-realsense2-camera + messages..."
sudo apt install -y \
    ros-foxy-realsense2-camera \
    ros-foxy-realsense2-camera-msgs \
    ros-foxy-realsense2-description

log_info "Cài đặt ROS2 dependencies bổ sung..."
sudo apt install -y \
    ros-foxy-diagnostic-updater \
    ros-foxy-nav-msgs \
    ros-foxy-rclcpp-components \
    ros-foxy-console-bridge-vendor \
    ros-foxy-tf2 \
    ros-foxy-lifecycle-msgs

log_ok "RealSense SDK + ROS2 driver đã cài xong!"
echo ""

# =============================================================================
# PHASE 3: Install urg_node2 (LiDAR driver)
# =============================================================================
echo "============================================================"
echo "  PHASE 3: Cài đặt urg_node2 (Hokuyo LiDAR driver)"
echo "============================================================"

log_info "Cài đặt urg_node2 dependencies (urg_c library)..."
sudo apt install -y ros-foxy-urg-c 2>/dev/null || true

cd "$ROS2_WS/src"

if [ ! -d "urg_node2" ]; then
    log_info "Clone urg_node2 từ GitHub..."
    git clone https://github.com/Hokuyo-aut/urg_node2.git
else
    log_info "urg_node2 đã tồn tại, pull latest..."
    cd urg_node2 && git pull && cd ..
fi

log_info "Cài đặt urg_node2 rosdep dependencies..."
source /opt/ros/foxy/setup.bash
rosdep install -i --from-paths "$ROS2_WS/src/urg_node2" --rosdistro foxy -y 2>/dev/null || true

log_info "Build urg_node2..."
cd "$ROS2_WS"
source /opt/ros/foxy/setup.bash
colcon build --packages-select urg_node2 --cmake-args -DCMAKE_BUILD_TYPE=Release 2>&1 | tail -5

if [ $? -eq 0 ]; then
    log_ok "urg_node2 build thành công!"
else
    log_warn "urg_node2 build thất bại - sẽ thử lại sau khi cài thêm dependencies"
fi

# Add user to dialout group for serial port access (LiDAR)
if ! groups "$USER" | grep -q dialout; then
    log_info "Thêm user vào group dialout (cho serial port LiDAR)..."
    sudo usermod -aG dialout "$USER"
    log_warn "Cần logout/login lại để dialout group có hiệu lực!"
fi

echo ""

# =============================================================================
# PHASE 4: Install PyTorch for Jetson (CUDA enabled)
# =============================================================================
echo "============================================================"
echo "  PHASE 4: Cài đặt PyTorch cho Jetson (GPU/CUDA)"
echo "============================================================"

log_info "Kiểm tra disk space trước khi cài PyTorch..."
DISK_FREE=$(check_disk_space)
if [ "$DISK_FREE" -lt 3 ]; then
    log_error "Không đủ dung lượng ($DISK_FREE GB free). Cần ít nhất 3GB."
    log_warn "Hãy giải phóng thêm dung lượng rồi chạy lại script."
    log_warn "Bỏ qua cài PyTorch, tiếp tục các bước khác..."
    SKIP_PYTORCH=true
else
    SKIP_PYTORCH=false
fi

if [ "$SKIP_PYTORCH" = false ]; then
    log_info "Upgrade pip..."
    pip3 install --upgrade pip 2>/dev/null || python3 -m pip install --upgrade pip

    log_info "Cài đặt dependencies cho PyTorch..."
    sudo apt install -y libopenblas-base libopenmpi-dev libomp-dev 2>/dev/null || true

    # Check if PyTorch already installed
    if python3 -c "import torch; print(torch.__version__)" 2>/dev/null; then
        log_ok "PyTorch đã được cài đặt!"
    else
        PYTORCH_URL="https://developer.download.nvidia.com/compute/redist/jp/v512/pytorch/torch-2.1.0a0+41361538.nv23.06-cp38-cp38-linux_aarch64.whl"
        log_info "Tải PyTorch 2.1 cho JetPack 5.x..."
        log_info "URL: $PYTORCH_URL"
        pip3 install --no-cache-dir "$PYTORCH_URL"

        if python3 -c "import torch; print('PyTorch', torch.__version__); print('CUDA:', torch.cuda.is_available())" 2>/dev/null; then
            log_ok "PyTorch cài thành công với CUDA support!"
        else
            log_error "PyTorch cài thất bại. Kiểm tra lại."
        fi
    fi

    # Install torchvision (build from source for Jetson)
    log_info "Cài đặt torchvision dependencies..."
    sudo apt install -y libjpeg-dev zlib1g-dev libpython3-dev libavcodec-dev libavformat-dev libswscale-dev 2>/dev/null || true

    if python3 -c "import torchvision" 2>/dev/null; then
        log_ok "torchvision đã được cài đặt!"
    else
        log_info "Build torchvision 0.16.0 từ source (mất ~10-20 phút trên Xavier)..."
        cd /tmp
        if [ ! -d "torchvision" ]; then
            git clone --branch v0.16.0 --depth 1 https://github.com/pytorch/vision torchvision
        fi
        cd torchvision
        export BUILD_VERSION=0.16.0
        python3 setup.py install --user 2>&1 | tail -10

        if python3 -c "import torchvision; print('torchvision', torchvision.__version__)" 2>/dev/null; then
            log_ok "torchvision build thành công!"
        else
            log_warn "torchvision build thất bại. Ultralytics vẫn chạy được nhưng thiếu vài feature."
        fi
        cd "$ROBOT_DIR"
        rm -rf /tmp/torchvision
    fi
fi

echo ""

# =============================================================================
# PHASE 5: Install Python packages
# =============================================================================
echo "============================================================"
echo "  PHASE 5: Cài đặt Python packages"
echo "============================================================"

log_info "Upgrade numpy..."
pip3 install --upgrade numpy 2>/dev/null || pip3 install numpy

log_info "Cài đặt ultralytics (YOLOv8)..."
pip3 install ultralytics

log_info "Cài đặt FastAPI web app dependencies..."
pip3 install -r "$ROBOT_DIR/requirements.txt"

log_ok "Python packages đã cài xong!"
echo ""

# =============================================================================
# PHASE 6: Build realsense_yolo ROS2 package
# =============================================================================
echo "============================================================"
echo "  PHASE 6: Build realsense_yolo ROS2 package"
echo "============================================================"

cd "$ROS2_WS"
source /opt/ros/foxy/setup.bash

# Source previously built packages if any
if [ -f "$ROS2_WS/install/setup.bash" ]; then
    source "$ROS2_WS/install/setup.bash"
fi

log_info "Build realsense_yolo..."
colcon build --packages-select realsense_yolo 2>&1 | tail -10

if [ $? -eq 0 ]; then
    log_ok "realsense_yolo build thành công!"
else
    log_error "realsense_yolo build thất bại. Kiểm tra errors ở trên."
fi

source "$ROS2_WS/install/setup.bash"

echo ""

# =============================================================================
# PHASE 7: Verification
# =============================================================================
echo "============================================================"
echo "  PHASE 7: Kiểm tra cài đặt"
echo "============================================================"

echo ""
echo "--- ROS2 Packages ---"
PACKAGES=("realsense2_camera" "realsense2_camera_msgs" "realsense_yolo" "urg_node2" "cv_bridge" "tf2_ros" "rviz2")
for pkg in "${PACKAGES[@]}"; do
    if ros2 pkg list 2>/dev/null | grep -q "^${pkg}$"; then
        echo -e "  ${GREEN}✓${NC} $pkg"
    else
        echo -e "  ${RED}✗${NC} $pkg"
    fi
done

echo ""
echo "--- Python Packages ---"
PY_PACKAGES=("torch" "torchvision" "ultralytics" "numpy" "cv2" "fastapi")
for pkg in "${PY_PACKAGES[@]}"; do
    ver=$(python3 -c "import $pkg; print($pkg.__version__)" 2>/dev/null)
    if [ -n "$ver" ]; then
        echo -e "  ${GREEN}✓${NC} $pkg ($ver)"
    else
        echo -e "  ${RED}✗${NC} $pkg"
    fi
done

echo ""
echo "--- Hardware ---"
# Check RealSense
if cat /sys/bus/usb/devices/*/product 2>/dev/null | grep -q "RealSense"; then
    echo -e "  ${GREEN}✓${NC} RealSense D435i detected via USB"
else
    echo -e "  ${RED}✗${NC} RealSense D435i NOT detected via USB"
fi

# Check CUDA
if python3 -c "import torch; assert torch.cuda.is_available()" 2>/dev/null; then
    echo -e "  ${GREEN}✓${NC} CUDA available for PyTorch"
else
    echo -e "  ${YELLOW}?${NC} CUDA not available (PyTorch may use CPU)"
fi

# Check video devices
if ls /dev/video* 2>/dev/null | head -1 > /dev/null; then
    echo -e "  ${GREEN}✓${NC} Video devices: $(ls /dev/video* 2>/dev/null | tr '\n' ' ')"
else
    echo -e "  ${YELLOW}?${NC} No /dev/video* (RealSense sẽ tạo sau khi chạy driver)"
fi

# Check serial ports
if ls /dev/ttyACM* 2>/dev/null | head -1 > /dev/null; then
    echo -e "  ${GREEN}✓${NC} Serial ports: $(ls /dev/ttyACM* 2>/dev/null | tr '\n' ' ')"
else
    echo -e "  ${YELLOW}?${NC} No /dev/ttyACM* (LiDAR chưa kết nối)"
fi

# Check dialout group
if groups "$USER" | grep -q dialout; then
    echo -e "  ${GREEN}✓${NC} User in dialout group"
else
    echo -e "  ${YELLOW}!${NC} User NOT in dialout group (cần logout/login)"
fi

echo ""
echo "--- Disk Space ---"
df -h / | tail -1

echo ""
echo "============================================================"
echo "  SETUP HOÀN TẤT!"
echo "============================================================"
echo ""
echo "Cách sử dụng:"
echo ""
echo "  # Chạy YOLO only (camera only, không cần LiDAR):"
echo "  cd $ROS2_WS"
echo "  source install/setup.bash"
echo "  ros2 launch realsense_yolo realsense_yolo_view.launch.py"
echo ""
echo "  # Chạy Fusion 3D (camera + LiDAR + RViz):"
echo "  ros2 launch realsense_yolo fusion_3d.launch.py"
echo ""
echo "  # Chạy FastAPI web app:"
echo "  cd $ROBOT_DIR"
echo "  python3 -m uvicorn app.main:app --host 0.0.0.0 --port 8000"
echo ""
echo "Lưu ý:"
echo "  - Nếu vừa thêm dialout group: logout rồi login lại"
echo "  - Model YOLOv8 (yolov8n.pt) sẽ tự tải lần đầu chạy"
echo "  - Không có LiDAR: launch file sẽ tự skip urg_node2"
echo "  - Camera USB 2.0: dùng 640x480 @ 15fps (đã là mặc định)"
echo ""
