#!/bin/bash
# =============================================================================
# POST-SUDO SETUP - Chạy SAU KHI đã chạy sudo_setup.sh
# =============================================================================
# Không cần quyền root. Build urg_node2 + reinstall Jetson PyTorch + verify.
# =============================================================================

set -e

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
CYAN='\033[0;36m'
NC='\033[0m'

export PATH="$HOME/.local/bin:$PATH"
ROS2_WS="$HOME/ros2_ws/src/minh_delivery_robot/ros2_ws"

echo ""
echo "============================================================"
echo "  POST-SUDO SETUP"
echo "============================================================"
echo ""

# --- Step 1: Build urg_node2 ---
echo -e "${CYAN}[1/4]${NC} Build urg_node2..."
cd "$ROS2_WS"
source /opt/ros/foxy/setup.bash
colcon build --packages-select urg_node2 --cmake-args -DCMAKE_BUILD_TYPE=Release 2>&1 | tail -5
echo ""

# --- Step 2: Rebuild realsense_yolo (in case anything changed) ---
echo -e "${CYAN}[2/4]${NC} Rebuild realsense_yolo..."
source install/setup.bash 2>/dev/null || true
colcon build --packages-select realsense_yolo 2>&1 | tail -3
source install/setup.bash
echo ""

# --- Step 3: Reinstall Jetson-specific PyTorch for CUDA support ---
echo -e "${CYAN}[3/4]${NC} Cài lại PyTorch Jetson (CUDA support)..."
PYTORCH_URL="https://developer.download.nvidia.com/compute/redist/jp/v512/pytorch/torch-2.1.0a0+41361538.nv23.06-cp38-cp38-linux_aarch64.whl"

CURRENT_TORCH=$(python3 -c "import torch; print(torch.__version__)" 2>/dev/null || echo "none")
echo "  PyTorch hiện tại: $CURRENT_TORCH"

if echo "$CURRENT_TORCH" | grep -q "nv23"; then
    echo -e "  ${GREEN}✓${NC} Đã là bản Jetson, bỏ qua."
else
    echo "  Đang cài lại bản Jetson..."
    pip3 install --user --no-cache-dir "$PYTORCH_URL" 2>&1 | tail -5
fi

# Verify CUDA
if python3 -c "import torch; assert torch.cuda.is_available()" 2>/dev/null; then
    echo -e "  ${GREEN}✓${NC} CUDA available!"
else
    echo -e "  ${YELLOW}!${NC} CUDA không available. PyTorch sẽ chạy CPU mode."
    echo "  (Ultralytics vẫn hoạt động nhưng chậm hơn trên CPU)"
fi
echo ""

# --- Step 4: Full verification ---
echo -e "${CYAN}[4/4]${NC} Kiểm tra toàn bộ..."
echo ""

echo "--- ROS2 Packages ---"
source /opt/ros/foxy/setup.bash
source "$ROS2_WS/install/setup.bash" 2>/dev/null || true
for pkg in realsense_yolo realsense2_camera realsense2_camera_msgs urg_node2 cv_bridge tf2_ros rviz2; do
    if ros2 pkg list 2>/dev/null | grep -q "^${pkg}$"; then
        echo -e "  ${GREEN}✓${NC} $pkg"
    else
        echo -e "  ${RED}✗${NC} $pkg"
    fi
done
echo ""

echo "--- Python Packages ---"
python3 -c "
import sys
pkgs = [
    ('torch', 'torch'),
    ('torchvision', 'torchvision'),
    ('ultralytics', 'ultralytics'),
    ('numpy', 'numpy'),
    ('opencv', 'cv2'),
    ('fastapi', 'fastapi'),
    ('uvicorn', 'uvicorn'),
]
for name, mod_name in pkgs:
    try:
        mod = __import__(mod_name)
        ver = getattr(mod, '__version__', 'OK')
        extra = ''
        if mod_name == 'torch':
            import torch
            extra = f' | CUDA: {torch.cuda.is_available()}'
        print(f'  ✓ {name} ({ver}){extra}')
    except Exception as e:
        print(f'  ✗ {name} ({e})')
" 2>&1
echo ""

echo "--- Hardware ---"
if cat /sys/bus/usb/devices/*/product 2>/dev/null | grep -q "RealSense"; then
    echo -e "  ${GREEN}✓${NC} RealSense D435i detected via USB"
else
    echo -e "  ${RED}✗${NC} RealSense D435i NOT detected"
fi

if ls /dev/ttyACM* 2>/dev/null | head -1 > /dev/null; then
    echo -e "  ${GREEN}✓${NC} Serial port: $(ls /dev/ttyACM* 2>/dev/null | tr '\n' ' ')"
else
    echo -e "  ${YELLOW}?${NC} No serial port (LiDAR chưa kết nối - OK)"
fi

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
echo "  VERIFICATION HOÀN TẤT!"
echo "============================================================"
echo ""
echo "Cách chạy:"
echo ""
echo "  # Camera only (không LiDAR):"
echo "  cd $ROS2_WS && source install/setup.bash"
echo "  ros2 launch realsense_yolo realsense_yolo_view.launch.py"
echo ""
echo "  # Full fusion (camera + LiDAR khi có):"
echo "  ros2 launch realsense_yolo fusion_3d.launch.py"
echo ""
echo "  # FastAPI web app:"
echo "  cd ~/ros2_ws/src/minh_delivery_robot"
echo "  python3 -m uvicorn app.main:app --host 0.0.0.0 --port 8000"
echo ""
