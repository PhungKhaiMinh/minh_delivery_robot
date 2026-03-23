# Minh Delivery Robot

Hệ thống fusion camera RealSense D435i + 2D LiDAR (Hokuyo UTM-30LX) + YOLO object detection cho robot giao hàng, chạy trên ROS2 Foxy.

## Tổng quan

- **RealSense D435i**: RGB + Depth (640x480 @ 15fps mặc định)
- **Hokuyo UTM-30LX**: 2D LiDAR 270°
- **YOLOv8**: Nhận diện vật thể trên ảnh RGB
- **Fusion 3D**: Kết hợp depth map + LiDAR + YOLO → PointCloud2 + 3D BBoxes, hiển thị trong RViz
- **Object Report**: Mỗi 1s in ra terminal: class, khoảng cách gần nhất (LiDAR ưu tiên), góc trái-phải

## Yêu cầu hệ thống

- Ubuntu 20.04
- ROS2 Foxy
- Python 3.8+
- NVIDIA GPU (khuyến nghị, cho YOLO; nếu không có sẽ chạy CPU)

## Clone & Cài đặt

### 1. Clone repo

```bash
cd ~
git clone https://github.com/PhungKhaiMinh/minh_delivery_robot.git
```

### 2. Cài đặt RealSense SDK (librealsense2)

```bash
sudo mkdir -p /etc/apt/keyrings
curl -sSf https://librealsense.intel.com/Debian/librealsense.pgp | \
  sudo tee /etc/apt/keyrings/librealsense.pgp > /dev/null
echo "deb [signed-by=/etc/apt/keyrings/librealsense.pgp] \
  https://librealsense.intel.com/Debian/apt-repo $(lsb_release -cs) main" | \
  sudo tee /etc/apt/sources.list.d/librealsense.list
sudo apt update
sudo apt install librealsense2-dkms librealsense2-utils librealsense2-dev -y
```

### 3. Cài đặt ROS2 dependencies

```bash
sudo apt install -y \
  ros-foxy-cv-bridge \
  ros-foxy-image-transport \
  ros-foxy-tf2-ros \
  ros-foxy-tf2-geometry-msgs \
  ros-foxy-rviz2
```

### 4. Cài đặt realsense-ros (nếu chưa có)

```bash
cd ~/minh_delivery_robot/ros2_ws/src
git clone https://github.com/IntelRealSense/realsense-ros.git -b ros2-development
cd ~/minh_delivery_robot/ros2_ws
source /opt/ros/foxy/setup.bash
rosdep install -i --from-paths src/realsense-ros --rosdistro foxy --skip-keys=librealsense2 -y
colcon build --packages-select realsense2_camera_msgs realsense2_camera
source install/setup.bash
```

### 5. Cài đặt urg_node2 (driver LiDAR Hokuyo) (nếu chưa có)

```bash
cd ~/minh_delivery_robot/ros2_ws/src
git clone https://github.com/Hokuyo-aut/urg_node2.git
cd ~/minh_delivery_robot/ros2_ws
rosdep install -i --from-paths src/urg_node2 --rosdistro foxy -y
colcon build --packages-select urg_node2
source install/setup.bash
```

### 6. Cài ultralytics (YOLO)

```bash
pip3 install ultralytics
```

### 7. Build package realsense_yolo

```bash
cd ~/minh_delivery_robot/ros2_ws
source /opt/ros/foxy/setup.bash
colcon build --packages-select realsense_yolo
source install/setup.bash
```

## Sử dụng

### Kiểm tra kết nối thiết bị

```bash
cd ~/minh_delivery_robot/ros2_ws/src/realsense_yolo/scripts
chmod +x check_devices.sh
./check_devices.sh
```

### Chạy Fusion 3D (RViz)

```bash
cd ~/minh_delivery_robot/ros2_ws
source install/setup.bash
ros2 launch realsense_yolo fusion_3d.launch.py
```

RViz sẽ hiển thị:
- **Depth Map 3D (surface)**: Bề mặt depth map với class-colored overlay
- **LiDAR PointCloud**: Point cloud từ LiDAR (xanh lá)
- **YOLO 3D Boxes**: Bounding box 3D cho từng vật thể
- **TF Axes**: Hệ trục tọa độ camera và laser

### Chạy Fusion 2D (OpenCV viewer)

```bash
cd ~/minh_delivery_robot/ros2_ws
source install/setup.bash
ros2 launch realsense_yolo fusion_view.launch.py
```

### Chạy YOLO only (không LiDAR)

```bash
cd ~/minh_delivery_robot/ros2_ws
source install/setup.bash
ros2 launch realsense_yolo realsense_yolo_view.launch.py
```

### Tham số thường dùng

```bash
# Đổi độ phân giải (mặc định 640x480)
ros2 launch realsense_yolo fusion_3d.launch.py \
  color_width:=1280 color_height:=720 depth_width:=1280 depth_height:=720

# Đổi FPS (mặc định 15)
ros2 launch realsense_yolo fusion_3d.launch.py color_fps:=30.0 depth_fps:=30.0

# Tắt RViz (chỉ chạy node)
ros2 launch realsense_yolo fusion_3d.launch.py use_rviz:=false

# Bật xoay trục TF (camera & LiDAR nhìn cùng hướng)
ros2 launch realsense_yolo fusion_3d.launch.py align_axes:=true
```

## Cấu trúc code

```
minh_delivery_robot/
├── ros2_ws/
│   └── src/
│       └── realsense_yolo/           # ROS2 package chính
│           ├── launch/
│           │   ├── fusion_3d.launch.py       # Launch fusion 3D + RViz
│           │   ├── fusion_view.launch.py     # Launch fusion 2D + OpenCV
│           │   ├── realsense_yolo_view.launch.py  # YOLO only + viewer
│           │   ├── realsense_yolo_full.launch.py  # Full launch
│           │   ├── realsense_yolo.launch.py       # Minimal launch
│           │   └── urg_serial.launch.py      # LiDAR driver launch
│           ├── realsense_yolo/
│           │   ├── realsense_yolo_node.py    # YOLO detection + NMS
│           │   ├── fusion_3d_node.py         # 3D fusion + object report
│           │   ├── camera_laser_tf_node.py   # TF camera ↔ laser
│           │   ├── lidar_depth_fusion_node.py # 2D fusion node
│           │   ├── image_viewer_node.py      # OpenCV image viewer
│           │   └── check_topics.py           # Debug: check ROS topics
│           ├── rviz/
│           │   ├── fusion_3d.rviz            # RViz config 3D
│           │   └── realsense_yolo.rviz       # RViz config basic
│           ├── scripts/
│           │   ├── check_devices.py          # Check camera + LiDAR
│           │   ├── check_devices.sh          # Check devices (bash)
│           │   └── run_realsense_yolo.sh     # Quick run script
│           ├── package.xml
│           ├── setup.py
│           ├── setup.cfg
│           └── README.md                     # Docs chi tiết
├── app/                                      # Delivery robot app (FastAPI)
├── data/                                     # Data collections
├── requirements.txt
└── README.md                                 # File này
```

## Lưu ý

- Nếu camera kết nối USB 2.0, nên dùng 640x480 @ 15fps, tắt infra (đã mặc định trong launch)
- Model YOLO (`yolov8n.pt`) sẽ tự tải lần đầu khi chạy
- Cổng LiDAR mặc định `/dev/ttyACM0`. Nếu khác, sửa trong `urg_node2/config/params_serial.yaml`
- Quyền serial cho LiDAR: `sudo usermod -aG dialout $USER` rồi logout/login
