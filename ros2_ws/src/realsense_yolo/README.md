# realsense_yolo

YOLO object detection on Intel RealSense D435i RGB image, map bounding boxes to Depth channel, visualize in RViz.

## Requirements

1. **librealsense2** - RealSense SDK
2. **realsense-ros** - ROS 2 wrapper
3. **ultralytics** - YOLOv8

## Installation

### 1. Install librealsense2

```bash
# Register server and install
sudo mkdir -p /etc/apt/keyrings
curl -sSf https://librealsense.intel.com/Debian/librealsense.pgp | sudo tee /etc/apt/keyrings/librealsense.pgp > /dev/null
echo "deb [signed-by=/etc/apt/keyrings/librealsense.pgp] https://librealsense.intel.com/Debian/apt-repo `lsb_release -cs` main" | sudo tee /etc/apt/sources.list.d/librealsense.list

sudo apt update
sudo apt install librealsense2-dkms librealsense2-utils librealsense2-dev -y

# Reboot if kernel module was installed
# sudo reboot
```

### 2. Install realsense-ros (in kinect_ws)

```bash
cd ~/kinect_ws
source /opt/ros/foxy/setup.bash

# Already cloned in src/realsense-ros
rosdep install -i --from-paths src/realsense-ros --rosdistro foxy --skip-keys=librealsense2 -y
colcon build --packages-select realsense2_camera_msgs realsense2_camera
source install/setup.bash
```

### 3. Install ultralytics (YOLO)

```bash
pip3 install ultralytics
```

### 4. Build realsense_yolo

```bash
cd ~/kinect_ws
source /opt/ros/foxy/setup.bash
colcon build --packages-select realsense_yolo
source install/setup.bash
```

## Usage

### Recommended: OpenCV Image Viewer (reliable display)

```bash
cd ~/kinect_ws
source install/setup.bash

ros2 launch realsense_yolo realsense_yolo_view.launch.py
```

- **YOLO Depth + BBoxes** – depth image với bounding boxes và khoảng cách
- **Camera Color** – ảnh màu trực tiếp từ camera
- Nhấn **q** hoặc **ESC** trong cửa sổ để thoát

### Option B: Full launch (camera + YOLO + image viewer + optional RViz)

```bash
ros2 launch realsense_yolo realsense_yolo_full.launch.py use_image_viewer:=true use_rviz:=false
```

### Option C: RealSense trước, sau đó chạy YOLO

**Terminal 1 - RealSense:**
```bash
ros2 launch realsense2_camera rs_launch.py align_depth:=true
```

**Terminal 2 - YOLO + viewer:**
```bash
source ~/kinect_ws/install/setup.bash
ros2 launch realsense_yolo realsense_yolo_view.launch.py
```

### Topics

- **Input**: `/camera/color/image_raw`, `/camera/aligned_depth_to_color/image_raw`
- **Output**: `/realsense_yolo/depth_with_bboxes` - Depth image with bounding boxes drawn

### RViz (optional)

Add **Image** display, Topic: `/realsense_yolo/depth_with_bboxes`.  
Lưu ý: OpenCV viewer (`realsense_yolo_view.launch.py`) thường hiển thị ổn định hơn RViz.

### Custom topics

```bash
ros2 launch realsense_yolo realsense_yolo.launch.py \
  color_topic:=/your/color/topic \
  depth_topic:=/your/aligned_depth/topic
```

### Điều chỉnh độ phân giải (bao quát xung quanh)

```bash
# Mặc định: 1280x720 (max D435i, bao quát tốt nhất)
ros2 launch realsense_yolo realsense_yolo_view.launch.py

# Phân giải thấp hơn (FPS cao hơn)
ros2 launch realsense_yolo realsense_yolo_view.launch.py color_width:=640 color_height:=480 depth_width:=640 depth_height:=480

# imgsz YOLO: 416 (nhanh) / 640 (cân bằng) / 1280 (chính xác)
ros2 launch realsense_yolo realsense_yolo_view.launch.py imgsz:=416
```

**Tham số:**
- `color_width`, `color_height`: 1280x720 (mặc định, max) / 640x480 (nhanh)
- `depth_width`, `depth_height`: phải khớp color khi dùng align_depth
- `imgsz`: 416 (nhanh) / 640 (cân bằng) / 1280 (chính xác hơn)

## Fusion RealSense + 2D LiDAR (UTM-30LX)

Kết hợp depth map + YOLO với dữ liệu 2D LiDAR (Hokuyo UTM-30LX). Ảnh fusion: depth + bbox + điểm LiDAR chiếu lên ảnh.

### Yêu cầu

- RealSense D435i (đã cài realsense2_camera)
- LiDAR UTM-30LX kết nối USB (driver **urg_node2** trong workspace)
- Build urg_node2: `cd ~/kinect_ws && colcon build --packages-select urg_node2`

### Bước 1: Kiểm tra kết nối

```bash
cd ~/kinect_ws/src/realsense_yolo/scripts
chmod +x check_devices.sh
./check_devices.sh
# hoặc: python3 check_devices.py
```

- RealSense: cần thấy `/dev/video*` và lsusb có Intel.
- LiDAR: cần thấy `/dev/ttyACM0` hoặc `/dev/ttyUSB0`. Nếu không có quyền: `sudo usermod -aG dialout $USER` rồi đăng xuất/đăng nhập lại.

### Bước 2: Cấu hình cổng LiDAR (nếu khác /dev/ttyACM0)

Sửa file `urg_node2/config/params_serial.yaml` (trong src hoặc install), đặt `serial_port: '/dev/ttyUSB0'` (hoặc đúng cổng của bạn). Sau đó build lại: `colcon build --packages-select urg_node2`.

### Bước 3: Chạy fusion

```bash
cd ~/kinect_ws
source install/setup.bash
ros2 launch realsense_yolo fusion_view.launch.py
```

Sẽ mở 3 cửa sổ: **Camera Color**, **YOLO Depth + BBoxes**, **Fused Depth + LiDAR + YOLO** (depth + bbox + điểm LiDAR màu vàng).

### Tham số fusion (TF laser – camera)

- **Vị trí**: LiDAR trong frame camera. Mặc định camera trên đầu LiDAR ~5 cm → `lidar_x=0`, `lidar_y=0.05`, `lidar_z=0`.
- **Hiệu chỉnh trục tọa độ**: Sửa file **`realsense_yolo/camera_laser_tf_node.py`** (translation + quaternion), rồi trong launch thay `static_transform_publisher` bằng node `camera_laser_tf_node`. Chi tiết xem comment đầu file đó.

```bash
ros2 launch realsense_yolo fusion_view.launch.py lidar_x:=0.0 lidar_y:=0.05 lidar_z:=0.0
```

### Topics

- **Input**: `/camera/color/image_raw`, `/camera/aligned_depth_to_color/image_raw`, `/camera/aligned_depth_to_color/camera_info`, `/scan` (LaserScan)
- **Output**: `/realsense_yolo/depth_with_bboxes`, `/realsense_yolo/fused_depth_lidar`

---

## Fusion 3D (xem trong RViz – phục vụ dẫn đường robot)

Kết quả fusion được đưa vào **không gian 3D**: point cloud (depth + LiDAR) và **3D bounding box** cho từng vật YOLO detect. Xem trong RViz, frame `camera_depth_optical_frame`.

### Chạy

```bash
cd ~/kinect_ws
source install/setup.bash
ros2 launch realsense_yolo fusion_3d.launch.py
```

- **RViz** hiển thị (trong view 3D):
  - **Depth Map 3D (surface)** – bề mặt depth map trong 3D (màu JET)
  - **Depth PointCloud** – point cloud từ depth (đỏ gần, xanh xa)
  - **LiDAR PointCloud** – point cloud từ 2D LiDAR (màu xanh lá), đã transform sang frame camera (TF: camera Z = laser X = cùng hướng)
  - **YOLO 3D Boxes** – hộp 3D (khung dây) cho từng vật được YOLO detect

### Topics 3D

- **Input**: `/camera/aligned_depth_to_color/image_raw`, `.../camera_info`, `/scan`, `/realsense_yolo/detections`
- **Output**:
  - `/realsense_yolo/depth_pointcloud` (PointCloud2)
  - `/realsense_yolo/lidar_pointcloud` (PointCloud2)
  - `/realsense_yolo/detection_boxes_3d` (MarkerArray)

### Tắt RViz (chỉ chạy node, xem bằng tool khác)

```bash
ros2 launch realsense_yolo fusion_3d.launch.py use_rviz:=false
```
