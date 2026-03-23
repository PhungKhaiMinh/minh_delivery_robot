# Minh Delivery Robot

Hệ thống fusion camera Intel RealSense D435i + 2D LiDAR Hokuyo UTM-30LX + YOLOv8 object detection cho robot giao hàng, chạy trên **ROS2 Foxy** / **Jetson AGX Xavier**.

## Tổng quan

- **RealSense D435i**: RGB + Depth (640×480 @ 15fps mặc định)
- **Hokuyo UTM-30LX**: 2D LiDAR 270°, kết nối serial `/dev/ttyACM0`
- **YOLOv8**: Nhận diện vật thể trên ảnh RGB (GPU FP16 trên Jetson, hoặc CPU)
- **Fusion 3D**: Kết hợp depth map + LiDAR + YOLO → PointCloud2 + 3D BBoxes, hiển thị trong RViz
- **Object Report**: Mỗi 1 giây in ra terminal: class, khoảng cách gần nhất (ưu tiên LiDAR nếu có), góc trái-phải

### Fusion logic — LiDAR hay Depth?

Hệ thống **ưu tiên LiDAR** cho đo khoảng cách. Quy trình xử lý cho mỗi vật thể YOLO phát hiện:

1. Từ bounding box 2D (pixel), dùng depth map + camera intrinsics để chiếu ra tọa độ 3D trong frame camera
2. Dùng TF transform để chuyển tọa độ 3D sang frame LiDAR → tính góc trái/phải của vật thể nhìn từ LiDAR
3. Lọc các tia LiDAR nằm trong khoảng góc đó → lấy `min(range)` = khoảng cách gần nhất
4. Nếu **có tia LiDAR hợp lệ** → `source=lidar` (chính xác hơn, không bị nhiễu như depth)
5. Nếu **không có** (vật thể ngoài tầm LiDAR, hoặc LiDAR chưa kết nối) → `source=depth` (dùng depth camera)

Output trên terminal:

```
person conf=0.85 nearest=1.23m source=lidar angles=[-5.2°, 8.1°] span=13.3°
chair  conf=0.72 nearest=2.45m source=depth angles=[-25.0°, -18.3°] span=6.7°
```

---

## Hệ tọa độ Camera vs LiDAR — Giải thích chi tiết

### Quy ước trục tọa độ

Hai cảm biến sử dụng **quy ước trục khác nhau**:

```
Camera (camera_depth_optical_frame)       LiDAR (laser frame)
────────────────────────────────          ─────────────────────
        Z (forward/trước)                      X (forward/trước)
       ╱                                      ╱
      ╱                                      ╱
     ╱                                      ╱
    O ─────── X (right/phải)               O ─────── Y (left/trái)
    │                                      │
    │                                      │
    Y (down/xuống)                         Z (up/lên)
```

| Hướng        | Camera frame          | LiDAR frame      |
|-------------|-----------------------|-------------------|
| **Trước**   | +Z                    | +X                |
| **Phải**    | +X                    | −Y                |
| **Xuống**   | +Y                    | −Z                |

### Static TF Transform

Để ROS2 biết mối quan hệ giữa 2 frame, ta publish một **static transform** từ `camera_depth_optical_frame` → `laser`:

```
Parent frame:  camera_depth_optical_frame
Child frame:   laser
Translation:   (lidar_x, lidar_y, lidar_z) — mét
Quaternion:    (0.5, -0.5, 0.5, 0.5)
```

**Quaternion `(0.5, -0.5, 0.5, 0.5)`** thực hiện phép xoay đồng trục (axis alignment):

| LiDAR axis | → | Camera axis | Ý nghĩa |
|------------|---|-------------|----------|
| X (trước)  | → | Z (trước)   | Cả hai nhìn cùng hướng |
| Y (trái)   | → | −X (trái)   | Trái LiDAR = trái camera |
| Z (lên)    | → | −Y (lên)    | Lên LiDAR = lên camera |

> **Lưu ý**: Quaternion này chỉ đúng khi camera và LiDAR **hướng cùng phía trước**. Nếu LiDAR bị xoay (ví dụ quay 90°), cần tính lại quaternion.

### Translation (lidar_x, lidar_y, lidar_z) — Vị trí vật lý

Translation mô tả **vị trí gốc tọa độ LiDAR** so với **gốc tọa độ camera**, đo trong **frame camera** (đơn vị: mét).

```
Nhìn từ phía trước robot:
                              ┌──────────────┐
          lidar_x ◄────────  │              │
        (right/phải)         │   CAMERA     │ ← gốc camera (0,0,0)
                              │   D435i      │
                              └──────────────┘
                                     │
                      lidar_y        │  (down/xuống dương)
                      = 0.05m       │
                                     ▼
                              ┌──────────────┐
                              │   LiDAR      │ ← gốc laser
                              │   UTM-30LX   │
                              └──────────────┘
                                     │
                      lidar_z ──────►  (forward/trước dương)
```

| Tham số   | Mặc định | Ý nghĩa                                                                 |
|-----------|----------|-------------------------------------------------------------------------|
| `lidar_x` | `0.0`    | LiDAR lệch sang **phải** bao nhiêu mét so với camera. Trái = giá trị âm |
| `lidar_y` | `0.05`   | LiDAR lệch **xuống** bao nhiêu mét so với camera. Lên = giá trị âm      |
| `lidar_z` | `0.0`    | LiDAR lệch **về trước** bao nhiêu mét so với camera. Sau = giá trị âm   |

**Mặc định `(0.0, 0.05, 0.0)`** = LiDAR nằm **ngay bên dưới camera 5 cm**, cùng trục dọc, cùng hướng nhìn.

### Cách đo và điều chỉnh khi thay đổi vị trí cảm biến

**Bước 1: Đo khoảng cách vật lý** giữa tâm thấu kính camera (gốc `camera_depth_optical_frame`) và tâm quay LiDAR (gốc `laser`), theo 3 hướng của frame camera:

- Sang phải → `lidar_x` dương (sang trái → âm)
- Xuống dưới → `lidar_y` dương (lên trên → âm)
- Về phía trước → `lidar_z` dương (về phía sau → âm)

**Bước 2: Truyền vào launch:**

```bash
ros2 launch realsense_yolo fusion_3d.launch.py \
  lidar_x:=0.03 \
  lidar_y:=0.08 \
  lidar_z:=-0.02
```

Ví dụ trên: LiDAR nằm lệch phải 3 cm, dưới 8 cm, lùi sau 2 cm so với camera.

**Bước 3: Kiểm tra trong RViz** — Bật hiển thị TF axes (`camera_depth_optical_frame` và `laser`). Hai gốc tọa độ phải khớp với vị trí vật lý thực tế.

> **Mẹo**: Nếu không chắc chắn, để mặc định `(0, 0.05, 0)`. Sai số vài cm chỉ ảnh hưởng nhẹ đến việc match tia LiDAR với bounding box — hệ thống vẫn hoạt động tốt.

---

## Yêu cầu hệ thống

| Thành phần    | Yêu cầu                                         |
|--------------|--------------------------------------------------|
| **OS**        | Ubuntu 20.04 (JetPack 5.x cho Jetson)           |
| **ROS2**      | Foxy Fitzroy                                     |
| **Python**    | 3.8+                                             |
| **GPU**       | NVIDIA (khuyến nghị cho YOLO; không có thì CPU)  |
| **Camera**    | Intel RealSense D435i (USB 3.0 khuyến nghị)     |
| **LiDAR**     | Hokuyo UTM-30LX (serial, `/dev/ttyACM0`)        |

---

## Cài đặt

### Cách nhanh — Jetson AGX Xavier (khuyến nghị)

```bash
cd ~
git clone https://github.com/PhungKhaiMinh/minh_delivery_robot.git \
  ros2_ws/src/minh_delivery_robot

cd ~/ros2_ws/src/minh_delivery_robot

# Bước 1: Cài packages hệ thống (cần sudo)
sudo bash sudo_setup.sh

# Bước 2: Build ROS2 + cài PyTorch Jetson (không cần sudo)
bash post_sudo_setup.sh
```

### Cài thủ công (từng bước)

<details>
<summary>Xem chi tiết</summary>

#### 1. Clone repo

```bash
cd ~/ros2_ws/src
git clone https://github.com/PhungKhaiMinh/minh_delivery_robot.git
```

#### 2. Cài RealSense SDK + ROS2 driver

```bash
sudo apt update
sudo apt install -y \
  ros-foxy-librealsense2 \
  ros-foxy-realsense2-camera \
  ros-foxy-realsense2-camera-msgs \
  ros-foxy-realsense2-description
```

#### 3. Cài ROS2 dependencies

```bash
sudo apt install -y \
  ros-foxy-cv-bridge \
  ros-foxy-image-transport \
  ros-foxy-tf2-ros \
  ros-foxy-tf2-geometry-msgs \
  ros-foxy-rviz2 \
  ros-foxy-diagnostic-updater \
  ros-foxy-lifecycle-msgs \
  ros-foxy-urg-c
```

#### 4. Clone & build urg_node2 (LiDAR driver)

```bash
cd ~/ros2_ws/src/minh_delivery_robot/ros2_ws/src
git clone https://github.com/Hokuyo-aut/urg_node2.git

cd ~/ros2_ws/src/minh_delivery_robot/ros2_ws
source /opt/ros/foxy/setup.bash
rosdep install -i --from-paths src/urg_node2 --rosdistro foxy -y
colcon build --packages-select urg_node2
source install/setup.bash
```

#### 5. Cài Python packages

```bash
pip3 install ultralytics numpy
```

#### 6. Build realsense_yolo

```bash
cd ~/ros2_ws/src/minh_delivery_robot/ros2_ws
source /opt/ros/foxy/setup.bash
source install/setup.bash 2>/dev/null || true
colcon build --packages-select realsense_yolo
source install/setup.bash
```

#### 7. Quyền serial cho LiDAR

```bash
sudo usermod -aG dialout $USER
# Logout rồi login lại
```

</details>

---

## Sử dụng

### Kiểm tra kết nối thiết bị

```bash
cd ~/ros2_ws/src/minh_delivery_robot/ros2_ws/src/realsense_yolo/scripts
chmod +x check_devices.sh
./check_devices.sh
```

### Chạy Fusion 3D (Camera + LiDAR + RViz)

```bash
cd ~/ros2_ws/src/minh_delivery_robot/ros2_ws
source install/setup.bash
ros2 launch realsense_yolo fusion_3d.launch.py
```

RViz sẽ hiển thị:

| Display                | Mô tả                                                    |
|------------------------|-----------------------------------------------------------|
| **Depth Map 3D**       | Bề mặt depth map với class-colored overlay (JET colormap) |
| **LiDAR PointCloud**   | Point cloud từ LiDAR (xanh lá)                           |
| **YOLO 3D Boxes**      | Bounding box 3D cho từng vật thể                          |
| **LaserScan**          | Raw laser scan (dạng quạt 270°)                           |
| **TF Axes**            | Hệ trục tọa độ camera và laser                            |

### Chạy Fusion 2D (OpenCV viewer)

```bash
ros2 launch realsense_yolo fusion_view.launch.py
```

### Chạy YOLO only (không cần LiDAR)

```bash
ros2 launch realsense_yolo realsense_yolo_view.launch.py
```

> Nếu không có LiDAR kết nối, launch file sẽ tự động skip `urg_node2`.

### Tham số launch thường dùng

```bash
# Thay đổi vị trí LiDAR so với camera (mét)
ros2 launch realsense_yolo fusion_3d.launch.py \
  lidar_x:=0.0 lidar_y:=0.05 lidar_z:=0.0

# Đổi độ phân giải
ros2 launch realsense_yolo fusion_3d.launch.py \
  rgb_profile:=1280x720x15 depth_profile:=1280x720x15

# Tắt RViz (chỉ chạy node, xem qua terminal)
ros2 launch realsense_yolo fusion_3d.launch.py use_rviz:=false
```

| Tham số          | Mặc định        | Mô tả                                          |
|------------------|------------------|-------------------------------------------------|
| `rgb_profile`    | `640x480x15`     | Độ phân giải + FPS ảnh RGB                      |
| `depth_profile`  | `640x480x15`     | Độ phân giải + FPS ảnh depth                    |
| `lidar_x`        | `0.0`            | LiDAR lệch phải so với camera (m)              |
| `lidar_y`        | `0.05`           | LiDAR lệch xuống so với camera (m)             |
| `lidar_z`        | `0.0`            | LiDAR lệch trước so với camera (m)             |
| `laser_frame`    | `laser`          | TF frame name của LiDAR                        |
| `use_rviz`       | `true`           | Bật/tắt RViz                                   |

---

## ROS2 Topics

| Topic                                        | Type               | Mô tả                                  |
|----------------------------------------------|--------------------|-----------------------------------------|
| `/camera/color/image_raw`                    | `Image`            | Ảnh RGB từ RealSense                   |
| `/camera/aligned_depth_to_color/image_raw`   | `Image`            | Ảnh depth đã align với RGB              |
| `/scan`                                      | `LaserScan`        | Scan 2D từ Hokuyo LiDAR                |
| `/realsense_yolo/detections`                 | `Float32MultiArray` | YOLO detections (x1,y1,x2,y2,depth,cls,conf) |
| `/realsense_yolo/depth_pointcloud`           | `PointCloud2`      | Point cloud từ depth (đỏ gần, xanh xa) |
| `/realsense_yolo/lidar_pointcloud`           | `PointCloud2`      | LiDAR points (xanh lá) trong frame camera |
| `/realsense_yolo/depth_image_plane`          | `PointCloud2`      | Depth map surface 3D (JET colormap)    |
| `/realsense_yolo/detection_boxes_3d`         | `MarkerArray`      | 3D bounding boxes trong RViz           |

---

## Cấu trúc code

```
minh_delivery_robot/
├── ros2_ws/
│   └── src/
│       └── realsense_yolo/               # ROS2 package chính
│           ├── launch/
│           │   ├── fusion_3d.launch.py          # ★ Launch fusion 3D + RViz
│           │   ├── fusion_view.launch.py        # Launch fusion 2D + OpenCV
│           │   ├── realsense_yolo_view.launch.py  # YOLO only + viewer
│           │   ├── realsense_yolo_full.launch.py  # Full launch
│           │   ├── realsense_yolo.launch.py       # Minimal launch
│           │   └── urg_serial.launch.py           # LiDAR driver launch
│           ├── realsense_yolo/
│           │   ├── fusion_3d_node.py            # ★ 3D fusion + object report
│           │   ├── realsense_yolo_node.py       # YOLO detection + NMS
│           │   ├── lidar_reset_node.py          # Reset LiDAR serial trước khi start
│           │   ├── camera_laser_tf_node.py      # TF camera ↔ laser (legacy)
│           │   ├── lidar_depth_fusion_node.py   # 2D fusion node
│           │   ├── image_viewer_node.py         # OpenCV image viewer
│           │   └── check_topics.py              # Debug: check ROS topics
│           ├── rviz/
│           │   ├── fusion_3d.rviz               # RViz config 3D
│           │   └── realsense_yolo.rviz          # RViz config basic
│           ├── scripts/
│           │   ├── check_devices.py
│           │   ├── check_devices.sh
│           │   └── run_realsense_yolo.sh
│           ├── package.xml
│           ├── setup.py
│           └── setup.cfg
├── app/                                         # Delivery robot app (FastAPI)
├── data/                                        # Data collections
├── setup_jetson.sh                              # Full auto-setup script
├── sudo_setup.sh                                # Bước 1: cài packages hệ thống (sudo)
├── post_sudo_setup.sh                           # Bước 2: build + verify (không sudo)
├── requirements.txt
└── README.md                                    # File này
```

---

## Troubleshooting

### LiDAR không publish `/scan`

**Triệu chứng**: `urg_node2` khởi động nhưng không có dữ liệu trên `/scan`, log báo "invalid response".

**Nguyên nhân**: Nếu hệ thống bị tắt đột ngột (kill -9, mất điện...), Hokuyo LiDAR vẫn ở chế độ SCIP2 streaming. Lần chạy tiếp theo, `urg_node2` không thể configure vì serial port đang nhận dữ liệu liên tục.

**Giải pháp**: Launch file `fusion_3d.launch.py` đã tự động gửi lệnh `QT` (quit) qua serial để reset LiDAR trước khi khởi động `urg_node2`. Nếu vẫn lỗi:

```bash
# Reset thủ công
python3 -c "
import os, termios, time
fd = os.open('/dev/ttyACM0', os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
termios.tcflush(fd, termios.TCIOFLUSH)
os.write(fd, b'QT\n'); time.sleep(0.3)
termios.tcflush(fd, termios.TCIOFLUSH)
os.close(fd); print('LiDAR reset OK')
"
```

### RViz hiển thị 3D theo chiều dọc (đứng)

**Nguyên nhân**: Fixed Frame trong RViz đang dùng `camera_depth_optical_frame` (Z = forward → RViz hiểu là hướng lên).

**Giải pháp**: Đổi `Global Options > Fixed Frame` thành `camera_link` (Z = up, theo quy ước ROS chuẩn). File `fusion_3d.rviz` đã được cấu hình sẵn.

### Permission denied khi mở LiDAR serial port

```bash
sudo usermod -aG dialout $USER
# Cần logout rồi login lại
```

### `cv_bridge` import lỗi trên Jetson

**Nguyên nhân**: Xung đột phiên bản OpenCV (system vs pip). Launch file đã set `LD_PRELOAD=/lib/aarch64-linux-gnu/libgomp.so.1` để workaround.

### Camera USB 2.0 — ảnh bị lag hoặc lỗi

Dùng profile thấp hơn:

```bash
ros2 launch realsense_yolo fusion_3d.launch.py \
  rgb_profile:=640x480x15 depth_profile:=640x480x15
```

Tắt infra (đã tắt mặc định trong launch), dùng USB 3.0 nếu có.

---

## Pipeline chi tiết

```
┌─────────────┐     ┌─────────────────┐     ┌──────────────┐
│  RealSense  │────▶│  YOLO Node      │────▶│  Fusion 3D   │
│  D435i      │     │  (detection +   │     │  Node         │
│  RGB+Depth  │     │   NMS)          │     │              │
└─────────────┘     └─────────────────┘     │  • depth →   │
                                             │    pointcloud│
┌─────────────┐     ┌─────────────────┐     │  • LiDAR →   │
│  Hokuyo     │────▶│  urg_node2      │────▶│    pointcloud│
│  UTM-30LX   │     │  (lifecycle)    │     │  • YOLO →    │
│  2D LiDAR   │     │  /scan topic    │     │    3D boxes  │
└─────────────┘     └─────────────────┘     │  • report    │
                                             └──────┬───────┘
                                                    │
                                                    ▼
                                             ┌──────────────┐
                                             │    RViz2     │
                                             │  3D viewer   │
                                             └──────────────┘
```

---

## License

MIT
