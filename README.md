# realsense_yolo (fusion_3d only)

> **Git history:** Older commits still contain the previous *Minh Delivery Robot* tree and long README. Use `git log` and `git show <hash>:README.md` to read them.  
> **Lịch sử Git:** Các commit cũ vẫn giữ README và cấu trúc workspace trước đây — xem `git log` / `git show`.

ROS 2 Python package: **RealSense D435i** + **YOLO11** (+ TensorRT) + **URG LiDAR** → **`/obstacles`** cho robot.

## Chạy

```bash
source install/setup.bash   # hoặc ws của bạn
ros2 launch realsense_yolo fusion_3d.launch.py
```

Tuỳ chọn RViz / pointcloud (tốn CPU + GPU):

```bash
ros2 launch realsense_yolo fusion_3d.launch.py use_rviz:=true publish_pointclouds:=true
```

Khởi động nhanh hơn ~5–8 s khi camera không cần hardware reset (thoát ROS sạch, không kẹt frame):

```bash
ros2 launch realsense_yolo fusion_3d.launch.py camera_initial_reset:=false
```

## Export TensorRT (offline trên Jetson)

```bash
ros2 run realsense_yolo export_engine --weights /path/to/yolo11n.pt --imgsz 480
```

## Khởi động nhanh / chậm

- `startup_camera_delay_s` — trì hoãn trước khi mở node RealSense (USB).
- `startup_pipeline_after_realsense_s` — sau khi **process camera đã start**, chờ thêm bấy nhiêu giây rồi mới chạy YOLO + fusion + RViz (luôn đứng sau camera, không chờ “mốc tuyệt đối” dài như trước).

Nếu depth/camera_info chưa kịp: tăng `startup_pipeline_after_realsense_s` (ví dụ `2.5`). USB chậm: tăng `startup_camera_delay_s`. PSU yếu: tăng `startup_pipeline_after_realsense_s` để tách spike GPU.

LiDAR Hokuyo: `urg_node2` đọc cổng trong `share/urg_node2/config/params_serial.yaml` (hoặc mặc định `/dev/deli_lidar`). Cần symlink hoặc chỉnh đúng device khi không phải `/dev/deli_lidar`.

## Nodes

| Executable           | Vai trò                          |
|---------------------|-----------------------------------|
| `realsense_yolo_node` | RGB + depth → `/realsense_yolo/detections` |
| `fusion_3d_node`      | detections + scan + depth → `/obstacles`   |
| `export_engine`       | Công cụ export `.engine` (không dùng khi chạy robot) |

## Topics chính

- `/camera/color/image_raw`, `/camera/aligned_depth_to_color/image_raw`
- `/scan`
- `/realsense_yolo/detections`
- `/obstacles`

Calib TF laser ↔ camera: launch args `lidar_x`, `lidar_y`, `lidar_z`, `laser_frame`.
