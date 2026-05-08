"""
Launch Fusion 3D: RealSense + YOLO + LiDAR — đường dây cho obstacle avoidance.

⚠️ JETSON AGX XAVIER POWER NOTE — đọc kỹ trước khi chạy
  1. KHÔNG dùng MAXN (mode 0) trừ khi PSU ≥ 90 W (19V/4.74A barrel chính hãng).
     PSU 65 W (19V/3.42A) sẽ trip OCP khi peak inference + RViz khởi động → SẬP NGUỒN.
     Khuyến cáo:
       sudo nvpmodel -m 3        # MODE_30W_ALL (đủ cho ~50 FPS YOLO11n + Fusion)
       sudo jetson_clocks         # khoá xung max, tắt DVFS (tránh spike đột ngột)
       sudo sh -c 'echo 255 > /sys/devices/pwm-fan/target_pwm'   # quạt full

  2. Engine TensorRT phải export OFFLINE trước (đã làm rồi):
       ros2 run realsense_yolo export_engine --weights /mnt/jetson_data/deli_ws/yolo11n.pt --imgsz 480

  3. RViz mặc định TẮT (use_rviz:=false) — RViz là kẻ ăn điện lớn nhất:
     allocate GL context + 3 PointCloud2 + Image + MarkerArray cùng lúc → spike GPU.
     RViz có thể chạy trên máy khác với ros2 bag hoặc DDS remote — hoặc bật use_rviz:=true.

  4. PointCloud2 mặc định TẮT (publish_pointclouds:=false). Vẫn có /obstacles.

Khởi động (ưu tiên tốc độ — chỉnh startup_* / camera_initial_reset nếu USB/PSU chưa ổn):
  t≈0    TF tĩnh + reset LiDAR + urg_node2 (song song)
  t=Tc   RealSense (Tc = startup_camera_delay_s, mặc định 1.0 s)
  sau khi process camera đã start → chờ Ta → YOLO + fusion + RViz (Ta = startup_pipeline_after_realsense_s, mặc định 1.8 s)
  TensorRT load trong luồng worker của realsense_yolo_node (song song với camera warm-up).
  camera_initial_reset=false tiết kiệm ~5–8 s (bỏ hardware reset D435i); chỉ dùng khi thoát ROS sạch, kẹt frame → true.

Calibration (camera + LiDAR cùng hướng quan sát):
  lidar_x, lidar_y, lidar_z = vị trí gốc laser trong camera_depth_optical_frame (m):
    X: phải +,  Y: xuống + (ảnh),  Z: tới phía trước mắt cảnh +.
  Gắn chuẩn (lidar trên camera 23 cm, sau camera 53 cm, không lệch ngang): (0, -0.23, -0.53).
  Quaternion (0.5, -0.5, 0.5, 0.5): trục laser X (tới trước) thẳng hàng với Z optical của camera.
"""

import os
import yaml
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, TimerAction, ExecuteProcess,
    RegisterEventHandler, EmitEvent,
)
from launch.substitutions import LaunchConfiguration
from launch.event_handlers import OnProcessStart, OnProcessExit
from launch.events import matches_action
from launch_ros.actions import Node, LifecycleNode
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from launch.conditions import IfCondition
from lifecycle_msgs.msg import Transition
from ament_index_python.packages import get_package_share_directory, PackageNotFoundError


def generate_launch_description():
    pkg = get_package_share_directory('realsense_yolo')
    # Gán thẳng đường dẫn ở đây
    yolo_weights = '/mnt/jetson_data/deli_ws/yolov8n.pt'
    has_urg_node2 = False
    urg_params = {
        'serial_port': '/dev/deli_lidar',
        'serial_baud': 115200,
        'frame_id': 'laser',
        'calibrate_time': False,
        'angle_min': -3.14,
        'angle_max': 3.14,
    }
    try:
        urg_pkg = get_package_share_directory('urg_node2')
        has_urg_node2 = True
        yaml_path = os.path.join(urg_pkg, 'config', 'params_serial.yaml')
        if os.path.isfile(yaml_path):
            with open(yaml_path, 'r') as f:
                urg_params = yaml.safe_load(f)['urg_node2']['ros__parameters']
    except (PackageNotFoundError, Exception):
        pass

    serial_port = urg_params.get('serial_port', '/dev/ttyACM0')
    rviz_config = os.path.join(pkg, 'rviz', 'fusion_3d.rviz')

    ld = LaunchDescription()

    ld.add_action(DeclareLaunchArgument('rgb_profile', default_value='640x480x30'))
    ld.add_action(DeclareLaunchArgument('depth_profile', default_value='640x480x30'))
    ld.add_action(DeclareLaunchArgument('laser_frame', default_value='laser'))
    ld.add_action(DeclareLaunchArgument(
        'lidar_x', default_value='0.0',
        description='Gốc laser trong camera_depth_optical_frame, trục X (m). Dương = phải. 0 = không lệch ngang.',
    ))
    ld.add_action(DeclareLaunchArgument(
        'lidar_y', default_value='-0.23',
        description='Trục Y (m, xuống dưới hình = dương). Laser trên camera 0.23 m => -0.23.',
    ))
    ld.add_action(DeclareLaunchArgument(
        'lidar_z', default_value='-0.53',
        description='Trục Z (m, tới trước mắt cảnh = dương). Laser sau camera 0.53 m => -0.53.',
    ))
    # ⚠️ RViz mặc định TẮT — bật ON gây spike GPU + tốn ~30% CPU.
    # Bật khi cần debug:  ros2 launch realsense_yolo fusion_3d.launch.py use_rviz:=true
    ld.add_action(DeclareLaunchArgument('use_rviz', default_value='false'))
    # Khi use_rviz=false thì cũng KHÔNG cần publish 3 PointCloud2 nặng.
    # Set 'true' nếu bạn dùng RViz để debug, hoặc subscribe pcs từ máy khác.
    ld.add_action(DeclareLaunchArgument('publish_pointclouds', default_value='false'))
    ld.add_action(DeclareLaunchArgument('publish_yolo_image', default_value='false'))
    ld.add_action(DeclareLaunchArgument(
        'camera_initial_reset',
        default_value='true',
        description=(
            'RealSense initial_reset (hardware reset). false ≈ tiết kiệm 5–8 s khởi động nếu thoát ROS sạch; '
            'true an toàn sau Ctrl-C / camera không pump frame.'
        ),
    ))

    # ── YOLO + Tracking + TensorRT params ─────────────────────────
    ld.add_action(DeclareLaunchArgument(
        'yolo_model', default_value='yolo11n.pt',
        description='Tên/đường dẫn model. yolo11n.pt = SOTA Ultralytics (mAP 39.5, 6.5 GFLOPs).',
    ))
    ld.add_action(DeclareLaunchArgument(
        'imgsz', default_value='480',
        description='Kích thước input YOLO. 480 = nhanh + nhẹ; 640 = hơi chính xác hơn nhưng tốn ~50% tải GPU.',
    ))
    ld.add_action(DeclareLaunchArgument(
        'use_tensorrt', default_value='true',
        description='Load TensorRT engine nếu có sẵn (nhanh gấp 3-5x PyTorch).',
    ))
    ld.add_action(DeclareLaunchArgument(
        'auto_export_engine', default_value='false',
        description='Export TRT runtime (NGUY HIỂM trên Jetson — có thể sập nguồn). Hãy export OFFLINE.',
    ))
    ld.add_action(DeclareLaunchArgument(
        'tensorrt_dla', default_value='-1',
        description='-1 = không DLA (GPU thuần, nhanh nhất với YOLO11). 0/1 = ép DLA core (ops sẽ fallback GPU).',
    ))
    ld.add_action(DeclareLaunchArgument(
        'tensorrt_int8', default_value='false',
        description='Bật INT8 (cần calibration set; off mặc định để giữ accuracy).',
    ))
    ld.add_action(DeclareLaunchArgument(
        'enable_tracking', default_value='true',
        description='Bật BoT-SORT tracker → phát track_id ổn định qua frame.',
    ))
    ld.add_action(DeclareLaunchArgument(
        'tracker_config', default_value='',
        description='Đường dẫn YAML BoT-SORT. Mặc định: share/realsense_yolo/config/botsort_deli.yaml.',
    ))
    ld.add_action(DeclareLaunchArgument(
        'startup_camera_delay_s',
        default_value='1.0',
        description='Giây sau khi launch trước khi start RealSense (USB enumerate).',
    ))
    ld.add_action(DeclareLaunchArgument(
        'startup_pipeline_after_realsense_s',
        default_value='1.8',
        description=(
            'Giây SAU KHI process realsense2_camera đã khởi động rồi mới start YOLO + fusion + RViz. '
            'Giữ ~1.5–2.5 s để có camera_info + vài frame; tăng nếu depth chưa kịp.'
        ),
    ))

    # ── RealSense ────────────────────────────────────────────────
    # Lưu ý: initial_reset cần BẬT khi camera còn dangling từ session trước
    # (rất hay xảy ra với D435i nếu Ctrl-C gãy giữa chừng). Nếu không reset,
    # USB enumerate vẫn OK (/dev/video0..5 có) nhưng camera KHÔNG pump frame
    # → "Frames didn't arrived within 5 seconds" liên tục.
    realsense_node = Node(
        package='realsense2_camera',
        executable='realsense2_camera_node',
        name='camera', namespace='camera',
        parameters=[{
            'align_depth.enable': True,
            'enable_gyro': False, 'enable_accel': False,
            'enable_infra1': False, 'enable_infra2': False,
            # enable_sync=False ⇒ không chờ UVC metadata để khớp timestamp depth/color
            #   → tránh "Non-sequential Video and Metadata v4l buffers" drop frames
            #   trên Jetson (kernel UVC không phát metadata D435i ổn định).
            # Align_depth vẫn chạy đúng vì align dựa vào extrinsic + timestamp xấp xỉ.
            'enable_sync': False,
            'rgb_camera.profile': LaunchConfiguration('rgb_profile'),
            'depth_module.profile': LaunchConfiguration('depth_profile'),
            'initial_reset': ParameterValue(
                LaunchConfiguration('camera_initial_reset'), value_type=bool),
            # Tắt mọi pointcloud/depth phụ trợ trên driver (ta tự gen ở fusion node)
            'pointcloud.enable': False,
            'colorizer.enable': False,
        }],
        output='screen',
    )

    # ── TF: static_transform_publisher x y z qx qy qz qw parent child
    #   parent=camera, child=laser  => biến điểm từ laser sang camera khi lookup(camera, laser).
    # Quaternion (0.5, -0.5, 0.5, 0.5):
    #   laser X (forward) → camera Z (forward); laser Y (left) → -camera X; laser Z (up) → -camera Y
    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_to_laser_tf',
        arguments=[
            LaunchConfiguration('lidar_x'),
            LaunchConfiguration('lidar_y'),
            LaunchConfiguration('lidar_z'),
            '0.5', '-0.5', '0.5', '0.5',
            'camera_depth_optical_frame',
            LaunchConfiguration('laser_frame'),
        ],
        output='screen',
    )

    # ── LiDAR: reset serial then start urg_node2 lifecycle ───────
    if has_urg_node2:
        lidar_reset = ExecuteProcess(
            cmd=['python3', '-c',
                 f"import os,termios,time\n"
                 f"try:\n"
                 f"  fd=os.open('{serial_port}',os.O_RDWR|os.O_NOCTTY|os.O_NONBLOCK)\n"
                 f"  termios.tcflush(fd,termios.TCIOFLUSH)\n"
                 f"  os.write(fd,b'QT\\n');time.sleep(0.12)\n"
                 f"  termios.tcflush(fd,termios.TCIOFLUSH)\n"
                 f"  os.close(fd);print('[lidar_reset] OK')\n"
                 f"except Exception as e: print(f'[lidar_reset] skip: {{e}}')\n"],
            output='screen',
        )
        ld.add_action(lidar_reset)

        lidar_node = LifecycleNode(
            package='urg_node2',
            executable='urg_node2_node',
            name='urg_node2',
            parameters=[urg_params],
            namespace='',
            output='screen',
        )

        # Start urg_node2 AFTER the serial reset finishes
        ld.add_action(RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=lidar_reset,
                on_exit=[lidar_node],
            ),
        ))

        # Lifecycle: process started → configure
        ld.add_action(RegisterEventHandler(
            event_handler=OnProcessStart(
                target_action=lidar_node,
                on_start=[EmitEvent(event=ChangeState(
                    lifecycle_node_matcher=matches_action(lidar_node),
                    transition_id=Transition.TRANSITION_CONFIGURE,
                ))],
            ),
        ))

        # Lifecycle: configured (inactive) → activate
        ld.add_action(RegisterEventHandler(
            event_handler=OnStateTransition(
                target_lifecycle_node=lidar_node,
                start_state='configuring',
                goal_state='inactive',
                entities=[EmitEvent(event=ChangeState(
                    lifecycle_node_matcher=matches_action(lidar_node),
                    transition_id=Transition.TRANSITION_ACTIVATE,
                ))],
            ),
        ))

    # ── Staggered startup để tránh spike GPU/CPU/PSU đồng thời ────
    yolo_node = Node(
        package='realsense_yolo',
        executable='realsense_yolo_node',
        name='realsense_yolo_node',
        parameters=[{
            'color_topic': '/camera/color/image_raw',
            'depth_topic': '/camera/aligned_depth_to_color/image_raw',
            'imgsz': LaunchConfiguration('imgsz'),
            'confidence_threshold': 0.45,
            'iou_threshold': 0.35,
            'duplicate_iou_threshold': 0.55,
            'detections_topic': '/realsense_yolo/detections',
            'yolo_model': LaunchConfiguration('yolo_model'),
            'use_tensorrt': LaunchConfiguration('use_tensorrt'),
            'auto_export_engine': LaunchConfiguration('auto_export_engine'),
            'tensorrt_dla': LaunchConfiguration('tensorrt_dla'),
            'tensorrt_int8': LaunchConfiguration('tensorrt_int8'),
            'use_half': True,
            'max_det': 20,
            'enable_tracking': LaunchConfiguration('enable_tracking'),
            'tracker_config': LaunchConfiguration('tracker_config'),
            'primary_class_ids': [0, 1, 2, 3],
            'others_class_id': 80,
            'others_label': 'others',
            # Tắt vis image publish khi không có RViz → save 30% CPU + 55 MB/s DDS.
            'publish_vis_image': LaunchConfiguration('publish_yolo_image'),
            'vis_image_every_n': 3,   # nếu bật thì cũng chỉ 1/3 frame
        }],
        output='screen',
        additional_env={'LD_PRELOAD': '/lib/aarch64-linux-gnu/libgomp.so.1'},
    )
    fusion_node = Node(
        package='realsense_yolo',
        executable='fusion_3d_node',
        name='fusion_3d_node',
        parameters=[{
            'depth_topic': '/camera/aligned_depth_to_color/image_raw',
            'camera_info_topic': '/camera/aligned_depth_to_color/camera_info',
            'scan_topic': '/scan',
            'detections_topic': '/realsense_yolo/detections',
            'depth_pointcloud_topic': '/realsense_yolo/depth_pointcloud',
            'lidar_pointcloud_topic': '/realsense_yolo/lidar_pointcloud',
            'markers_3d_topic': '/realsense_yolo/detection_boxes_3d',
            'depth_image_plane_topic': '/realsense_yolo/depth_image_plane',
            'camera_frame': 'camera_depth_optical_frame',
            'laser_frame': LaunchConfiguration('laser_frame'),
            'depth_scale': 0.001,
            'depth_max_m': 10.0,
            'depth_step': 8,           # tăng từ 6 → 8 (giảm point cloud size 30%)
            'depth_plane_step': 16,
            'report_period_s': 1.0,
            'lidar_max_range': 10.0,
            'obstacle_topic': '/obstacles',
            'obstacle_rate_hz': 2.0,  # fallback only; obstacles now publish event-driven from _det_cb
            'obstacle_distance_preference': 'camera',
            'detection_stale_s': 0.3,
            'bbox_depth_inner_margin': 0.3,
            'depth_estimate_percentile': 15.0,
            'log_print_others': False,
            # Tắt PointCloud2 nặng khi không có RViz subscriber.
            'publish_pointclouds': LaunchConfiguration('publish_pointclouds'),
            'publish_markers': LaunchConfiguration('use_rviz'),  # markers chỉ cần khi RViz
            'viz_period_s': 0.2,        # 5 Hz (đủ cho RViz, giảm tải khi bật)
        }],
        output='screen',
        additional_env={'LD_PRELOAD': '/lib/aarch64-linux-gnu/libgomp.so.1'},
    )
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_rviz')),
    )

    # ── Startup: camera theo timer; pipeline theo OnProcessStart(camera)+timer ──────────
    # Tránh chờ cố định dài từ t=0 (như 14–22 s cũ): pipeline luôn đi sau camera một khoảng ngắn.
    pipeline_after_camera = TimerAction(
        period=LaunchConfiguration('startup_pipeline_after_realsense_s'),
        actions=[yolo_node, fusion_node, rviz_node],
    )
    ld.add_action(static_tf)
    ld.add_action(TimerAction(
        period=LaunchConfiguration('startup_camera_delay_s'),
        actions=[realsense_node],
    ))
    ld.add_action(RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=realsense_node,
            on_start=[pipeline_after_camera],
        ),
    ))

    return ld
