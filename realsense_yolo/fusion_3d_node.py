#!/usr/bin/env python3
"""
Fusion 3D: Depth point cloud + LiDAR point cloud + YOLO 3D bounding boxes
        + Optimized obstacle topic for obstacle avoidance.

Subscribe: depth image, camera_info, /scan, /realsense_yolo/detections
Publish (visualization, frame camera_depth_optical_frame):
  - /realsense_yolo/depth_pointcloud    (PointCloud2)
  - /realsense_yolo/depth_image_plane   (PointCloud2)
  - /realsense_yolo/lidar_pointcloud    (PointCloud2)
  - /realsense_yolo/detection_boxes_3d  (MarkerArray)
Publish (avoidance input, frame camera_depth_optical_frame — góc trong mặt phẳng ngang XZ, Z tới trước):
  - /obstacles  (utils/ObstacleArray)
    QoS: BestEffort, KeepLast(1), Volatile — always latest, never stale.
"""

import time as _time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image, LaserScan, CameraInfo, PointCloud2, PointField
from std_msgs.msg import Float32MultiArray, Header
from visualization_msgs.msg import MarkerArray, Marker
from geometry_msgs.msg import Point
import numpy as np
import cv2
from cv_bridge import CvBridge
from tf2_ros import Buffer, TransformListener, TransformException

from utils.msg import Obstacle, ObstacleArray

# Low-latency QoS: always take latest, never queue stale data.
# BestEffort subscriber is compatible with both Reliable and BestEffort publishers.
FAST_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
    durability=DurabilityPolicy.VOLATILE,
)

# YOLO node maps any non-primary class to this id; must match realsense_yolo_node others_class_id
OTHERS_CLASS_ID = 80

COCO_NAMES = [
    'person', 'bicycle', 'car', 'motorcycle', 'airplane', 'bus', 'train', 'truck', 'boat',
    'traffic light', 'fire hydrant', 'stop sign', 'parking meter', 'bench', 'bird', 'cat',
    'dog', 'horse', 'sheep', 'cow', 'elephant', 'bear', 'zebra', 'giraffe', 'backpack',
    'umbrella', 'handbag', 'tie', 'suitcase', 'frisbee', 'skis', 'snowboard', 'sports ball',
    'kite', 'baseball bat', 'baseball glove', 'skateboard', 'surfboard', 'tennis racket',
    'bottle', 'wine glass', 'cup', 'fork', 'knife', 'spoon', 'bowl', 'banana', 'apple',
    'sandwich', 'orange', 'broccoli', 'carrot', 'hot dog', 'pizza', 'donut', 'cake', 'chair',
    'couch', 'potted plant', 'bed', 'dining table', 'toilet', 'tv', 'laptop', 'mouse',
    'remote', 'keyboard', 'cell phone', 'microwave', 'oven', 'toaster', 'sink', 'refrigerator',
    'book', 'clock', 'vase', 'scissors', 'teddy bear', 'hair drier', 'toothbrush'
]


def make_point_cloud2(header, frame_id, xyz, rgb=None):
    """Create PointCloud2 from Nx3 xyz and optional Nx3 RGB [0-255]."""
    n = len(xyz)
    if n == 0:
        return None
    valid = np.isfinite(xyz).all(axis=1)
    if not np.any(valid):
        return None
    xyz = np.asarray(xyz[valid], dtype=np.float32)
    if rgb is not None:
        rgb = np.asarray(rgb[valid], dtype=np.uint8)
    n = len(xyz)
    if rgb is None:
        rgb = np.ones((n, 3), dtype=np.uint8) * 128
    rgb_u32 = (
        (rgb[:, 0].astype(np.uint32) << 16) |
        (rgb[:, 1].astype(np.uint32) << 8) |
        rgb[:, 2].astype(np.uint32)
    )
    rgb_f32 = rgb_u32.view(np.float32)
    dtype = [('x', np.float32), ('y', np.float32), ('z', np.float32), ('rgb', np.float32)]
    data = np.empty(n, dtype=dtype)
    data['x'], data['y'], data['z'] = xyz[:, 0], xyz[:, 1], xyz[:, 2]
    data['rgb'] = rgb_f32
    point_step = 16
    msg = PointCloud2()
    msg.header = header
    msg.header.frame_id = frame_id
    msg.height = 1
    msg.width = n
    msg.fields = [
        PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        PointField(name='rgb', offset=12, datatype=PointField.FLOAT32, count=1),
    ]
    msg.is_bigendian = False
    msg.point_step = point_step
    msg.row_step = point_step * n
    msg.data = data.tobytes()
    return msg


class Fusion3DNode(Node):
    def __init__(self):
        super().__init__('fusion_3d_node')

        # ── Parameters ────────────────────────────────────────────
        self.declare_parameter('depth_topic', '/camera/aligned_depth_to_color/image_raw')
        self.declare_parameter('camera_info_topic', '/camera/aligned_depth_to_color/camera_info')
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('detections_topic', '/realsense_yolo/detections')
        self.declare_parameter('depth_pointcloud_topic', '/realsense_yolo/depth_pointcloud')
        self.declare_parameter('lidar_pointcloud_topic', '/realsense_yolo/lidar_pointcloud')
        self.declare_parameter('markers_3d_topic', '/realsense_yolo/detection_boxes_3d')
        self.declare_parameter('depth_image_plane_topic', '/realsense_yolo/depth_image_plane')
        self.declare_parameter('camera_frame', 'camera_depth_optical_frame')
        self.declare_parameter('laser_frame', 'laser')
        self.declare_parameter('depth_scale', 0.001)
        self.declare_parameter('depth_max_m', 10.0)
        self.declare_parameter('depth_step', 4)
        self.declare_parameter('depth_plane_step', 10)
        self.declare_parameter('lidar_max_range', 10.0)
        self.declare_parameter('report_period_s', 1.0)
        self.declare_parameter('obstacle_topic', '/obstacles')
        self.declare_parameter('obstacle_rate_hz', 10.0)
        # obstacle_distance_preference 'camera': có cả depth + LiDAR → fused (ưu tiên LiDAR đã đưa về gốc camera);
        #   chỉ depth → khoảng cách camera. 'lidar': luôn dùng LiDAR (đã đưa về gốc camera) khi có.
        self.declare_parameter('obstacle_distance_preference', 'camera')
        # If no new YOLO message in this time, treat as no detections (fix stale box/log/RViz)
        self.declare_parameter('detection_stale_s', 0.3)
        # Inner crop tỉ lệ (0.3 ⇒ giữ giữa 40%) để loại tường/sàn ở mép bbox khi YOLO bao hơi rộng
        self.declare_parameter('bbox_depth_inner_margin', 0.3)
        # Percentile thấp = lấy bề mặt gần nhất (mặt/ngực người), ít bị kéo bởi nền phía sau
        self.declare_parameter('depth_estimate_percentile', 15.0)
        # Console report: set false to hide 'others' lines
        self.declare_parameter('log_print_others', False)
        # ── Visualization throttling (giảm tải khi không có RViz subscriber) ──
        # publish_pointclouds=False → bỏ 3 PointCloud2 (depth/lidar/depth_plane)
        #   tiết kiệm ~25-40% CPU + bandwidth DDS đáng kể trên Jetson.
        # publish_markers=False    → bỏ MarkerArray (rất nhẹ, default vẫn ON).
        # viz_period_s             → chu kì publish viz (mặc định 0.12 = ~8.3 Hz).
        self.declare_parameter('publish_pointclouds', True)
        self.declare_parameter('publish_markers', True)
        self.declare_parameter('viz_period_s', 0.12)

        self.camera_frame = self._str_param('camera_frame')
        self.laser_frame = self._str_param('laser_frame')
        self.depth_scale = self._dbl_param('depth_scale')
        self.depth_max_m = self._dbl_param('depth_max_m')
        self.depth_step = int(self._int_param('depth_step'))
        self.depth_plane_step = int(self._int_param('depth_plane_step'))
        self.lidar_max_range = self._dbl_param('lidar_max_range')
        self.report_period_s = self._dbl_param('report_period_s')
        obstacle_rate = self._dbl_param('obstacle_rate_hz')
        _dpref = self._str_param('obstacle_distance_preference').strip().lower()
        self._dist_pref = _dpref if _dpref in ('lidar', 'camera') else 'camera'
        self._det_stale_s = max(0.05, self._dbl_param('detection_stale_s'))
        self._bbox_inner_margin = max(0.0, min(0.45, self._dbl_param('bbox_depth_inner_margin')))
        self._depth_pct = max(1.0, min(50.0, self._dbl_param('depth_estimate_percentile')))
        _lo = self.get_parameter('log_print_others').value
        self._log_others = (
            bool(_lo) if isinstance(_lo, bool)
            else str(_lo).strip().lower() in ('1', 'true', 'yes', 'on')
        )
        self._publish_pcs = bool(self.get_parameter('publish_pointclouds').value)
        self._publish_markers_flag = bool(self.get_parameter('publish_markers').value)
        self._viz_period_s = max(0.05, float(self._dbl_param('viz_period_s')))

        # ── Sensor state ──────────────────────────────────────────
        self.bridge = CvBridge()
        self._depth = None
        self._depth_header = None
        self._camera_info = None
        self._scan = None
        self._detections = None

        # Dirty flags: skip obstacle recomputation when nothing changed
        self._depth_dirty = False
        self._scan_dirty = False
        self._det_dirty = False

        # Cached static TF (laser↔camera is constant)
        # _R_lc, _tvec_lc: p_laser  = R_lc · p_camera + t_lc   (camera → laser)
        # _R_cl, _tvec_cl: p_camera = R_cl · p_laser  + t_cl   (laser  → camera)  — INVERSE
        self._cached_tf_lc = None
        self._cached_R_lc = None
        self._cached_tvec_lc = None
        self._cached_R_cl = None
        self._cached_tvec_cl = None
        self._tf_cache_attempts = 0

        # Pre-computed LiDAR scan geometry (recompute only when config changes)
        self._scan_n = 0
        self._scan_angles_rad = None
        self._scan_cos = None
        self._scan_sin = None

        # Obstacle publisher state
        self._obs_seq = 0
        self._last_obstacles = []          # cached result for report reuse
        self._lidar_active = False
        self._last_scan_time = 0.0
        self._last_det_time = None  # monotonic; None = never

        # Tracking state — duy trì track_age + last_seen cho mỗi track_id
        # mapping: track_id (>=0) → {first_seen, last_seen, count}
        self._track_history: dict = {}
        # Stride detection cho /realsense_yolo/detections (8 = mới có track_id, 7 = legacy)
        self._det_stride = 0

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._last_tf_warn_sec = -1

        # Cached depth in meters (invalidated each _depth_cb, avoids repeated float32*scale)
        self._cached_depth_m = None

        # ── Subscribers (FAST_QOS = BestEffort/KeepLast(1) for lowest latency) ──
        self.create_subscription(
            Image, self._str_param('depth_topic'), self._depth_cb, FAST_QOS)
        self.create_subscription(
            CameraInfo, self._str_param('camera_info_topic'), self._camera_info_cb, 10)
        self.create_subscription(
            LaserScan, self._str_param('scan_topic'), self._scan_cb, FAST_QOS)
        self.create_subscription(
            Float32MultiArray, self._str_param('detections_topic'), self._det_cb, FAST_QOS)

        # ── Visualization publishers (standard QoS) ───────────────
        # Chỉ tạo publisher khi có nhu cầu để tránh alloc DDS context không cần.
        if self._publish_pcs:
            self._pub_depth_pc = self.create_publisher(
                PointCloud2, self._str_param('depth_pointcloud_topic'), 10)
            self._pub_lidar_pc = self.create_publisher(
                PointCloud2, self._str_param('lidar_pointcloud_topic'), 10)
            self._pub_depth_plane = self.create_publisher(
                PointCloud2, self._str_param('depth_image_plane_topic'), 10)
        else:
            self._pub_depth_pc = None
            self._pub_lidar_pc = None
            self._pub_depth_plane = None
        self._pub_markers = self.create_publisher(
            MarkerArray, self._str_param('markers_3d_topic'), 10
        ) if self._publish_markers_flag else None

        # ── Obstacle publisher (optimized QoS for avoidance) ──────
        obs_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self._pub_obstacles = self.create_publisher(
            ObstacleArray, self._str_param('obstacle_topic'), obs_qos)

        # ── Timers ────────────────────────────────────────────────
        # Chỉ chạy viz timer khi có publisher viz nào bật, tiết kiệm CPU.
        if self._publish_pcs or self._publish_markers_flag:
            self._timer_viz = self.create_timer(self._viz_period_s, self._publish_3d)
        else:
            self._timer_viz = None
        # Obstacles are published EVENT-DRIVEN from _det_cb (near-zero latency).
        # Timer is a low-rate FALLBACK for stale detection cleanup only.
        obs_fallback_period = max(0.1, 1.0 / max(1.0, obstacle_rate))
        self._timer_obs = self.create_timer(obs_fallback_period, self._publish_obstacles)
        self._timer_report = self.create_timer(
            max(0.5, self.report_period_s), self._report_objects)

        self.get_logger().info(
            f'Fusion3D: obstacle topic={self._str_param("obstacle_topic")} '
            f'@ {obstacle_rate:.0f} Hz, QoS=BestEffort/KeepLast(1), '
            f'pcs={"ON" if self._publish_pcs else "OFF"}, '
            f'markers={"ON" if self._publish_markers_flag else "OFF"}, '
            f'distance={self._dist_pref} (fused=lidar range; chỉ depth=khoảng cách camera)')

    # ── Param helpers ─────────────────────────────────────────────
    def _str_param(self, name):
        return self.get_parameter(name).get_parameter_value().string_value

    def _dbl_param(self, name):
        return self.get_parameter(name).get_parameter_value().double_value

    def _int_param(self, name):
        return self.get_parameter(name).get_parameter_value().integer_value

    # ── Callbacks (with dirty flags) ──────────────────────────────
    def _depth_cb(self, msg):
        try:
            self._depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            self._depth_header = msg.header
            self._depth_dirty = True
            self._cached_depth_m = None
        except Exception as e:
            self.get_logger().error(f'Depth cb: {e}')

    def _camera_info_cb(self, msg):
        self._camera_info = msg

    def _scan_cb(self, msg):
        self._scan = msg
        self._scan_dirty = True
        self._last_scan_time = _time.monotonic()
        n = len(msg.ranges)
        if n != self._scan_n:
            self._scan_n = n
            angles = np.arange(n, dtype=np.float64) * msg.angle_increment + msg.angle_min
            self._scan_angles_rad = angles
            self._scan_cos = np.cos(angles)
            self._scan_sin = np.sin(angles)

    def _det_cb(self, msg):
        self._detections = msg
        self._det_dirty = True
        self._last_det_time = _time.monotonic()
        # Event-driven: compute & publish obstacles IMMEDIATELY on new detection.
        # Eliminates the timer-wait latency (was up to 125ms at 8 Hz).
        self._publish_obstacles()

    def _detections_stale(self):
        if self._last_det_time is None:
            return True
        return (_time.monotonic() - self._last_det_time) > self._det_stale_s

    # ── Detection layout helper (auto-detect 7- vs 8-float stride) ──
    def _det_stride_from_msg(self) -> int:
        msg = self._detections
        if msg is None or not msg.data:
            return 0
        # Ưu tiên đọc từ MultiArrayDimension stride
        try:
            for dim in msg.layout.dim:
                if dim.label and 'val' in dim.label.lower() and dim.size in (7, 8):
                    return int(dim.size)
                if dim.label and 'detect' in dim.label.lower() and dim.stride in (7, 8):
                    return int(dim.stride)
        except Exception:
            pass
        n = len(msg.data)
        if n % 8 == 0 and n // 8 <= 64:
            return 8
        if n % 7 == 0:
            return 7
        return 0

    def _detections_as_array(self):
        """Trả về (-1, 8) float array (track_id = -1 nếu publisher cũ chỉ phát 7 floats)."""
        msg = self._detections
        if msg is None or not msg.data:
            return None
        stride = self._det_stride_from_msg()
        if stride not in (7, 8):
            return None
        if stride != self._det_stride:
            self._det_stride = stride
            self.get_logger().info(
                f'Detections stride={stride} ({"có" if stride == 8 else "không"} track_id)')
        data = np.array(msg.data, dtype=np.float32)
        if data.size < stride:
            return None
        arr = data.reshape(-1, stride)
        if stride == 7:
            pad = np.full((arr.shape[0], 1), -1.0, dtype=np.float32)
            arr = np.concatenate([arr, pad], axis=1)
        return arr

    def _update_track(self, track_id: int, now_mono: float):
        """Cập nhật history một track. Trả về (track_age_frames, reid_active)."""
        if track_id < 0:
            return 0, False
        h = self._track_history.get(track_id)
        if h is None:
            self._track_history[track_id] = {
                'first_seen': now_mono, 'last_seen': now_mono, 'count': 1,
            }
            return 1, True
        h['last_seen'] = now_mono
        h['count'] += 1
        return int(h['count']), True

    def _depth_samples_in_bbox(
        self, depth_m, h, w, x1i, y1i, x2i, y2i,
    ):
        """Depth pixels in inner bbox (reduces side wall/floor mix when subject is off-axis), else full box."""
        wi = int(x2i - x1i)
        hi = int(y2i - y1i)
        if wi < 2 or hi < 2:
            return None, None, None

        for margin in (self._bbox_inner_margin, 0.0):
            if margin > 0:
                ax = int(max(0, min(wi * margin, (wi - 1) * 0.5)))
                ay = int(max(0, min(hi * margin, (hi - 1) * 0.5)))
                ix1, ix2 = x1i + ax, x2i - ax
                iy1, iy2 = y1i + ay, y2i - ay
            else:
                ix1, ix2, iy1, iy2 = x1i, x2i, y1i, y2i
            if ix2 <= ix1 or iy2 <= iy1:
                continue
            ixw, iyh = ix2 - ix1, iy2 - iy1
            sx = max(1, ixw // 20)
            sy = max(1, iyh // 20)
            uu, vv = np.meshgrid(
                np.arange(ix1, ix2 + 1, sx, dtype=np.float32),
                np.arange(iy1, iy2 + 1, sy, dtype=np.float32),
            )
            uu = np.clip(uu, 0, w - 1)
            vv = np.clip(vv, 0, h - 1)
            z = depth_m[vv.astype(int), uu.astype(int)].ravel()
            valid = (z > 0.05) & (z < self.depth_max_m) & np.isfinite(z)
            if int(np.count_nonzero(valid)) < 4 and margin > 0:
                continue
            if int(np.count_nonzero(valid)) < 3:
                continue
            u = uu.ravel()[valid]
            v = vv.ravel()[valid]
            zv = z[valid]
            return zv, u, v
        return None, None, None

    # ── TF cache (static transform — lookup once) ────────────────
    def _ensure_tf_cache(self):
        if self._cached_R_lc is not None:
            return True
        self._tf_cache_attempts += 1
        if self._tf_cache_attempts > 200 and self._tf_cache_attempts % 50 != 0:
            return False
        t = self._lookup_transform(self.laser_frame, self.camera_frame, 0.05)
        if t is None:
            return False
        q = t.transform.rotation
        R_lc = self._quat_to_rot(q.x, q.y, q.z, q.w)
        tvec_lc = np.array([
            t.transform.translation.x,
            t.transform.translation.y,
            t.transform.translation.z,
        ], dtype=np.float64)
        self._cached_R_lc = R_lc
        self._cached_tvec_lc = tvec_lc
        # Inverse: p_camera = R_lc^T · p_laser - R_lc^T · t_lc
        self._cached_R_cl = R_lc.T
        self._cached_tvec_cl = -R_lc.T @ tvec_lc
        self._cached_tf_lc = t
        self.get_logger().info(
            f'TF cached: laser_origin_in_camera={(-R_lc.T @ tvec_lc).round(3).tolist()} m '
            f'(camera_origin_in_laser={tvec_lc.round(3).tolist()} m)')
        return True

    # ── Core obstacle computation ─────────────────────────────────
    def _compute_obstacles(self):
        """Compute obstacle list from current sensor data.
        Returns list of dicts with all fields needed for ObstacleArray."""
        if self._detections is None or self._camera_info is None or self._depth is None:
            return []
        if self._detections_stale():
            return []
        if not self._ensure_tf_cache():
            return []

        R_lc = self._cached_R_lc        # camera → laser
        tvec_lc = self._cached_tvec_lc
        R_cl = self._cached_R_cl        # laser  → camera (inverse)
        tvec_cl = self._cached_tvec_cl

        if self._cached_depth_m is None:
            depth_raw = np.asarray(self._depth, dtype=np.float32)
            if depth_raw.size and np.nanmax(depth_raw) > 100:
                self._cached_depth_m = depth_raw * self.depth_scale
            else:
                self._cached_depth_m = depth_raw
        depth_m = self._cached_depth_m
        h, w = depth_m.shape[:2]

        K = self._camera_info.k
        fx, fy = float(K[0]), float(K[4])
        cx, cy = float(K[2]), float(K[5])
        if fx == 0.0 or fy == 0.0:
            return []

        # Pre-fetch LiDAR data once
        has_lidar = (
            self._scan is not None
            and self._scan_angles_rad is not None
            and len(self._scan.ranges) > 0
        )
        scan_ranges = None
        scan_angles_rad = None
        if has_lidar:
            scan_ranges = np.array(self._scan.ranges, dtype=np.float32)
            scan_angles_rad = self._scan_angles_rad

        dets = self._detections_as_array()
        if dets is None:
            return []
        obstacles = []

        # Cleanup track history: bỏ những track không thấy quá 2 s
        now_mono = _time.monotonic()
        if self._track_history:
            stale_ids = [tid for tid, h in self._track_history.items()
                         if now_mono - h['last_seen'] > 2.0]
            for tid in stale_ids:
                self._track_history.pop(tid, None)

        for row in dets:
            x1, y1, x2, y2, det_depth_m, cls_id, conf, track_id_f = row
            track_id = int(track_id_f) if np.isfinite(track_id_f) else -1
            x1i = int(np.clip(np.floor(x1), 0, w - 1))
            x2i = int(np.clip(np.ceil(x2), 0, w - 1))
            y1i = int(np.clip(np.floor(y1), 0, h - 1))
            y2i = int(np.clip(np.ceil(y2), 0, h - 1))
            if x2i <= x1i or y2i <= y1i:
                continue

            z, u, v = self._depth_samples_in_bbox(depth_m, h, w, x1i, y1i, x2i, y2i)
            if z is None or z.size < 3:
                continue
            x_cam = (u - cx) * z / fx
            y_cam = (v - cy) * z / fy
            pts_cam = np.stack([x_cam, y_cam, z], axis=0)         # 3×N, optical: X phải, Y xuống, Z tới trước
            pts_laser = (R_lc @ pts_cam).T + tvec_lc              # N×3

            # Góc cho /obstacles: trong frame camera (trùng “phía trước” robot), trái > 0 — atan2(-X, Z)
            ang_cam = np.arctan2(-pts_cam[0, :], pts_cam[2, :])
            angle_max_rad = float(np.max(ang_cam))
            angle_min_rad = float(np.min(ang_cam))
            angle_center = float(np.mean(ang_cam))
            # Góc trong frame laser — chỉ để khớp sector với /scan (LaserScan vẫn là laser)
            ang_laser = np.arctan2(pts_laser[:, 1], pts_laser[:, 0])
            angle_max_laser = float(np.max(ang_laser))
            angle_min_laser = float(np.min(ang_laser))
            # Khoảng cách MẶT PHẲNG NGANG (XZ) từ gốc camera → loại bias 23 cm theo Y của LiDAR
            horiz_dists = np.sqrt(pts_cam[0, :] ** 2 + pts_cam[2, :] ** 2)
            # Bề mặt gần nhất trong inner-bbox: percentile thấp + median nhóm gần (tương đương 'closest cluster')
            cam_depth_m = float(np.percentile(z, self._depth_pct))
            if not (np.isfinite(cam_depth_m) and cam_depth_m > 0.01):
                cam_depth_m = float(np.median(z)) if z.size else 0.0
            cam_horiz_m = float(np.percentile(horiz_dists, self._depth_pct))
            if not (np.isfinite(cam_horiz_m) and cam_horiz_m > 0.01):
                cam_horiz_m = float(np.median(horiz_dists)) if horiz_dists.size else cam_depth_m
            # Refine: median của 30% pixel gần nhất → ổn định, ít bị kéo bởi nền/sàn còn sót
            n_close = max(3, int(0.3 * horiz_dists.size))
            closest_idx = np.argpartition(horiz_dists, n_close - 1)[:n_close]
            cam_horiz_m = float(np.median(horiz_dists[closest_idx]))

            # Estimate width: arc length at median distance
            median_dist = float(np.median(horiz_dists))
            width_m = median_dist * abs(angle_max_rad - angle_min_rad)

            # ── LiDAR sector → khoảng cách tính từ GỐC CAMERA (đã bù lệch 53 cm sau / 23 cm trên) ──
            lidar_distance = -1.0  # dist mét, đã quy về gốc camera (XZ ngang)
            lidar_distance_raw = -1.0  # dist từ gốc laser (debug)
            if has_lidar:
                right_deg = np.degrees(angle_min_laser)
                left_deg = np.degrees(angle_max_laser)
                m = self._angle_mask_deg(
                    np.degrees(scan_angles_rad), right_deg, left_deg)
                r_sec = scan_ranges[m]
                a_sec = scan_angles_rad[m]
                ok = np.isfinite(r_sec) & (r_sec > 0.02) & (r_sec < self.lidar_max_range)
                r_sec, a_sec = r_sec[ok], a_sec[ok]
                if r_sec.size > 0:
                    lidar_distance_raw = float(np.min(r_sec))
                    # Hit của từng tia trong frame laser → chuyển sang camera bằng INVERSE TF
                    # (R_cl, t_cl): p_camera = R_cl · p_laser + t_cl
                    x_l = r_sec * np.cos(a_sec)
                    y_l = r_sec * np.sin(a_sec)
                    z_l = np.zeros_like(r_sec)
                    p_l = np.stack([x_l, y_l, z_l], axis=0)               # 3×N
                    p_c = (R_cl @ p_l).T + tvec_cl                        # N×3 trong frame camera
                    d_cam_xz = np.sqrt(p_c[:, 0] ** 2 + p_c[:, 2] ** 2)
                    lidar_distance = float(np.min(d_cam_xz))

            has_cam = bool(
                np.isfinite(cam_depth_m) and 0.02 < cam_depth_m < self.depth_max_m
            )
            # Logic chọn distance — đều quy chuẩn về GỐC CAMERA (mép trước robot)
            if self._dist_pref == 'lidar' and lidar_distance > 0:
                best_dist = lidar_distance
                source = Obstacle.SOURCE_LIDAR
            elif self._dist_pref == 'camera' and has_cam:
                if lidar_distance > 0:
                    # Fused: ưu tiên LiDAR (đã quy về gốc camera) — chính xác hơn depth ở khoảng xa
                    best_dist = lidar_distance
                    source = Obstacle.SOURCE_FUSED
                else:
                    best_dist = cam_horiz_m
                    source = Obstacle.SOURCE_DEPTH
            elif lidar_distance > 0:
                best_dist = lidar_distance
                source = Obstacle.SOURCE_LIDAR
            elif has_cam:
                best_dist = cam_horiz_m
                source = Obstacle.SOURCE_DEPTH
            else:
                best_dist = max(float(det_depth_m), 0.0) if det_depth_m > 0 else 0.0
                source = Obstacle.SOURCE_DEPTH

            cls_id_int = int(cls_id)
            track_age, reid_active = self._update_track(track_id, now_mono)
            obstacles.append({
                'distance': best_dist,
                'angle': angle_center,
                'angle_min': angle_min_rad,
                'angle_max': angle_max_rad,
                'width': width_m,
                'depth_distance': cam_horiz_m if has_cam else -1.0,
                'lidar_distance': lidar_distance,
                'lidar_distance_raw': lidar_distance_raw,
                'class_id': cls_id_int,
                'class_name': self._class_name(cls_id_int),
                'confidence': float(conf),
                'source': source,
                'track_id': track_id,
                'track_age': track_age,
                'reid_active': reid_active,
            })

        obstacles.sort(key=lambda o: o['distance'])
        return obstacles

    # ── Obstacle publisher (high-rate, optimized QoS) ─────────────
    def _publish_obstacles(self):
        stale = self._detections_stale()
        any_dirty = self._depth_dirty or self._scan_dirty or self._det_dirty
        if stale or any_dirty or self._last_obstacles is None:
            self._depth_dirty = False
            self._scan_dirty = False
            self._det_dirty = False
            t0 = _time.monotonic()
            obstacles = self._compute_obstacles()
            compute_ms = (_time.monotonic() - t0) * 1000.0
            self._last_obstacles = obstacles
            self._last_compute_ms = compute_ms
        else:
            obstacles = self._last_obstacles

        self._lidar_active = (_time.monotonic() - self._last_scan_time) < 2.0

        msg = ObstacleArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.camera_frame
        msg.seq = self._obs_seq
        self._obs_seq += 1
        msg.lidar_active = self._lidar_active
        msg.compute_ms = getattr(self, '_last_compute_ms', 0.0)

        for o in obstacles:
            obs = Obstacle()
            obs.distance = o['distance']
            obs.angle = o['angle']
            obs.angle_min = o['angle_min']
            obs.angle_max = o['angle_max']
            obs.width = o['width']
            obs.depth_distance = o['depth_distance']
            obs.lidar_distance = o['lidar_distance']
            obs.class_id = o['class_id']
            obs.class_name = o['class_name']
            obs.confidence = o['confidence']
            obs.source = o['source']
            obs.track_id = int(o.get('track_id', -1))
            obs.track_age = int(max(0, o.get('track_age', 0)))
            obs.reid_active = bool(o.get('reid_active', False))
            msg.obstacles.append(obs)

        self._pub_obstacles.publish(msg)
        self._obs_publish_count = getattr(self, '_obs_publish_count', 0) + 1

    # ── Terminal report (low-rate, human-readable) ────────────────
    def _report_objects(self):
        obstacles = self._last_obstacles
        if not obstacles:
            return
        shown = [
            o for o in obstacles
            if self._log_others or o['class_id'] != OTHERS_CLASS_ID
        ]
        if not shown:
            return
        src_names = {Obstacle.SOURCE_DEPTH: 'depth',
                     Obstacle.SOURCE_LIDAR: 'lidar',
                     Obstacle.SOURCE_FUSED: 'fused'}
        obs_hz = getattr(self, '_obs_publish_count', 0)
        self.get_logger().info(
            f'--- Obstacles ({len(shown)}) seq={self._obs_seq} '
            f'lidar={"ON" if self._lidar_active else "OFF"} '
            f'compute={getattr(self, "_last_compute_ms", 0):.1f}ms '
            f'obs_hz≈{obs_hz}/s ---')
        self._obs_publish_count = 0
        for o in shown:
            dc = o['depth_distance']
            d_cam = f'{dc:.2f}' if dc is not None and dc > 0 else 'N/A'
            ldm = o['lidar_distance']
            lid = f'{ldm:.2f}' if ldm is not None and ldm > 0 else 'N/A'
            ldr = o.get('lidar_distance_raw', -1.0)
            lid_raw = f'{ldr:.2f}' if ldr is not None and ldr > 0 else 'N/A'
            tid = o.get('track_id', -1)
            tid_str = f'#{tid}' if tid >= 0 else '#?'
            tag_age = f"age={o.get('track_age', 0)}"
            self.get_logger().info(
                f"  {tid_str} {o['class_name']} conf={o['confidence']:.2f} "
                f"dist={o['distance']:.2f}m src={src_names.get(o['source'], '?')} "
                f"angle=[{np.degrees(o['angle_min']):.1f}°,{np.degrees(o['angle_max']):.1f}°] "
                f"w={o['width']:.2f}m "
                f"(depth_cam={d_cam}m lidar_cam={lid}m lidar_raw={lid_raw}m {tag_age})")

    # ── Visualization publisher (unchanged logic) ─────────────────
    def _publish_3d(self):
        stamp = self.get_clock().now().to_msg()
        header = Header()
        header.stamp = stamp
        header.frame_id = self.camera_frame

        # 1) Depth → PointCloud2
        if self._publish_pcs and self._depth is not None and self._camera_info is not None:
            K = self._camera_info.k
            fx, fy = float(K[0]), float(K[4])
            cx, cy = float(K[2]), float(K[5])
            h, w = self._depth.shape[:2]
            depth = np.asarray(self._depth, dtype=np.float32)
            if depth.size and np.nanmax(depth) > 100:
                depth = depth * self.depth_scale
            step = max(1, self.depth_step)
            u = np.arange(0, w, step)
            v = np.arange(0, h, step)
            uu, vv = np.meshgrid(u, v)
            z = depth[vv, uu].ravel()
            valid = (z > 0.01) & (z < self.depth_max_m) & np.isfinite(z)
            z = z[valid]
            uu, vv = uu.ravel()[valid], vv.ravel()[valid]
            x = (uu - cx) * z / fx
            y = (vv - cy) * z / fy
            xyz = np.stack([x, y, z], axis=1)
            if len(xyz) > 0:
                rgb = np.zeros((len(xyz), 3), dtype=np.uint8)
                rgb[:, 0] = np.clip(255 * (1 - z / self.depth_max_m), 0, 255).astype(np.uint8)
                rgb[:, 2] = np.clip(255 * z / self.depth_max_m, 0, 255).astype(np.uint8)
                pc = make_point_cloud2(header, self.camera_frame, xyz, rgb)
                if pc:
                    self._pub_depth_pc.publish(pc)

        # 1b) Depth map surface (3D JET colormap)
        if self._publish_pcs and self._depth is not None and self._camera_info is not None:
            K = self._camera_info.k
            fx, fy = float(K[0]), float(K[4])
            cx, cy = float(K[2]), float(K[5])
            h, w = self._depth.shape[:2]
            depth_raw = np.asarray(self._depth, dtype=np.float32)
            if depth_raw.size and np.nanmax(depth_raw) > 100:
                depth_m = depth_raw * self.depth_scale
            else:
                depth_m = depth_raw.copy()
            step = max(1, self.depth_plane_step)
            u = np.arange(0, w, step, dtype=np.float32)
            v = np.arange(0, h, step, dtype=np.float32)
            uu, vv = np.meshgrid(u, v)
            z = depth_m[vv.astype(int), uu.astype(int)].ravel()
            valid = (z > 0.01) & (z < self.depth_max_m) & np.isfinite(z)
            z = z[valid]
            uu_flat = uu.ravel()[valid]
            vv_flat = vv.ravel()[valid]
            x = (uu_flat - cx) * z / fx
            y = (vv_flat - cy) * z / fy
            xyz_surf = np.stack([x, y, z], axis=1).astype(np.float32)
            depth_8 = np.clip(255.0 * z / self.depth_max_m, 0, 255).astype(np.uint8)
            colormap_bgr = cv2.applyColorMap(depth_8.reshape(-1, 1), cv2.COLORMAP_JET)
            rgb_surf = colormap_bgr.reshape(-1, 3)[:, ::-1]
            det_arr = self._detections_as_array() if not self._detections_stale() else None
            if det_arr is not None:
                for d in det_arr:
                    x1, y1, x2, y2, _, cls_id, _, _ = d
                    cls_rgb = self._class_color_rgb(int(cls_id)).astype(np.float32)
                    in_box = (
                        (uu_flat >= x1) & (uu_flat <= x2) &
                        (vv_flat >= y1) & (vv_flat <= y2)
                    )
                    if np.any(in_box):
                        rgb_surf[in_box] = (
                            0.35 * rgb_surf[in_box].astype(np.float32) + 0.65 * cls_rgb
                        ).astype(np.uint8)
            pc_surf = make_point_cloud2(header, self.camera_frame, xyz_surf, rgb_surf)
            if pc_surf:
                self._pub_depth_plane.publish(pc_surf)

        # 2) LiDAR → PointCloud2 (transform to camera frame)
        if self._publish_pcs and self._scan is not None and self._scan_angles_rad is not None:
            ranges = np.array(self._scan.ranges, dtype=np.float32)
            valid = np.isfinite(ranges) & (ranges > 0) & (ranges < self.lidar_max_range)
            r = ranges[valid]
            if r.size > 0:
                cos_v = self._scan_cos[valid]
                sin_v = self._scan_sin[valid]
                x_l = r * cos_v
                y_l = r * sin_v
                z_l = np.zeros_like(r)
                p_l = np.stack([x_l, y_l, z_l], axis=0)
                t = self._get_lidar_to_camera_tf()
                if t is not None:
                    qx, qy, qz, qw = (t.transform.rotation.x, t.transform.rotation.y,
                                       t.transform.rotation.z, t.transform.rotation.w)
                    R = self._quat_to_rot(qx, qy, qz, qw)
                    tvec = np.array([t.transform.translation.x,
                                     t.transform.translation.y,
                                     t.transform.translation.z])
                    p_c = (R @ p_l).T + tvec
                    rgb = np.full((len(p_c), 3), 0, dtype=np.uint8)
                    rgb[:, 1] = 255
                    pc = make_point_cloud2(header, self.camera_frame, p_c, rgb)
                    if pc:
                        self._pub_lidar_pc.publish(pc)

        # 3) Detections → 3D markers (DELETEALL mỗi lần để RViz không “dính” hộp cũ)
        if self._publish_markers_flag and self._pub_markers is not None and self._camera_info is not None:
            K = self._camera_info.k
            fx, fy = float(K[0]), float(K[4])
            cx, cy = float(K[2]), float(K[5])
            markers = MarkerArray()
            clr = Marker()
            clr.header = header
            clr.ns = 'bbox3d'
            clr.id = 0
            clr.action = Marker.DELETEALL
            markers.markers.append(clr)
            det_arr = self._detections_as_array() if not self._detections_stale() else None
            if det_arr is not None:
                for i in range(det_arr.shape[0]):
                    x1, y1, x2, y2, depth_m, cls_id, conf, track_id_f = det_arr[i]
                    if depth_m <= 0:
                        continue
                    cx_p = (x1 + x2) / 2
                    cy_p = (y1 + y2) / 2
                    center_x = (cx_p - cx) * depth_m / fx
                    center_y = (cy_p - cy) * depth_m / fy
                    w_px = x2 - x1
                    h_px = y2 - y1
                    if w_px <= 1 or h_px <= 1:
                        continue
                    w_m = max(0.03, w_px * depth_m / fx)
                    h_m = max(0.03, h_px * depth_m / fy)
                    d_m = float(np.clip(0.5 * (w_m + h_m), 0.05, 1.0))
                    center_z = depth_m + d_m / 2.0
                    half = np.array([w_m / 2, h_m / 2, d_m / 2])
                    c = np.array([center_x, center_y, center_z])
                    edges = [
                        ([-1, -1, -1], [1, -1, -1]), ([-1, -1, -1], [-1, 1, -1]),
                        ([-1, -1, -1], [-1, -1, 1]), ([1, -1, -1], [1, 1, -1]),
                        ([1, -1, -1], [1, -1, 1]), ([-1, 1, -1], [1, 1, -1]),
                        ([-1, 1, -1], [-1, 1, 1]), ([1, 1, -1], [1, 1, 1]),
                        ([1, -1, 1], [1, 1, 1]), ([1, -1, 1], [-1, -1, 1]),
                        ([-1, 1, 1], [1, 1, 1]), ([-1, 1, 1], [-1, -1, 1]),
                    ]
                    mk = Marker()
                    mk.header = header
                    mk.ns = 'bbox3d'
                    mk.id = i + 1
                    mk.type = Marker.LINE_LIST
                    mk.action = Marker.ADD
                    mk.scale.x = 0.02
                    cls_rgb = self._class_color_rgb(int(cls_id))
                    mk.color.r = float(cls_rgb[0]) / 255.0
                    mk.color.g = float(cls_rgb[1]) / 255.0
                    mk.color.b = float(cls_rgb[2]) / 255.0
                    mk.color.a = 1.0
                    for e1, e2 in edges:
                        p1 = c + np.array(e1) * half
                        p2 = c + np.array(e2) * half
                        mk.points.append(Point(x=float(p1[0]), y=float(p1[1]), z=float(p1[2])))
                        mk.points.append(Point(x=float(p2[0]), y=float(p2[1]), z=float(p2[2])))
                    markers.markers.append(mk)
            self._pub_markers.publish(markers)

    # ── Helpers ───────────────────────────────────────────────────
    def _get_lidar_to_camera_tf(self):
        """Get TF laser→camera (for pointcloud viz). Uses cache if available."""
        if self._cached_tf_lc is not None:
            # Invert cached laser←camera to get camera←laser
            # But we need camera←laser for viz. Let's look up directly.
            pass
        source_frames = []
        if self._scan and self._scan.header.frame_id:
            source_frames.append(self._scan.header.frame_id)
        if self.laser_frame and self.laser_frame not in source_frames:
            source_frames.append(self.laser_frame)
        for src in source_frames:
            try:
                return self._tf_buffer.lookup_transform(
                    self.camera_frame, src,
                    rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=0.05))
            except TransformException:
                pass
        now_sec = int(self.get_clock().now().nanoseconds / 1e9)
        if now_sec != self._last_tf_warn_sec and now_sec % 5 == 0:
            self._last_tf_warn_sec = now_sec
            self.get_logger().warn(
                f'Cannot transform LiDAR to {self.camera_frame}. '
                f'scan frame={self._scan.header.frame_id if self._scan else "?"}, '
                f'laser_frame={self.laser_frame}')
        return None

    def _lookup_transform(self, target_frame, source_frame, timeout_s=0.1):
        try:
            return self._tf_buffer.lookup_transform(
                target_frame, source_frame, rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=timeout_s))
        except TransformException:
            return None

    @staticmethod
    def _class_name(cls_id):
        i = int(cls_id)
        if i == OTHERS_CLASS_ID:
            return 'others'
        return COCO_NAMES[i] if 0 <= i < len(COCO_NAMES) else f'class_{i}'

    @staticmethod
    def _class_color_rgb(cls_id):
        hue = (int(cls_id) * 37) % 180
        hsv = np.uint8([[[hue, 220, 255]]])
        bgr = cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)[0, 0]
        return np.array([int(bgr[2]), int(bgr[1]), int(bgr[0])], dtype=np.uint8)

    @staticmethod
    def _quat_to_rot(qx, qy, qz, qw):
        return np.array([
            [1 - 2*qy*qy - 2*qz*qz, 2*qx*qy - 2*qw*qz, 2*qx*qz + 2*qw*qy],
            [2*qx*qy + 2*qw*qz, 1 - 2*qx*qx - 2*qz*qz, 2*qy*qz - 2*qw*qx],
            [2*qx*qz - 2*qw*qy, 2*qy*qz + 2*qw*qx, 1 - 2*qx*qx - 2*qy*qy],
        ], dtype=np.float64)

    @staticmethod
    def _angle_mask_deg(angles_deg, right_deg, left_deg):
        if right_deg <= left_deg:
            return (angles_deg >= right_deg) & (angles_deg <= left_deg)
        return (angles_deg >= right_deg) | (angles_deg <= left_deg)


def main(args=None):
    rclpy.init(args=args)
    node = Fusion3DNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
