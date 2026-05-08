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
Publish (avoidance input, frame laser):
  - /obstacles  (delivery_interfaces/ObstacleArray)
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

from delivery_interfaces.msg import Obstacle, ObstacleArray

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

        self.camera_frame = self._str_param('camera_frame')
        self.laser_frame = self._str_param('laser_frame')
        self.depth_scale = self._dbl_param('depth_scale')
        self.depth_max_m = self._dbl_param('depth_max_m')
        self.depth_step = int(self._int_param('depth_step'))
        self.depth_plane_step = int(self._int_param('depth_plane_step'))
        self.lidar_max_range = self._dbl_param('lidar_max_range')
        self.report_period_s = self._dbl_param('report_period_s')
        obstacle_rate = self._dbl_param('obstacle_rate_hz')

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
        self._cached_tf_lc = None          # laser ← camera
        self._cached_R_lc = None
        self._cached_tvec_lc = None
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

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._last_tf_warn_sec = -1

        # ── Subscribers ───────────────────────────────────────────
        self.create_subscription(
            Image, self._str_param('depth_topic'), self._depth_cb, 10)
        self.create_subscription(
            CameraInfo, self._str_param('camera_info_topic'), self._camera_info_cb, 10)
        self.create_subscription(
            LaserScan, self._str_param('scan_topic'), self._scan_cb, 10)
        self.create_subscription(
            Float32MultiArray, self._str_param('detections_topic'), self._det_cb, 10)

        # ── Visualization publishers (standard QoS) ───────────────
        self._pub_depth_pc = self.create_publisher(
            PointCloud2, self._str_param('depth_pointcloud_topic'), 10)
        self._pub_lidar_pc = self.create_publisher(
            PointCloud2, self._str_param('lidar_pointcloud_topic'), 10)
        self._pub_markers = self.create_publisher(
            MarkerArray, self._str_param('markers_3d_topic'), 10)
        self._pub_depth_plane = self.create_publisher(
            PointCloud2, self._str_param('depth_image_plane_topic'), 10)

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
        self._timer_viz = self.create_timer(0.12, self._publish_3d)
        obs_period = max(0.02, 1.0 / max(1.0, obstacle_rate))
        self._timer_obs = self.create_timer(obs_period, self._publish_obstacles)
        self._timer_report = self.create_timer(
            max(0.5, self.report_period_s), self._report_objects)

        self.get_logger().info(
            f'Fusion3D: obstacle topic={self._str_param("obstacle_topic")} '
            f'@ {obstacle_rate:.0f} Hz, QoS=BestEffort/KeepLast(1)')

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
        self._cached_R_lc = self._quat_to_rot(q.x, q.y, q.z, q.w)
        self._cached_tvec_lc = np.array([
            t.transform.translation.x,
            t.transform.translation.y,
            t.transform.translation.z,
        ], dtype=np.float64)
        self._cached_tf_lc = t
        self.get_logger().info('TF laser←camera cached (static)')
        return True

    # ── Core obstacle computation ─────────────────────────────────
    def _compute_obstacles(self):
        """Compute obstacle list from current sensor data.
        Returns list of dicts with all fields needed for ObstacleArray."""
        if self._detections is None or self._camera_info is None or self._depth is None:
            return []
        if not self._ensure_tf_cache():
            return []

        R_lc = self._cached_R_lc
        tvec_lc = self._cached_tvec_lc

        depth_raw = np.asarray(self._depth, dtype=np.float32)
        if depth_raw.size and np.nanmax(depth_raw) > 100:
            depth_m = depth_raw * self.depth_scale
        else:
            depth_m = depth_raw
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

        data = np.array(self._detections.data, dtype=np.float32)
        if data.size < 7:
            return []
        dets = data.reshape(-1, 7)
        obstacles = []

        for row in dets:
            x1, y1, x2, y2, det_depth_m, cls_id, conf = row
            x1i = int(np.clip(np.floor(x1), 0, w - 1))
            x2i = int(np.clip(np.ceil(x2), 0, w - 1))
            y1i = int(np.clip(np.floor(y1), 0, h - 1))
            y2i = int(np.clip(np.ceil(y2), 0, h - 1))
            if x2i <= x1i or y2i <= y1i:
                continue

            sx = max(2, (x2i - x1i) // 24)
            sy = max(2, (y2i - y1i) // 24)
            uu, vv = np.meshgrid(
                np.arange(x1i, x2i + 1, sx, dtype=np.float32),
                np.arange(y1i, y2i + 1, sy, dtype=np.float32),
            )
            z = depth_m[vv.astype(int), uu.astype(int)].ravel()
            valid = (z > 0.05) & (z < self.depth_max_m) & np.isfinite(z)
            if np.count_nonzero(valid) < 6:
                continue
            z = z[valid]
            u = uu.ravel()[valid]
            v = vv.ravel()[valid]
            x_cam = (u - cx) * z / fx
            y_cam = (v - cy) * z / fy
            pts_cam = np.stack([x_cam, y_cam, z], axis=0)         # 3×N
            pts_laser = (R_lc @ pts_cam).T + tvec_lc              # N×3

            ang_rad = np.arctan2(pts_laser[:, 1], pts_laser[:, 0])
            angle_max_rad = float(np.max(ang_rad))   # left edge
            angle_min_rad = float(np.min(ang_rad))   # right edge
            angle_center = float(np.mean(ang_rad))
            horiz_dists = np.linalg.norm(pts_laser[:, :2], axis=1)
            depth_distance = float(np.min(horiz_dists))

            # Estimate width: arc length at median distance
            median_dist = float(np.median(horiz_dists))
            width_m = median_dist * abs(angle_max_rad - angle_min_rad)

            lidar_distance = -1.0
            if has_lidar:
                right_deg = np.degrees(angle_min_rad)
                left_deg = np.degrees(angle_max_rad)
                m = self._angle_mask_deg(
                    np.degrees(scan_angles_rad), right_deg, left_deg)
                r = scan_ranges[m]
                r_valid = r[np.isfinite(r) & (r > 0.02) & (r < self.lidar_max_range)]
                if r_valid.size > 0:
                    lidar_distance = float(np.min(r_valid))

            if lidar_distance > 0:
                best_dist = lidar_distance
                source = Obstacle.SOURCE_LIDAR
            else:
                best_dist = depth_distance if depth_distance > 0 else float(det_depth_m)
                source = Obstacle.SOURCE_DEPTH

            cls_id_int = int(cls_id)
            obstacles.append({
                'distance': best_dist,
                'angle': angle_center,
                'angle_min': angle_min_rad,
                'angle_max': angle_max_rad,
                'width': width_m,
                'depth_distance': depth_distance,
                'lidar_distance': lidar_distance,
                'class_id': cls_id_int,
                'class_name': self._class_name(cls_id_int),
                'confidence': float(conf),
                'source': source,
            })

        obstacles.sort(key=lambda o: o['distance'])
        return obstacles

    # ── Obstacle publisher (high-rate, optimized QoS) ─────────────
    def _publish_obstacles(self):
        any_dirty = self._depth_dirty or self._scan_dirty or self._det_dirty
        if not any_dirty and self._last_obstacles is not None:
            # No new data → re-publish cached result with updated timestamp
            if not self._last_obstacles:
                return
            obstacles = self._last_obstacles
        else:
            self._depth_dirty = False
            self._scan_dirty = False
            self._det_dirty = False
            t0 = _time.monotonic()
            obstacles = self._compute_obstacles()
            compute_ms = (_time.monotonic() - t0) * 1000.0
            self._last_obstacles = obstacles
            self._last_compute_ms = compute_ms

        self._lidar_active = (_time.monotonic() - self._last_scan_time) < 2.0

        msg = ObstacleArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.laser_frame
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
            msg.obstacles.append(obs)

        self._pub_obstacles.publish(msg)

    # ── Terminal report (low-rate, human-readable) ────────────────
    def _report_objects(self):
        obstacles = self._last_obstacles
        if not obstacles:
            return
        src_names = {Obstacle.SOURCE_DEPTH: 'depth',
                     Obstacle.SOURCE_LIDAR: 'lidar',
                     Obstacle.SOURCE_FUSED: 'fused'}
        self.get_logger().info(
            f'--- Obstacles ({len(obstacles)}) seq={self._obs_seq} '
            f'lidar={"ON" if self._lidar_active else "OFF"} '
            f'compute={getattr(self, "_last_compute_ms", 0):.1f}ms ---')
        for o in obstacles:
            self.get_logger().info(
                f"  {o['class_name']} conf={o['confidence']:.2f} "
                f"dist={o['distance']:.2f}m src={src_names.get(o['source'], '?')} "
                f"angle=[{np.degrees(o['angle_min']):.1f}°,{np.degrees(o['angle_max']):.1f}°] "
                f"w={o['width']:.2f}m "
                f"(depth={o['depth_distance']:.2f}m lidar={'%.2f' % o['lidar_distance'] if o['lidar_distance'] > 0 else 'N/A'}m)")

    # ── Visualization publisher (unchanged logic) ─────────────────
    def _publish_3d(self):
        stamp = self.get_clock().now().to_msg()
        header = Header()
        header.stamp = stamp
        header.frame_id = self.camera_frame

        # 1) Depth → PointCloud2
        if self._depth is not None and self._camera_info is not None:
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
        if self._depth is not None and self._camera_info is not None:
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
            if self._detections is not None and len(self._detections.data) >= 7:
                det_arr = np.array(self._detections.data, dtype=np.float32).reshape(-1, 7)
                for d in det_arr:
                    x1, y1, x2, y2, _, cls_id, _ = d
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
        if self._scan is not None and self._scan_angles_rad is not None:
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

        # 3) Detections → 3D bounding box markers
        if (self._detections is not None and self._camera_info is not None
                and len(self._detections.data) >= 7):
            K = self._camera_info.k
            fx, fy = float(K[0]), float(K[4])
            cx, cy = float(K[2]), float(K[5])
            data = self._detections.data
            n = len(data) // 7
            markers = MarkerArray()
            for i in range(n):
                x1, y1, x2, y2, depth_m, cls_id, conf = data[i*7:(i+1)*7]
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
                mk.id = i
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
            if markers.markers:
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
