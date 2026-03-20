#!/usr/bin/env python3
"""
Fusion 3D: Depth point cloud + depth map surface (3D) + LiDAR point cloud + YOLO 3D bounding boxes.

- Subscribe: depth image, camera_info, /scan, /realsense_yolo/detections
- Publish (frame camera_depth_optical_frame):
  - /realsense_yolo/depth_pointcloud (PointCloud2) - điểm 3D từ depth (đỏ gần, xanh xa)
  - /realsense_yolo/depth_image_plane (PointCloud2) - bề mặt độ sâu trong 3D, màu JET (depth map trong view 3D)
  - /realsense_yolo/lidar_pointcloud (PointCloud2) - từ /scan (đã transform sang camera)
  - /realsense_yolo/detection_boxes_3d (MarkerArray) - hộp 3D cho mỗi detection

Depth map hiển thị trong môi trường 3D RViz (không dùng cửa sổ Image bên trái).
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, LaserScan, CameraInfo, PointCloud2, PointField
from std_msgs.msg import Float32MultiArray, Header
from visualization_msgs.msg import MarkerArray, Marker
import numpy as np
import cv2
from cv_bridge import CvBridge
from tf2_ros import Buffer, TransformListener, TransformException

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
    """Create PointCloud2 from Nx3 xyz and optional Nx3 RGB [0-255].
    Uses standard x,y,z,rgb(float32 packed) layout for RViz stability."""
    n = len(xyz)
    if n == 0:
        return None
    # Loại bỏ inf/nan để RViz không crash (exit -11)
    valid = np.isfinite(xyz).all(axis=1)
    if not np.any(valid):
        return None
    xyz = np.asarray(xyz[valid], dtype=np.float32)
    if rgb is not None:
        rgb = np.asarray(rgb[valid], dtype=np.uint8)
    n = len(xyz)
    if rgb is None:
        rgb = np.ones((n, 3), dtype=np.uint8) * 128
    # Standard packed RGB as float32 (PCL/RViz-friendly): point_step = 16
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

        self.declare_parameter('depth_topic', '/camera/aligned_depth_to_color/image_raw')
        self.declare_parameter('camera_info_topic', '/camera/aligned_depth_to_color/camera_info')
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('detections_topic', '/realsense_yolo/detections')
        self.declare_parameter('depth_pointcloud_topic', '/realsense_yolo/depth_pointcloud')
        self.declare_parameter('lidar_pointcloud_topic', '/realsense_yolo/lidar_pointcloud')
        self.declare_parameter('markers_3d_topic', '/realsense_yolo/detection_boxes_3d')
        self.declare_parameter('camera_frame', 'camera_depth_optical_frame')
        self.declare_parameter('laser_frame', 'laser')
        self.declare_parameter('depth_scale', 0.001)  # depth in mm -> m
        self.declare_parameter('depth_max_m', 10.0)
        self.declare_parameter('depth_step', 4)  # subsample depth image
        self.declare_parameter('lidar_max_range', 10.0)
        self.declare_parameter('depth_image_plane_topic', '/realsense_yolo/depth_image_plane')
        self.declare_parameter('depth_plane_step', 10)  # subsample for depth-map surface
        self.declare_parameter('report_period_s', 1.0)

        self.camera_frame = self.get_parameter('camera_frame').get_parameter_value().string_value
        self.laser_frame = self.get_parameter('laser_frame').get_parameter_value().string_value
        self.depth_scale = self.get_parameter('depth_scale').get_parameter_value().double_value
        self.depth_max_m = self.get_parameter('depth_max_m').get_parameter_value().double_value
        self.depth_step = int(self.get_parameter('depth_step').get_parameter_value().integer_value)
        self.lidar_max_range = self.get_parameter('lidar_max_range').get_parameter_value().double_value
        self.depth_plane_step = int(self.get_parameter('depth_plane_step').get_parameter_value().integer_value)
        self.report_period_s = self.get_parameter('report_period_s').get_parameter_value().double_value
        self._last_tf_warn_sec = -1

        self.bridge = CvBridge()
        self._depth = None
        self._depth_header = None
        self._camera_info = None
        self._scan = None
        self._detections = None
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        self.create_subscription(Image, self.get_parameter('depth_topic').get_parameter_value().string_value, self._depth_cb, 10)
        self.create_subscription(CameraInfo, self.get_parameter('camera_info_topic').get_parameter_value().string_value, self._camera_info_cb, 10)
        self.create_subscription(LaserScan, self.get_parameter('scan_topic').get_parameter_value().string_value, self._scan_cb, 10)
        self.create_subscription(Float32MultiArray, self.get_parameter('detections_topic').get_parameter_value().string_value, self._det_cb, 10)

        self._pub_depth_pc = self.create_publisher(PointCloud2, self.get_parameter('depth_pointcloud_topic').get_parameter_value().string_value, 10)
        self._pub_lidar_pc = self.create_publisher(PointCloud2, self.get_parameter('lidar_pointcloud_topic').get_parameter_value().string_value, 10)
        self._pub_markers = self.create_publisher(MarkerArray, self.get_parameter('markers_3d_topic').get_parameter_value().string_value, 10)
        self._pub_depth_plane = self.create_publisher(
            PointCloud2,
            self.get_parameter('depth_image_plane_topic').get_parameter_value().string_value,
            10,
        )

        self._timer = self.create_timer(0.12, self._publish_3d)  # ~8 Hz for RViz stability
        self._report_timer = self.create_timer(max(0.2, self.report_period_s), self._report_objects)

        self.get_logger().info('Fusion3D: depth + LiDAR -> PointCloud2, detections -> 3D MarkerArray')

    def _depth_cb(self, msg):
        try:
            self._depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            self._depth_header = msg.header
        except Exception as e:
            self.get_logger().error(f'Depth cb: {e}')

    def _camera_info_cb(self, msg):
        self._camera_info = msg

    def _scan_cb(self, msg):
        self._scan = msg

    def _det_cb(self, msg):
        self._detections = msg

    @staticmethod
    def _class_name(cls_id):
        i = int(cls_id)
        if 0 <= i < len(COCO_NAMES):
            return COCO_NAMES[i]
        return f'class_{i}'

    @staticmethod
    def _class_color_rgb(cls_id):
        hue = (int(cls_id) * 37) % 180
        hsv = np.uint8([[[hue, 220, 255]]])
        bgr = cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)[0, 0]
        return np.array([int(bgr[2]), int(bgr[1]), int(bgr[0])], dtype=np.uint8)  # RGB

    def _lookup_transform(self, target_frame, source_frame, timeout_s=0.1):
        try:
            return self._tf_buffer.lookup_transform(
                target_frame, source_frame, rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=timeout_s)
            )
        except TransformException:
            return None

    @staticmethod
    def _quat_to_rot(qx, qy, qz, qw):
        return np.array([
            [1 - 2*qy*qy - 2*qz*qz, 2*qx*qy - 2*qw*qz, 2*qx*qz + 2*qw*qy],
            [2*qx*qy + 2*qw*qz, 1 - 2*qx*qx - 2*qz*qz, 2*qy*qz - 2*qw*qx],
            [2*qx*qz - 2*qw*qy, 2*qy*qz + 2*qw*qx, 1 - 2*qx*qx - 2*qy*qy]
        ], dtype=np.float64)

    @staticmethod
    def _angle_mask(angles_deg, right_deg, left_deg):
        # Normal case: right <= left (e.g., -12 .. +8)
        if right_deg <= left_deg:
            return (angles_deg >= right_deg) & (angles_deg <= left_deg)
        # Wrap-around case (rare): interval crosses -180/180
        return (angles_deg >= right_deg) | (angles_deg <= left_deg)

    def _report_objects(self):
        if self._detections is None or self._camera_info is None or self._depth is None:
            return

        t_lc = self._lookup_transform(self.laser_frame, self.camera_frame, timeout_s=0.05)
        if t_lc is None:
            return
        q = t_lc.transform.rotation
        R_lc = self._quat_to_rot(q.x, q.y, q.z, q.w)
        tvec_lc = np.array([
            t_lc.transform.translation.x,
            t_lc.transform.translation.y,
            t_lc.transform.translation.z
        ], dtype=np.float64)

        depth_raw = np.asarray(self._depth, dtype=np.float32)
        if depth_raw.size and np.nanmax(depth_raw) > 100:
            depth_m = depth_raw * self.depth_scale
        else:
            depth_m = depth_raw
        h, w = depth_m.shape[:2]

        K = self._camera_info.k
        fx, fy = float(K[0]), float(K[4])
        cx, cy = float(K[2]), float(K[5])

        scan_angles_deg = None
        scan_ranges = None
        if self._scan is not None and len(self._scan.ranges) > 0:
            n = len(self._scan.ranges)
            scan_angles_deg = np.degrees(
                np.arange(n, dtype=np.float64) * self._scan.angle_increment + self._scan.angle_min
            )
            scan_ranges = np.array(self._scan.ranges, dtype=np.float32)

        data = np.array(self._detections.data, dtype=np.float32)
        if data.size < 7:
            return
        dets = data.reshape(-1, 7)
        reports = []

        for row in dets:
            x1, y1, x2, y2, det_depth_m, cls_id, conf = row
            x1i = int(np.clip(np.floor(x1), 0, w - 1))
            x2i = int(np.clip(np.ceil(x2), 0, w - 1))
            y1i = int(np.clip(np.floor(y1), 0, h - 1))
            y2i = int(np.clip(np.ceil(y2), 0, h - 1))
            if x2i <= x1i or y2i <= y1i:
                continue

            # Sample depth points inside bbox to estimate object span/range.
            sx = max(2, (x2i - x1i) // 24)
            sy = max(2, (y2i - y1i) // 24)
            uu, vv = np.meshgrid(
                np.arange(x1i, x2i + 1, sx, dtype=np.float32),
                np.arange(y1i, y2i + 1, sy, dtype=np.float32)
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
            pts_cam = np.stack([x_cam, y_cam, z], axis=0)  # 3xN
            pts_laser = (R_lc @ pts_cam).T + tvec_lc  # Nx3

            # LiDAR convention requested: left positive, right negative.
            ang_deg = np.degrees(np.arctan2(pts_laser[:, 1], pts_laser[:, 0]))
            left_deg = float(np.max(ang_deg))
            right_deg = float(np.min(ang_deg))
            depth_nearest = float(np.min(np.linalg.norm(pts_laser[:, :2], axis=1)))

            lidar_nearest = None
            if scan_angles_deg is not None and scan_ranges is not None:
                m = self._angle_mask(scan_angles_deg, right_deg, left_deg)
                r = scan_ranges[m]
                r_valid = r[np.isfinite(r) & (r > 0.02) & (r < self.lidar_max_range)]
                if r_valid.size > 0:
                    lidar_nearest = float(np.min(r_valid))

            if lidar_nearest is not None:
                nearest = lidar_nearest
                source = 'lidar'
            else:
                nearest = depth_nearest if depth_nearest > 0 else float(det_depth_m)
                source = 'depth'

            reports.append({
                'name': self._class_name(int(cls_id)),
                'conf': float(conf),
                'nearest': float(nearest),
                'source': source,
                'left_deg': left_deg,
                'right_deg': right_deg,
                'span_deg': float(left_deg - right_deg),
            })

        if reports:
            self.get_logger().info('--- Fusion object report (1s) ---')
            for r in reports:
                self.get_logger().info(
                    f"{r['name']} conf={r['conf']:.2f} "
                    f"nearest={r['nearest']:.2f}m source={r['source']} "
                    f"angles=[{r['right_deg']:.1f}°, {r['left_deg']:.1f}°] span={r['span_deg']:.1f}°"
                )

    def _publish_3d(self):
        stamp = self.get_clock().now().to_msg()
        header = Header()
        header.stamp = stamp
        header.frame_id = self.camera_frame

        # 1) Depth -> PointCloud2 (độ sâu thật trong 3D, màu đỏ=gần xanh=xa)
        if self._depth is not None and self._camera_info is not None:
            K = self._camera_info.k
            fx, fy = float(K[0]), float(K[4])
            cx, cy = float(K[2]), float(K[5])
            h, w = self._depth.shape[:2]
            depth = np.asarray(self._depth, dtype=np.float32)
            if depth.size and np.nanmax(depth) > 100:
                depth = depth * self.depth_scale  # mm -> m
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
                rgb[:, 0] = np.clip(255 * (1 - z / self.depth_max_m), 0, 255).astype(np.uint8)  # red near
                rgb[:, 2] = np.clip(255 * z / self.depth_max_m, 0, 255).astype(np.uint8)  # blue far
                pc = make_point_cloud2(header, self.camera_frame, xyz, rgb)
                if pc:
                    self._pub_depth_pc.publish(pc)

        # 1b) Depth map trong 3D: bề mặt độ sâu thật (x,y,z từ depth), màu JET như ảnh depth
        if self._depth is not None and self._camera_info is not None:
            K = self._camera_info.k
            fx, fy = float(K[0]), float(K[4])
            cx, cy = float(K[2]), float(K[5])
            h, w = self._depth.shape[:2]
            depth_raw = np.asarray(self._depth, dtype=np.float32)
            if depth_raw.size and np.nanmax(depth_raw) > 100:
                depth_m = depth_raw * self.depth_scale  # mm -> m
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
            # Màu JET theo độ sâu (đỏ gần, xanh xa) giống ảnh depth
            depth_8 = np.clip(255.0 * z / self.depth_max_m, 0, 255).astype(np.uint8)
            colormap_bgr = cv2.applyColorMap(depth_8.reshape(-1, 1), cv2.COLORMAP_JET)
            rgb_surf = colormap_bgr.reshape(-1, 3)[:, ::-1]  # BGR -> RGB
            # Overlay class color onto depth-map points inside YOLO bboxes
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

        # 2) LiDAR -> PointCloud2 (transform to camera frame)
        if self._scan is not None:
            n = len(self._scan.ranges)
            angles = np.arange(n, dtype=np.float64) * self._scan.angle_increment + self._scan.angle_min
            ranges = np.array(self._scan.ranges, dtype=np.float32)
            valid = np.isfinite(ranges) & (ranges > 0) & (ranges < self.lidar_max_range)
            angles = angles[valid]
            ranges = ranges[valid]
            x_l = ranges * np.cos(angles)
            y_l = ranges * np.sin(angles)
            z_l = np.zeros_like(ranges)
            p_l = np.stack([x_l, y_l, z_l], axis=0)
            source_frames = []
            if self._scan.header.frame_id:
                source_frames.append(self._scan.header.frame_id)
            if self.laser_frame and self.laser_frame not in source_frames:
                source_frames.append(self.laser_frame)
            t = None
            for src in source_frames:
                try:
                    t = self._tf_buffer.lookup_transform(
                        self.camera_frame, src,
                        rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=0.1)
                    )
                    break
                except TransformException:
                    t = None
            if t is None:
                now_sec = int(self.get_clock().now().nanoseconds / 1e9)
                if now_sec != self._last_tf_warn_sec and now_sec % 5 == 0:
                    self._last_tf_warn_sec = now_sec
                    self.get_logger().warn(
                        f'Cannot transform LiDAR to {self.camera_frame}. '
                        f'scan frame={self._scan.header.frame_id}, configured laser_frame={self.laser_frame}'
                    )
            else:
                qx, qy, qz, qw = t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w
                R = self._quat_to_rot(qx, qy, qz, qw)
                p_c = (R @ p_l).T + np.array([t.transform.translation.x, t.transform.translation.y, t.transform.translation.z])
                xyz = p_c
                if len(xyz) > 0:
                    rgb = np.full((len(xyz), 3), 0, dtype=np.uint8)
                    rgb[:, 1] = 255  # green for LiDAR
                    pc = make_point_cloud2(header, self.camera_frame, xyz, rgb)
                    if pc:
                        self._pub_lidar_pc.publish(pc)

        # 3) Detections -> 3D bounding box markers
        if self._detections is not None and self._camera_info is not None and len(self._detections.data) >= 7:
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
                center_z = depth_m + d_m / 2.0  # depth_m is near/front face
                half = np.array([w_m/2, h_m/2, d_m/2])
                c = np.array([center_x, center_y, center_z])
                # 12 edges of box: (p1, p2) pairs
                edges = [
                    ([-1,-1,-1], [1,-1,-1]), ([-1,-1,-1], [-1,1,-1]), ([-1,-1,-1], [-1,-1,1]),
                    ([1,-1,-1], [1,1,-1]), ([1,-1,-1], [1,-1,1]),
                    ([-1,1,-1], [1,1,-1]), ([-1,1,-1], [-1,1,1]),
                    ([1,1,-1], [1,1,1]), ([1,-1,1], [1,1,1]), ([1,-1,1], [-1,-1,1]),
                    ([-1,1,1], [1,1,1]), ([-1,1,1], [-1,-1,1]),
                ]
                from geometry_msgs.msg import Point
                m = Marker()
                m.header = header
                m.ns = 'bbox3d'
                m.id = i
                m.type = Marker.LINE_LIST
                m.action = Marker.ADD
                m.scale.x = 0.02
                cls_rgb = self._class_color_rgb(int(cls_id))
                m.color.r = float(cls_rgb[0]) / 255.0
                m.color.g = float(cls_rgb[1]) / 255.0
                m.color.b = float(cls_rgb[2]) / 255.0
                m.color.a = 1.0
                for e1, e2 in edges:
                    p1 = c + np.array(e1) * half
                    p2 = c + np.array(e2) * half
                    m.points.append(Point(x=float(p1[0]), y=float(p1[1]), z=float(p1[2])))
                    m.points.append(Point(x=float(p2[0]), y=float(p2[1]), z=float(p2[2])))
                markers.markers.append(m)
            if markers.markers:
                self._pub_markers.publish(markers)


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
