#!/usr/bin/env python3
"""
RealSense D435i + YOLO Object Detection Node (Optimized for high FPS)

- GPU acceleration, FP16, multi-threaded pipeline
- Runs YOLO on RGB in dedicated thread (non-blocking)
- Keeps resolution for wide viewing (bao quat)
"""

import rclpy
from rclpy.node import Node
import time
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray, MultiArrayDimension
from cv_bridge import CvBridge
import cv2
import numpy as np
import threading

# YOLO - use ultralytics
try:
    from ultralytics import YOLO
    import torch
    YOLO_AVAILABLE = True
    USE_GPU = torch.cuda.is_available()
except ImportError:
    YOLO_AVAILABLE = False
    USE_GPU = False


class RealsenseYoloNode(Node):
    def __init__(self):
        super().__init__('realsense_yolo_node')

        self.declare_parameter('color_topic', '/camera/color/image_raw')
        self.declare_parameter('depth_topic', '/camera/aligned_depth_to_color/image_raw')
        self.declare_parameter('output_topic', '/realsense_yolo/depth_with_bboxes')
        self.declare_parameter('detections_topic', '/realsense_yolo/detections')
        self.declare_parameter('yolo_model', 'yolov8n.pt')
        self.declare_parameter('confidence_threshold', 0.5)
        self.declare_parameter('iou_threshold', 0.45)
        self.declare_parameter('duplicate_iou_threshold', 0.6)
        self.declare_parameter('imgsz', 480)  # YOLO input size: 416 faster, 640 more accurate
        self.declare_parameter('use_half', True)  # FP16 on GPU

        self.color_topic = self.get_parameter('color_topic').get_parameter_value().string_value
        self.depth_topic = self.get_parameter('depth_topic').get_parameter_value().string_value
        self.output_topic = self.get_parameter('output_topic').get_parameter_value().string_value
        self.detections_topic = self.get_parameter('detections_topic').get_parameter_value().string_value
        self.conf_thresh = self.get_parameter('confidence_threshold').get_parameter_value().double_value
        self.iou_thresh = self.get_parameter('iou_threshold').get_parameter_value().double_value
        self.dup_iou_thresh = self.get_parameter('duplicate_iou_threshold').get_parameter_value().double_value
        try:
            self.imgsz = int(self.get_parameter('imgsz').value)
        except (ValueError, TypeError):
            self.imgsz = 480
        try:
            self.use_half = bool(self.get_parameter('use_half').value)
        except (ValueError, TypeError):
            self.use_half = True

        self.bridge = CvBridge()
        self._lock = threading.Lock()
        self._color_image = None
        self._depth_image = None
        self._color_header = None
        self._depth_header = None
        self._logged_color = False
        self._logged_depth = False
        self._logged_size_mismatch = False
        self._running = True

        if YOLO_AVAILABLE:
            model_name = self.get_parameter('yolo_model').get_parameter_value().string_value
            device = 'cuda' if USE_GPU else 'cpu'
            try:
                self.yolo = YOLO(model_name)
                self.get_logger().info(
                    f'YOLO loaded: {model_name} | device={device} | FP16={self.use_half and USE_GPU}'
                )
            except Exception as e:
                self.get_logger().error(f'Failed to load YOLO: {e}')
                self.yolo = None
        else:
            self.get_logger().warn('ultralytics not installed')
            self.yolo = None

        self.color_sub = self.create_subscription(Image, self.color_topic, self.color_callback, 10)
        self.depth_sub = self.create_subscription(Image, self.depth_topic, self.depth_callback, 10)
        self.pub = self.create_publisher(Image, self.output_topic, 10)
        self.det_pub = self.create_publisher(Float32MultiArray, self.detections_topic, 10)

        if self.yolo is not None:
            self._inference_thread = threading.Thread(target=self._inference_loop, daemon=True)
            self._inference_thread.start()

        self.get_logger().info(
            f'Subscribing to {self.color_topic}, {self.depth_topic} | imgsz={self.imgsz}'
        )

    def color_callback(self, msg):
        if not self._logged_color:
            self.get_logger().info('Received first color frame')
            self._logged_color = True
        with self._lock:
            self._color_header = msg.header
            self._color_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

    def depth_callback(self, msg):
        if not self._logged_depth:
            self.get_logger().info('Received first depth frame')
            self._logged_depth = True
        with self._lock:
            self._depth_header = msg.header
            self._depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

    def _get_frame_pair(self):
        """Returns (color, depth, header) only when BOTH are available and sizes match.
        Returns None otherwise - output must always be depth map + bboxes, never color."""
        with self._lock:
            if self._color_image is None or self._depth_image is None:
                return None
            if self._color_image.shape[:2] != self._depth_image.shape[:2]:
                if not self._logged_size_mismatch:
                    self._logged_size_mismatch = True
                    self.get_logger().warn(
                        f'Color/depth size mismatch: color {self._color_image.shape[:2]} '
                        f'vs depth {self._depth_image.shape[:2]}. Enable align_depth and match resolution.'
                    )
                return None
            return (
                self._color_image.copy(),
                self._depth_image.copy(),
                self._depth_header,
            )

    def _get_depth_at_point(self, depth_image, x, y):
        if depth_image is None:
            return None
        h, w = depth_image.shape[:2]
        xi, yi = int(np.clip(x, 0, w - 1)), int(np.clip(y, 0, h - 1))
        d = float(depth_image[yi, xi])
        if d <= 0 or np.isnan(d):
            return None
        if depth_image.dtype == np.uint16:
            return d / 1000.0
        return d

    @staticmethod
    def _iou_xyxy(a, b):
        ax1, ay1, ax2, ay2 = a
        bx1, by1, bx2, by2 = b
        inter_x1 = max(ax1, bx1)
        inter_y1 = max(ay1, by1)
        inter_x2 = min(ax2, bx2)
        inter_y2 = min(ay2, by2)
        iw = max(0.0, inter_x2 - inter_x1)
        ih = max(0.0, inter_y2 - inter_y1)
        inter = iw * ih
        if inter <= 0.0:
            return 0.0
        area_a = max(0.0, ax2 - ax1) * max(0.0, ay2 - ay1)
        area_b = max(0.0, bx2 - bx1) * max(0.0, by2 - by1)
        union = area_a + area_b - inter
        return float(inter / union) if union > 0 else 0.0

    def _suppress_duplicate_boxes(self, dets):
        """Extra duplicate suppression by class + IoU after YOLO NMS."""
        if not dets:
            return []
        dets_sorted = sorted(dets, key=lambda d: d['conf'], reverse=True)
        kept = []
        for d in dets_sorted:
            duplicated = False
            for k in kept:
                if d['cls_id'] != k['cls_id']:
                    continue
                if self._iou_xyxy(d['xyxy'], k['xyxy']) >= self.dup_iou_thresh:
                    duplicated = True
                    break
            if not duplicated:
                kept.append(d)
        return kept

    @staticmethod
    def _class_color_bgr(cls_id):
        # Stable vivid color from class id (OpenCV uses BGR)
        hue = (int(cls_id) * 37) % 180
        hsv = np.uint8([[[hue, 220, 255]]])
        bgr = cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)[0, 0]
        return int(bgr[0]), int(bgr[1]), int(bgr[2])

    def _inference_loop(self):
        from std_msgs.msg import Header

        frame_count = 0
        t0 = time.perf_counter()
        logged_first = False

        while self._running and self.yolo is not None:
            try:
                pair = self._get_frame_pair()
                if pair is None:
                    time.sleep(0.01)
                    continue

                color, depth, header = pair

                # YOLO inference
                half = self.use_half and USE_GPU
                results = self.yolo(
                    color,
                    imgsz=self.imgsz,
                    conf=self.conf_thresh,
                    iou=self.iou_thresh,
                    verbose=False,
                    half=half,
                    augment=False,
                    max_det=25,
                )[0]

                # Always use depth map for visualization (never color)
                depth_np = np.asarray(depth, dtype=np.float32)
                depth_8 = np.clip(depth_np / 10.0, 0, 255).astype(np.uint8)
                vis = cv2.applyColorMap(depth_8, cv2.COLORMAP_JET)

                # Draw bboxes (scale thickness/font by resolution for visibility)
                h, w = vis.shape[:2]
                thickness = max(2, int(2 * w / 640))
                font_scale = max(0.5, 0.5 * w / 640)

                raw_dets = []
                for box in results.boxes:
                    xyxy = box.xyxy[0].cpu().numpy()
                    x1, y1, x2, y2 = map(float, xyxy)
                    cls_id = int(box.cls[0])
                    conf = float(box.conf[0])
                    label = results.names[cls_id]

                    cx = (x1 + x2) / 2
                    cy = (y1 + y2) / 2
                    depth_m = self._get_depth_at_point(depth, int(cx), int(cy))
                    raw_dets.append({
                        'xyxy': (x1, y1, x2, y2),
                        'cls_id': cls_id,
                        'conf': conf,
                        'label': label,
                        'depth_m': depth_m if depth_m is not None else 0.0,
                    })

                filtered_dets = self._suppress_duplicate_boxes(raw_dets)
                det_data = []
                for d in filtered_dets:
                    x1, y1, x2, y2 = d['xyxy']
                    cls_id = d['cls_id']
                    conf = d['conf']
                    label = d['label']
                    depth_val = d['depth_m']
                    depth_str = f'{depth_val:.2f}m' if depth_val > 0 else 'N/A'

                    det_data.extend([x1, y1, x2, y2, depth_val, float(cls_id), conf])
                    color_bgr = self._class_color_bgr(cls_id)
                    cv2.rectangle(vis, (int(x1), int(y1)), (int(x2), int(y2)), color_bgr, thickness)
                    cv2.putText(
                        vis, f'{label} {conf:.2f} {depth_str}',
                        (int(x1), max(0, int(y1) - 5)), cv2.FONT_HERSHEY_SIMPLEX,
                        font_scale, color_bgr, max(1, thickness - 1)
                    )

                if det_data:
                    det_msg = Float32MultiArray()
                    n_det = len(det_data) // 7
                    det_msg.layout.dim.append(MultiArrayDimension(label='detections', size=n_det, stride=7))
                    det_msg.layout.dim.append(MultiArrayDimension(label='vals', size=7, stride=1))
                    det_msg.data = det_data
                    self.det_pub.publish(det_msg)

                out_msg = self.bridge.cv2_to_imgmsg(vis, encoding='bgr8')
                if header is not None:
                    out_msg.header = header
                    if out_msg.header.stamp.sec == 0 and out_msg.header.stamp.nanosec == 0:
                        out_msg.header.stamp = self.get_clock().now().to_msg()
                else:
                    out_msg.header = Header()
                    out_msg.header.stamp = self.get_clock().now().to_msg()
                    out_msg.header.frame_id = 'camera_depth_optical_frame'

                self.pub.publish(out_msg)

                if not logged_first:
                    self.get_logger().info('Published first YOLO frame')
                    logged_first = True

                frame_count += 1
                if frame_count % 100 == 0:
                    elapsed = time.perf_counter() - t0
                    fps = 100.0 / elapsed if elapsed > 0 else 0
                    self.get_logger().info(f'FPS: {fps:.1f}')
                    t0 = time.perf_counter()

            except Exception as e:
                self.get_logger().error(f'Inference error: {e}')
                time.sleep(0.05)

    def destroy_node(self, *args, **kwargs):
        self._running = False
        if hasattr(self, '_inference_thread') and self._inference_thread.is_alive():
            self._inference_thread.join(timeout=1.0)
        super().destroy_node(*args, **kwargs)


def main(args=None):
    rclpy.init(args=args)
    node = RealsenseYoloNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
