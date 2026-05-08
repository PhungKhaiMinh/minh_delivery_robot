#!/usr/bin/env python3
"""
RealSense D435i + YOLO11n object detection & multi-object tracking (BoT-SORT)
optimized cho Jetson AGX Xavier.

Highlights
----------
* YOLO11n (≥39.5 mAP, 6.5 GFLOPs)  — kế thừa Ultralytics, ổn định và nhanh hơn v8n
* Auto-export weights → TensorRT engine (.engine) FP16 trên GPU; tuỳ chọn DLA fallback
* BoT-SORT tracker (Kalman + GMC + Re-ID) — track_id nhất quán qua các frame
* Detection batch trong 1 lần inference; tracker xử lý tất cả vật cùng lúc (Hungarian)
* Output topic /realsense_yolo/detections nay là 8 floats / vật:
        [x1, y1, x2, y2, depth_m, cls_id, conf, track_id]
"""

from __future__ import annotations

import os
import time
import threading
from pathlib import Path

# ── Compatibility shims (PHẢI đặt trước khi import tensorrt / ultralytics) ──
# 1) NumPy ≥1.20 đã xoá `np.bool`, nhưng TensorRT 8.5.x trên JetPack 5 vẫn dùng nó.
#    Thiếu shim → `AttributeError: module 'numpy' has no attribute 'bool'`.
# 2) Tắt Ultralytics auto-install: trong runtime đừng để nó tự `pip install` (đụng pip
#    sẽ gây dependency conflict + spike RAM khi đang chạy hệ thống).
import warnings
import numpy as np
# NumPy 1.20+ drop `np.bool/float/int` nhưng TensorRT bindings cũ vẫn gọi → shim lại.
# `hasattr(np, 'bool')` ở NumPy 1.24+ phát FutureWarning → dùng try/except an toàn hơn.
with warnings.catch_warnings():
    warnings.simplefilter("ignore", FutureWarning)
    try:
        _ = np.bool         # type: ignore[attr-defined]
    except AttributeError:
        np.bool = bool      # type: ignore[attr-defined]
    try:
        _ = np.float        # type: ignore[attr-defined]
    except AttributeError:
        np.float = float    # type: ignore[attr-defined]
    try:
        _ = np.int          # type: ignore[attr-defined]
    except AttributeError:
        np.int = int        # type: ignore[attr-defined]
os.environ.setdefault('YOLO_AUTOINSTALL', 'False')
os.environ.setdefault('YOLO_OFFLINE', 'True')
# Hạn chế số luồng OMP/MKL: Jetson AGX Xavier có 8 core nhưng nếu PyTorch/
# NumPy spawn >4 OMP thread thì chúng cạnh tranh cache + làm chậm
# post-processing (NMS, tensor ops trên CPU). 4 thread là sweet spot.
os.environ.setdefault('OMP_NUM_THREADS', '4')
os.environ.setdefault('MKL_NUM_THREADS', '4')

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray, MultiArrayDimension, Header
from cv_bridge import CvBridge
import cv2


# QoS cho camera topic: BestEffort + KeepLast(1) — khớp publisher RealSense (SensorDataQoS).
# Với depth=1, ROS2 KHÔNG giữ queue frame cũ → luôn lấy frame mới nhất, không back-pressure.
SENSOR_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
    durability=DurabilityPolicy.VOLATILE,
)

DET_FLOATS_PER_BOX = 8  # [x1, y1, x2, y2, depth, cls_id, conf, track_id]

try:
    from ultralytics import YOLO
    import torch
    # Thêm: set_num_threads sau khi torch import — giới hạn intra-op thread.
    # cv2 cũng vậy — OpenCV post-processing dùng <=4 thread là đủ.
    try:
        torch.set_num_threads(4)
    except Exception:
        pass
    try:
        cv2.setNumThreads(4)
    except Exception:
        pass
    YOLO_AVAILABLE = True
    USE_GPU = torch.cuda.is_available()
except Exception as e:
    print(f'[realsense_yolo_node] import error: {e}')
    YOLO_AVAILABLE = False
    USE_GPU = False


class RealsenseYoloNode(Node):
    def __init__(self):
        super().__init__('realsense_yolo_node')

        # ── ROS topics ────────────────────────────────────────────────
        self.declare_parameter('color_topic', '/camera/color/image_raw')
        self.declare_parameter('depth_topic', '/camera/aligned_depth_to_color/image_raw')
        self.declare_parameter('output_topic', '/realsense_yolo/depth_with_bboxes')
        self.declare_parameter('detections_topic', '/realsense_yolo/detections')

        # ── Model / weights ───────────────────────────────────────────
        # Mặc định YOLO11n: tốt hơn v8n cả mAP lẫn FLOPs. Nếu file không có sẵn,
        # Ultralytics sẽ tự download lần đầu. Có thể override qua launch arg.
        self.declare_parameter('yolo_model', 'yolo11n.pt')
        self.declare_parameter('model_path', '')

        # ── TensorRT engine ───────────────────────────────────────────
        # use_tensorrt: nếu engine .engine sẵn có → load. KHÔNG tự export trong runtime
        #   (export ngốn 4-6 GB RAM + 100 % GPU vài phút → AGX Xavier có thể sập nguồn khi
        #   đang chạy cùng RealSense + LiDAR). Hãy export OFFLINE bằng:
        #     ros2 run realsense_yolo export_engine --imgsz 640 --half
        # auto_export_engine=True chỉ dùng khi bạn chắc PSU + nguồn đủ tải.
        self.declare_parameter('use_tensorrt', True)
        self.declare_parameter('auto_export_engine', False)
        self.declare_parameter('tensorrt_engine_path', '')   # auto-derive nếu rỗng
        self.declare_parameter('tensorrt_dla', -1)           # -1 = không DLA, 0/1 = core
        self.declare_parameter('tensorrt_int8', False)       # cần calibration set, default off
        self.declare_parameter('tensorrt_workspace_gb', 2)

        # ── Inference ─────────────────────────────────────────────────
        self.declare_parameter('confidence_threshold', 0.5)
        self.declare_parameter('iou_threshold', 0.45)
        self.declare_parameter('duplicate_iou_threshold', 0.6)
        self.declare_parameter('imgsz', 480)
        self.declare_parameter('use_half', True)
        self.declare_parameter('max_det', 25)

        # ── Tracking ──────────────────────────────────────────────────
        self.declare_parameter('enable_tracking', True)
        self.declare_parameter('tracker_config', '')         # '' → dùng botsort_deli.yaml trong package

        # ── Class mapping ─────────────────────────────────────────────
        self.declare_parameter('primary_class_ids', [0, 1, 2, 3])
        self.declare_parameter('others_class_id', 80)
        self.declare_parameter('others_label', 'others')

        # ── Output throttling (giảm tải CPU / DDS bandwidth) ─────────
        # publish_vis_image=False → bỏ cv_bridge encode + publish
        #   → save ~30% CPU + ~55 MB/s DDS bandwidth khi không có ai xem trực tiếp.
        # vis_image_every_n=N    → publish 1 frame mỗi N frame (3 = 20 FPS từ 60 FPS).
        self.declare_parameter('publish_vis_image', True)
        self.declare_parameter('vis_image_every_n', 1)

        # ── Resolve params ────────────────────────────────────────────
        self.color_topic = self._sp('color_topic')
        self.depth_topic = self._sp('depth_topic')
        self.output_topic = self._sp('output_topic')
        self.detections_topic = self._sp('detections_topic')
        self.conf_thresh = self._dp('confidence_threshold')
        self.iou_thresh = self._dp('iou_threshold')
        self.dup_iou_thresh = self._dp('duplicate_iou_threshold')
        self.imgsz = int(self.get_parameter('imgsz').value or 480)
        self.use_half = bool(self.get_parameter('use_half').value)
        self.max_det = int(self.get_parameter('max_det').value or 25)
        self.use_trt = bool(self.get_parameter('use_tensorrt').value)
        self.auto_export = bool(self.get_parameter('auto_export_engine').value)
        self.trt_dla = int(self.get_parameter('tensorrt_dla').value)
        self.trt_int8 = bool(self.get_parameter('tensorrt_int8').value)
        self.trt_workspace_gb = int(self.get_parameter('tensorrt_workspace_gb').value or 2)
        self.enable_tracking = bool(self.get_parameter('enable_tracking').value)

        ia = self.get_parameter('primary_class_ids').get_parameter_value().integer_array_value
        self._primary_class_ids = set(int(x) for x in ia) if ia else {0, 1, 2, 3}
        self._others_class_id = int(self._ip('others_class_id'))
        self._others_label = self._sp('others_label')
        self._publish_vis = bool(self.get_parameter('publish_vis_image').value)
        self._vis_every_n = max(1, int(self.get_parameter('vis_image_every_n').value or 1))

        # ── State ─────────────────────────────────────────────────────
        self.bridge = CvBridge()
        self._lock = threading.Lock()
        self._color_image = None
        self._depth_image = None
        self._color_header = None
        self._depth_header = None
        self._color_stamp_ns = 0            # nanosec timestamp của color frame mới nhất
        self._color_frames_received = 0     # đếm frame color thô từ camera (để tính camera FPS)
        self._logged_color = False
        self._logged_depth = False
        self._logged_size_mismatch = False
        self._running = True
        self._frame_event = threading.Event()

        # ── Model: load trong inference thread để node ROS khởi tạo ngay,
        # RealSense stream và TensorRT load song song → giảm thời gian tới frame đầu.
        self.yolo = None
        self._using_engine = False

        # ── Subscribers / publishers ──────────────────────────────────
        # SENSOR_QOS (BestEffort, KeepLast 1) khớp RealSense publisher → không drop do
        # QoS mismatch + luôn nhận frame mới nhất (không queue back-pressure).
        self.color_sub = self.create_subscription(
            Image, self.color_topic, self.color_callback, SENSOR_QOS)
        self.depth_sub = self.create_subscription(
            Image, self.depth_topic, self.depth_callback, SENSOR_QOS)
        self.pub = self.create_publisher(Image, self.output_topic, 10) if self._publish_vis else None
        self.det_pub = self.create_publisher(Float32MultiArray, self.detections_topic, 10)

        self._inference_thread = None
        if YOLO_AVAILABLE:
            self._inference_thread = threading.Thread(target=self._inference_loop, daemon=True)
            self._inference_thread.start()

        self.get_logger().info(
            f'sub: {self.color_topic} + {self.depth_topic} | imgsz={self.imgsz} | '
            f'tracking={"ON" if self.enable_tracking else "off"} | '
            f'model={"loading in worker thread" if YOLO_AVAILABLE else "DISABLED"} | '
            f'vis_image={"every " + str(self._vis_every_n) if self._publish_vis else "OFF"} | '
            f'primary={sorted(self._primary_class_ids)} else→"{self._others_label}"({self._others_class_id})'
        )

    # ── Param helpers ────────────────────────────────────────────────
    def _sp(self, name): return self.get_parameter(name).get_parameter_value().string_value
    def _dp(self, name): return self.get_parameter(name).get_parameter_value().double_value
    def _ip(self, name): return self.get_parameter(name).get_parameter_value().integer_value

    # ── Model loading + TensorRT auto-export ─────────────────────────
    def _resolve_weights_path(self) -> str:
        """Trả về đường dẫn .pt khả dụng; rơi về tên ngắn để Ultralytics auto-download."""
        explicit = (self._sp('model_path') or '').strip()
        if explicit and os.path.isfile(explicit):
            return explicit
        name = (self._sp('yolo_model') or 'yolo11n.pt').strip()
        # Tìm trong các vị trí thường dùng
        here = os.path.dirname(os.path.abspath(__file__))
        candidates = [
            explicit,
            os.path.normpath(os.path.join(here, '..', '..', name)),     # repo root
            os.path.normpath(os.path.join(here, '..', name)),           # package root
            os.path.join('/mnt/jetson_data/deli_ws', name),
            os.path.join('/mnt/jetson_data/deli_ws/src/realsense_yolo', name),
        ]
        for c in candidates:
            if c and os.path.isfile(c):
                return c
        # Fallback: trả về tên file để Ultralytics tự download
        return name

    def _engine_path_for(self, pt_path: str) -> str:
        explicit = (self._sp('tensorrt_engine_path') or '').strip()
        if explicit:
            return explicit
        suffix = ''
        if self.trt_dla in (0, 1):
            suffix += f'_dla{self.trt_dla}'
        if self.trt_int8:
            suffix += '_int8'
        elif self.use_half:
            suffix += '_fp16'
        return str(Path(pt_path).with_suffix('')) + f'_imgsz{self.imgsz}{suffix}.engine'

    def _export_engine(self, pt_path: str, engine_path: str) -> bool:
        """Export .pt → TensorRT .engine (cache lại). Trả về True nếu OK."""
        try:
            self.get_logger().info(f'[TRT] exporting {pt_path} → {engine_path} (imgsz={self.imgsz})')
            tmp = YOLO(pt_path)
            export_kwargs = dict(
                format='engine',
                imgsz=self.imgsz,
                half=self.use_half and not self.trt_int8,
                int8=self.trt_int8,
                device=0,
                batch=1,
                dynamic=False,
                simplify=True,
                workspace=self.trt_workspace_gb,
                verbose=False,
            )
            if self.trt_dla in (0, 1):
                export_kwargs['dla'] = self.trt_dla
            out = tmp.export(**export_kwargs)
            # Ultralytics có thể đặt engine ở thư mục weights; copy/rename về engine_path
            out = str(out)
            if os.path.isfile(out) and out != engine_path:
                try:
                    os.replace(out, engine_path)
                except OSError:
                    import shutil
                    shutil.copy2(out, engine_path)
            if os.path.isfile(engine_path):
                self.get_logger().info(f'[TRT] engine sẵn sàng: {engine_path}')
                return True
        except Exception as e:
            self.get_logger().warn(f'[TRT] export thất bại ({e}); fallback PyTorch')
        return False

    def _load_model(self) -> None:
        pt_path = self._resolve_weights_path()
        weights = pt_path
        # Try TensorRT engine (chỉ load nếu đã có sẵn — không export runtime)
        if self.use_trt and USE_GPU:
            engine_path = self._engine_path_for(pt_path)
            if os.path.isfile(engine_path):
                weights = engine_path
                self._using_engine = True
            elif self.auto_export:
                self.get_logger().warn(
                    'auto_export_engine=True: sẽ export TRT engine trong runtime '
                    '(có thể ngốn 4-6 GB RAM + 100% GPU; nguy cơ sập nguồn trên Jetson)')
                if os.path.isfile(pt_path) and self._export_engine(pt_path, engine_path):
                    weights = engine_path
                    self._using_engine = True
            else:
                self.get_logger().warn(
                    f'TRT engine chưa có ({engine_path}). Đang chạy PyTorch FP16. '
                    f'Để có hiệu năng tối đa, hãy export OFFLINE: '
                    f'  python3 -m realsense_yolo.export_engine --weights {pt_path} --imgsz {self.imgsz}')
        try:
            self.yolo = YOLO(weights)
            self.get_logger().info(
                f'YOLO loaded: {weights} | engine={self._using_engine} | '
                f'gpu={USE_GPU} | half={self.use_half}'
            )
        except Exception as e:
            self.get_logger().error(f'Failed to load YOLO weights={weights}: {e}')
            self.yolo = None

    def _resolve_tracker_cfg(self) -> str:
        explicit = (self._sp('tracker_config') or '').strip()
        if explicit and os.path.isfile(explicit):
            return explicit
        # share/realsense_yolo/config/botsort_deli.yaml (sau khi cài)
        try:
            from ament_index_python.packages import get_package_share_directory
            shared = os.path.join(
                get_package_share_directory('realsense_yolo'), 'config', 'botsort_deli.yaml'
            )
            if os.path.isfile(shared):
                return shared
        except Exception:
            pass
        # fallback: Ultralytics default
        return 'botsort.yaml'

    # ── Image callbacks ──────────────────────────────────────────────
    def color_callback(self, msg):
        if not self._logged_color:
            self.get_logger().info('Received first color frame')
            self._logged_color = True
        with self._lock:
            self._color_header = msg.header
            self._color_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            stamp_ns = int(msg.header.stamp.sec) * 1_000_000_000 + int(msg.header.stamp.nanosec)
            if stamp_ns == 0:
                stamp_ns = self._color_frames_received + 1
            self._color_stamp_ns = stamp_ns
            self._color_frames_received += 1
        self._frame_event.set()

    def depth_callback(self, msg):
        if not self._logged_depth:
            self.get_logger().info('Received first depth frame')
            self._logged_depth = True
        with self._lock:
            self._depth_header = msg.header
            self._depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

    def _get_frame_pair(self):
        with self._lock:
            if self._color_image is None or self._depth_image is None:
                return None
            color = self._color_image.copy()
            depth = self._depth_image.copy()
            header = self._depth_header
            stamp_ns = self._color_stamp_ns
        ch, cw = color.shape[:2]
        dh, dw = depth.shape[:2]
        if (ch, cw) != (dh, dw):
            if not self._logged_size_mismatch:
                self._logged_size_mismatch = True
                self.get_logger().warn(
                    f'Color {cw}x{ch} vs depth {dw}x{dh} — resize depth to color')
            depth = cv2.resize(depth, (cw, ch), interpolation=cv2.INTER_NEAREST)
        return (color, depth, header, stamp_ns)

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
        ax1, ay1, ax2, ay2 = a; bx1, by1, bx2, by2 = b
        ix1, iy1 = max(ax1, bx1), max(ay1, by1)
        ix2, iy2 = min(ax2, bx2), min(ay2, by2)
        iw, ih = max(0.0, ix2 - ix1), max(0.0, iy2 - iy1)
        inter = iw * ih
        if inter <= 0.0:
            return 0.0
        union = max(0.0, ax2 - ax1) * max(0.0, ay2 - ay1) \
              + max(0.0, bx2 - bx1) * max(0.0, by2 - by1) - inter
        return float(inter / union) if union > 0 else 0.0

    def _suppress_duplicate_boxes(self, dets):
        if not dets:
            return []
        dets_sorted = sorted(dets, key=lambda d: d['conf'], reverse=True)
        kept = []
        for d in dets_sorted:
            if any(d['cls_id'] == k['cls_id']
                   and self._iou_xyxy(d['xyxy'], k['xyxy']) >= self.dup_iou_thresh
                   for k in kept):
                continue
            kept.append(d)
        return kept

    @staticmethod
    def _class_color_bgr(cls_id):
        hue = (int(cls_id) * 37) % 180
        bgr = cv2.cvtColor(np.uint8([[[hue, 220, 255]]]), cv2.COLOR_HSV2BGR)[0, 0]
        return int(bgr[0]), int(bgr[1]), int(bgr[2])

    @staticmethod
    def _id_color_bgr(track_id: int):
        if track_id < 0:
            return 200, 200, 200
        hue = (int(track_id) * 53 + 17) % 180
        bgr = cv2.cvtColor(np.uint8([[[hue, 240, 255]]]), cv2.COLOR_HSV2BGR)[0, 0]
        return int(bgr[0]), int(bgr[1]), int(bgr[2])

    # ── Inference loop ────────────────────────────────────────────────
    def _inference_loop(self):
        if self.yolo is None:
            self._load_model()
        if self.yolo is None:
            self.get_logger().error('YOLO không load được — dừng inference loop')
            return

        tracker_cfg = self._resolve_tracker_cfg() if self.enable_tracking else None
        self.get_logger().info(
            f'inference loop start | tracker_cfg={tracker_cfg or "off"}')
        frame_count = 0         # số lần inference đã chạy (bao gồm frame mới)
        t0 = time.perf_counter()
        last_camera_frames_seen = 0  # snapshot counter để tính camera FPS thật
        logged_first = False
        last_stamp_ns = -1      # dedup: chỉ infer khi stamp đổi
        track_ms_sum = 0.0      # tổng thời gian yolo.track() trong cửa sổ 100 frame
        total_ms_sum = 0.0      # tổng thời gian full loop (incl. post, publish)

        # Predict kwargs chung
        predict_kwargs = dict(
            imgsz=self.imgsz,
            conf=self.conf_thresh,
            iou=self.iou_thresh,
            verbose=False,
            half=self.use_half and USE_GPU and not self._using_engine,
            augment=False,
            max_det=self.max_det,
            device=0 if USE_GPU else 'cpu',
        )

        while self._running and self.yolo is not None:
            try:
                pair = self._get_frame_pair()
                if pair is None:
                    time.sleep(0.002)
                    continue
                color, depth, header, stamp_ns = pair

                if stamp_ns == last_stamp_ns:
                    time.sleep(0.001)
                    continue
                last_stamp_ns = stamp_ns

                _loop_start = time.perf_counter()

                # ── YOLO inference (+ tracking) ─────────────────────
                _t_track = time.perf_counter()
                if self.enable_tracking:
                    results = self.yolo.track(
                        color, persist=True, tracker=tracker_cfg, **predict_kwargs)[0]
                else:
                    results = self.yolo(color, **predict_kwargs)[0]
                track_ms_sum += (time.perf_counter() - _t_track) * 1000.0

                will_publish_vis = (
                    self._publish_vis
                    and self.pub is not None
                    and (frame_count % self._vis_every_n == 0)
                )
                if will_publish_vis:
                    depth_np = np.asarray(depth, dtype=np.float32)
                    depth_8 = np.clip(depth_np / 10.0, 0, 255).astype(np.uint8)
                    vis = cv2.applyColorMap(depth_8, cv2.COLORMAP_JET)
                    hh, ww = vis.shape[:2]
                    thickness = max(2, int(2 * ww / 640))
                    font_scale = max(0.5, 0.5 * ww / 640)
                else:
                    vis = None

                # ── Trích xuất raw detections + track_id ───────────
                raw_dets = []
                boxes = results.boxes
                if boxes is not None and len(boxes) > 0:
                    xyxy = boxes.xyxy.cpu().numpy()
                    cls_arr = boxes.cls.cpu().numpy().astype(np.int32)
                    conf_arr = boxes.conf.cpu().numpy()
                    if boxes.id is not None:
                        id_arr = boxes.id.cpu().numpy().astype(np.int32)
                    else:
                        id_arr = np.full((len(xyxy),), -1, dtype=np.int32)
                    for i in range(len(xyxy)):
                        x1, y1, x2, y2 = (float(v) for v in xyxy[i])
                        cls_id = int(cls_arr[i])
                        conf = float(conf_arr[i])
                        track_id = int(id_arr[i])
                        cx = (x1 + x2) / 2; cy = (y1 + y2) / 2
                        depth_m = self._get_depth_at_point(depth, int(cx), int(cy)) or 0.0
                        raw_dets.append({
                            'xyxy': (x1, y1, x2, y2),
                            'cls_id': cls_id,
                            'conf': conf,
                            'depth_m': depth_m,
                            'track_id': track_id,
                            'label': results.names.get(cls_id, str(cls_id))
                                if hasattr(results.names, 'get') else
                                (results.names[cls_id] if 0 <= cls_id < len(results.names) else str(cls_id)),
                        })

                filtered = self._suppress_duplicate_boxes(raw_dets)
                det_data = []
                for d in filtered:
                    x1, y1, x2, y2 = d['xyxy']
                    if d['cls_id'] in self._primary_class_ids:
                        out_cls = d['cls_id']
                        label = d['label']
                    else:
                        out_cls = self._others_class_id
                        label = self._others_label
                    conf = d['conf']
                    depth_v = d['depth_m']
                    tid = d['track_id']
                    det_data.extend([x1, y1, x2, y2, depth_v, float(out_cls), conf, float(tid)])

                if vis is not None:
                    ww = vis.shape[1]
                    thickness = max(2, int(2 * ww / 640))
                    font_scale = max(0.5, 0.5 * ww / 640)
                    for d in filtered:
                        x1, y1, x2, y2 = d['xyxy']
                        out_cls = d['cls_id'] if d['cls_id'] in self._primary_class_ids else self._others_class_id
                        tid = d['track_id']
                        color_bgr = (
                            self._id_color_bgr(tid) if tid >= 0 else self._class_color_bgr(out_cls)
                        )
                        cv2.rectangle(vis, (int(x1), int(y1)), (int(x2), int(y2)),
                                      color_bgr, thickness)
                        depth_v = d['depth_m']
                        label = d['label'] if d['cls_id'] in self._primary_class_ids else self._others_label
                        depth_str = f'{depth_v:.2f}m' if depth_v > 0 else 'N/A'
                        tag = f'#{tid} ' if tid >= 0 else ''
                        cv2.putText(
                            vis, f'{tag}{label} {d["conf"]:.2f} {depth_str}',
                            (int(x1), max(0, int(y1) - 5)),
                            cv2.FONT_HERSHEY_SIMPLEX, font_scale, color_bgr,
                            max(1, thickness - 1),
                        )

                # ── Publish detections (luôn publish kể cả rỗng) ───
                det_msg = Float32MultiArray()
                n_det = len(det_data) // DET_FLOATS_PER_BOX
                det_msg.layout.dim.append(MultiArrayDimension(
                    label='detections', size=n_det, stride=DET_FLOATS_PER_BOX))
                det_msg.layout.dim.append(MultiArrayDimension(
                    label='vals', size=DET_FLOATS_PER_BOX, stride=1))
                det_msg.data = det_data
                self.det_pub.publish(det_msg)

                # ── Publish vis image (chỉ khi cần) ────────────────
                if vis is not None and self.pub is not None:
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
                total_ms_sum += (time.perf_counter() - _loop_start) * 1000.0
                # Log mỗi 100 lần inference (tức 100 frame camera mới sau khi dedup).
                #   infer_fps  = số lần YOLO thực sự chạy / s
                #   camera_fps = số color frame nhận được từ driver / s (phản ánh tỉ lệ drop)
                #   track_ms   = trung bình thời gian yolo.track() (pre + engine + post + tracker)
                #   loop_ms    = trung bình toàn loop (kể cả publish detections)
                if frame_count % 100 == 0:
                    elapsed = time.perf_counter() - t0
                    infer_fps = 100.0 / elapsed if elapsed > 0 else 0.0
                    with self._lock:
                        cam_frames_now = self._color_frames_received
                    cam_delta = cam_frames_now - last_camera_frames_seen
                    last_camera_frames_seen = cam_frames_now
                    cam_fps = cam_delta / elapsed if elapsed > 0 else 0.0
                    avg_track_ms = track_ms_sum / 100.0
                    avg_loop_ms = total_ms_sum / 100.0
                    self.get_logger().info(
                        f'infer_fps={infer_fps:.1f}  camera_fps={cam_fps:.1f}  '
                        f'track={avg_track_ms:.1f}ms  loop={avg_loop_ms:.1f}ms  '
                        f'(imgsz={self.imgsz}, engine={self._using_engine})'
                    )
                    t0 = time.perf_counter()
                    track_ms_sum = 0.0
                    total_ms_sum = 0.0
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
