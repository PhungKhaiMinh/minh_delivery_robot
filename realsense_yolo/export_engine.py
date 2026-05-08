#!/usr/bin/env python3
"""
Export YOLO weights → TensorRT .engine OFFLINE.

LƯU Ý: Chạy script này trước khi launch hệ thống chính. Vì export sẽ:
  - Ngốn 4-6 GB RAM
  - Dùng 100 % GPU + onnx + onnxslim trong vài phút
  - Trên Jetson AGX Xavier nếu chạy đồng thời với RealSense + LiDAR có thể
    spike nguồn → sập máy.

Cách dùng:
  ros2 run realsense_yolo export_engine \
        --weights /mnt/jetson_data/deli_ws/yolo11n.pt \
        --imgsz 640 \
        [--dla 0]      # 0 hoặc 1 để build trên DLA core (không bắt buộc)
        [--int8]       # nếu có calibration set
        [--workspace 2]
"""

from __future__ import annotations
import argparse
import os
import sys
import time
from pathlib import Path

# Cùng compatibility shims như realsense_yolo_node.py
import numpy as np
if not hasattr(np, 'bool'):
    np.bool = bool          # type: ignore[attr-defined]
if not hasattr(np, 'float'):
    np.float = float        # type: ignore[attr-defined]
if not hasattr(np, 'int'):
    np.int = int            # type: ignore[attr-defined]
os.environ.setdefault('YOLO_AUTOINSTALL', 'False')
os.environ.setdefault('YOLO_OFFLINE', 'True')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--weights', default='/mnt/jetson_data/deli_ws/yolo11n.pt')
    ap.add_argument('--imgsz', type=int, default=640)
    ap.add_argument('--half', action='store_true', default=True)
    ap.add_argument('--no-half', dest='half', action='store_false')
    ap.add_argument('--int8', action='store_true', default=False)
    ap.add_argument('--dla', type=int, default=-1, help='0 hoặc 1 để dùng DLA core')
    ap.add_argument('--workspace', type=int, default=2)
    ap.add_argument('--out', default='', help='đường dẫn .engine đầu ra (mặc định cùng thư mục weights)')
    args = ap.parse_args()

    if not os.path.isfile(args.weights):
        print(f'ERROR: weights không tồn tại: {args.weights}', file=sys.stderr)
        return 2

    suffix = ''
    if args.dla in (0, 1):
        suffix += f'_dla{args.dla}'
    if args.int8:
        suffix += '_int8'
    elif args.half:
        suffix += '_fp16'
    out_path = args.out or (str(Path(args.weights).with_suffix('')) + f'_imgsz{args.imgsz}{suffix}.engine')

    if os.path.isfile(out_path):
        print(f'engine đã có sẵn → {out_path}')
        return 0

    print(f'Loading {args.weights} ...')
    from ultralytics import YOLO
    m = YOLO(args.weights)

    print(f'Exporting → {out_path} (imgsz={args.imgsz}, half={args.half and not args.int8}, '
          f'int8={args.int8}, dla={args.dla if args.dla in (0,1) else "off"})')
    t0 = time.perf_counter()
    kwargs = dict(
        format='engine',
        imgsz=args.imgsz,
        half=args.half and not args.int8,
        int8=args.int8,
        device=0,
        batch=1,
        dynamic=False,
        simplify=True,
        workspace=args.workspace,
        verbose=False,
    )
    if args.dla in (0, 1):
        kwargs['dla'] = args.dla
    out = m.export(**kwargs)
    out = str(out)
    if out != out_path and os.path.isfile(out):
        try:
            os.replace(out, out_path)
        except OSError:
            import shutil
            shutil.copy2(out, out_path)
    elapsed = time.perf_counter() - t0
    if os.path.isfile(out_path):
        size_mb = os.path.getsize(out_path) / 1e6
        print(f'OK ({elapsed:.1f}s) → {out_path} ({size_mb:.1f} MB)')
        return 0
    print(f'FAILED ({elapsed:.1f}s)', file=sys.stderr)
    return 1


if __name__ == '__main__':
    sys.exit(main())
