#!/usr/bin/env python3
"""Kiểm tra kết nối RealSense D435i và LiDAR (Hokuyo UTM-30LX). Chạy: python3 check_devices.py"""

import os
import sys
import subprocess

def run(cmd, check=False):
    try:
        return subprocess.run(cmd, shell=True, capture_output=True, text=True, timeout=5)
    except Exception as e:
        print(f"  Lỗi: {e}")
        return None

def main():
    print("=" * 50)
    print("  Kiểm tra thiết bị RealSense + LiDAR")
    print("=" * 50)

    # USB
    print("\n--- 1. USB (lsusb) ---")
    r = run("lsusb 2>/dev/null | grep -E 'Intel|RealSense|Hokuyo|Prolific|FTDI|CP210|CH340'")
    if r and r.stdout:
        print(r.stdout)
    else:
        print("  (Không thấy thiết bị quen thuộc hoặc chưa cài usbutils)")

    # Serial
    print("\n--- 2. Cổng serial (LiDAR) ---")
    for dev in ["/dev/ttyUSB0", "/dev/ttyUSB1", "/dev/ttyACM0", "/dev/ttyACM1"]:
        if os.path.exists(dev):
            print(f"  Tìm thấy: {dev}")
    if not any(os.path.exists(d) for d in ["/dev/ttyUSB0", "/dev/ttyUSB1", "/dev/ttyACM0", "/dev/ttyACM1"]):
        print("  Không thấy /dev/ttyUSB* hoặc /dev/ttyACM*. Thêm user vào dialout: sudo usermod -aG dialout $USER")

    # Video
    print("\n--- 3. Video (RealSense) ---")
    found = False
    for i in range(10):
        if os.path.exists(f"/dev/video{i}"):
            print(f"  /dev/video{i}")
            found = True
    if not found:
        print("  Không thấy /dev/video*")

    # ROS2 topics
    print("\n--- 4. ROS2 topics (khi đã chạy camera/lidar) ---")
    if "ROS_DISTRO" in os.environ:
        r = run("ros2 topic list 2>/dev/null")
        if r and r.stdout:
            for line in r.stdout.strip().split("\n"):
                if "scan" in line or "camera" in line or "realsense" in line or "depth" in line:
                    print(f"  {line}")
    else:
        print("  Chưa source ROS2: source /opt/ros/foxy/setup.bash")

    print("\n" + "=" * 50)

if __name__ == "__main__":
    main()
    sys.exit(0)
