#!/bin/bash
# Kill stale RealSense/camera processes before launching.
# Use when you see "Device or resource busy" error.

pkill -f realsense2_camera_node 2>/dev/null
pkill -f realsense-viewer 2>/dev/null
sleep 2

cd "$(dirname "$0")/../../.."  # scripts -> realsense_yolo -> src -> workspace
source install/setup.bash
exec ros2 launch realsense_yolo realsense_yolo_view.launch.py "$@"
