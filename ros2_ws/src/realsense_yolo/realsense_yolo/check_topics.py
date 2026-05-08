#!/usr/bin/env python3
"""
Diagnostic: subscribe to camera topics and report if messages are received.
Run WITH the launch in another terminal: ros2 launch realsense_yolo realsense_yolo_view.launch.py

Usage: ros2 run realsense_yolo check_topics
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image


class TopicChecker(Node):
    def __init__(self):
        super().__init__('topic_checker')
        self.color_count = 0
        self.depth_count = 0
        self.yolo_count = 0

        self.create_subscription(
            Image, '/camera/color/image_raw',
            self._color_cb, 10
        )
        self.create_subscription(
            Image, '/camera/aligned_depth_to_color/image_raw',
            self._depth_cb, 10
        )
        self.create_subscription(
            Image, '/realsense_yolo/depth_with_bboxes',
            self._yolo_cb, 10
        )

        self.get_logger().info('Checking topics (launch must be running in another terminal)...')

    def _color_cb(self, msg):
        self.color_count += 1
        if self.color_count <= 3:
            self.get_logger().info(f'color frame {self.color_count}')

    def _depth_cb(self, msg):
        self.depth_count += 1
        if self.depth_count <= 3:
            self.get_logger().info(f'depth frame {self.depth_count}')

    def _yolo_cb(self, msg):
        self.yolo_count += 1
        if self.yolo_count <= 3:
            self.get_logger().info(f'YOLO frame {self.yolo_count}')


def main():
    rclpy.init()
    node = TopicChecker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    print(f'\n--- Summary: color={node.color_count} depth={node.depth_count} yolo={node.yolo_count} ---')
    if node.color_count == 0:
        print('NO color messages - QoS or connection issue with RealSense')
    if node.depth_count == 0:
        print('NO depth messages - check align_depth')
    if node.yolo_count == 0:
        print('NO YOLO messages - YOLO node may not be receiving/publishing')
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
