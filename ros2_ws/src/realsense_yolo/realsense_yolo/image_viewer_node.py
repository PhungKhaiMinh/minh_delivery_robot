#!/usr/bin/env python3
"""
Simple OpenCV image viewer for RealSense + YOLO output.
Displays images at native resolution - guaranteed to work without RViz/rqt.
Press 'q' or ESC to quit (terminates entire launch).
"""

import os
import signal
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np


class ImageViewerNode(Node):
    def __init__(self):
        super().__init__('image_viewer_node')
        self.declare_parameter('yolo_topic', '/realsense_yolo/depth_with_bboxes')
        self.declare_parameter('color_topic', '/camera/color/image_raw')
        self.declare_parameter('fused_topic', '')
        self.declare_parameter('show_color', True)

        yolo_topic = self.get_parameter('yolo_topic').get_parameter_value().string_value
        color_topic = self.get_parameter('color_topic').get_parameter_value().string_value
        fused_topic = self.get_parameter('fused_topic').get_parameter_value().string_value
        try:
            self.show_color = self.get_parameter('show_color').get_parameter_value().bool_value
        except Exception:
            self.show_color = self.get_parameter('show_color').get_parameter_value().string_value.lower() in ('true', '1', 'yes')

        self.bridge = CvBridge()
        self.yolo_image = None
        self.color_image = None
        self.fused_image = None
        self._quit = False

        self.create_subscription(Image, yolo_topic, self._yolo_cb, 10)
        if self.show_color:
            self.create_subscription(Image, color_topic, self._color_cb, 10)
        if fused_topic:
            self.show_fused = True
            self.create_subscription(Image, fused_topic, self._fused_cb, 10)
        else:
            self.show_fused = False

        self.timer = self.create_timer(0.03, self._display_cb)
        self.get_logger().info(
            f'ImageViewer: {yolo_topic}' +
            (f', {color_topic}' if self.show_color else '') +
            (f', fused={fused_topic}' if fused_topic else '')
        )

    def _yolo_cb(self, msg):
        try:
            self.yolo_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'YOLO callback: {e}')

    def _color_cb(self, msg):
        try:
            self.color_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'Color callback: {e}')

    def _fused_cb(self, msg):
        try:
            self.fused_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'Fused callback: {e}')

    def _display_cb(self):
        if self._quit:
            return

        if self.yolo_image is not None:
            cv2.imshow('YOLO Depth + BBoxes', self.yolo_image)
        else:
            ph = np.zeros((480, 640, 3), dtype=np.uint8)
            cv2.putText(
                ph, 'Waiting for /realsense_yolo/depth_with_bboxes...',
                (80, 250), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2
            )
            cv2.imshow('YOLO Depth + BBoxes', ph)

        if self.show_color and self.color_image is not None:
            cv2.imshow('Camera Color', self.color_image)
        elif self.show_color:
            ph = np.zeros((480, 640, 3), dtype=np.uint8)
            cv2.putText(
                ph, 'Waiting for color...',
                (180, 250), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2
            )
            cv2.imshow('Camera Color', ph)

        if self.show_fused:
            if self.fused_image is not None:
                cv2.imshow('Fused Depth + LiDAR + YOLO', self.fused_image)
            else:
                ph = np.zeros((480, 640, 3), dtype=np.uint8)
                cv2.putText(
                    ph, 'Waiting for fused (LiDAR+Depth+YOLO)...',
                    (60, 250), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2
                )
                cv2.imshow('Fused Depth + LiDAR + YOLO', ph)

        key = cv2.waitKey(1) & 0xFF
        if key == ord('q') or key == 27:
            self._quit = True
            self.get_logger().info('User quit (q/ESC) — shutting down all nodes')
            self.timer.cancel()
            cv2.destroyAllWindows()
            os.kill(os.getpid(), signal.SIGINT)


def main(args=None):
    rclpy.init(args=args)
    node = ImageViewerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
