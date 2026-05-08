#!/usr/bin/env python3
"""
Fusion node: 2D LiDAR (UTM-30LX) + Depth map + YOLO detections.

- Subscribe: /realsense_yolo/depth_with_bboxes, /scan, camera_info (depth)
- Transform LiDAR points (laser frame) -> camera_depth_optical_frame
- Project LiDAR onto image and draw on depth+YOLO image
- Publish: /realsense_yolo/fused_depth_lidar (Image)
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, LaserScan, CameraInfo
from cv_bridge import CvBridge
import cv2
import numpy as np
from tf2_ros import Buffer, TransformListener, TransformException


class LidarDepthFusionNode(Node):
    def __init__(self):
        super().__init__('lidar_depth_fusion_node')

        self.declare_parameter('depth_bbox_topic', '/realsense_yolo/depth_with_bboxes')
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('camera_info_topic', '/camera/aligned_depth_to_color/camera_info')
        self.declare_parameter('output_topic', '/realsense_yolo/fused_depth_lidar')
        self.declare_parameter('laser_frame', 'laser')
        self.declare_parameter('camera_frame', 'camera_depth_optical_frame')
        self.declare_parameter('lidar_max_range', 10.0)
        self.declare_parameter('lidar_point_radius', 2)

        self.depth_topic = self.get_parameter('depth_bbox_topic').get_parameter_value().string_value
        self.scan_topic = self.get_parameter('scan_topic').get_parameter_value().string_value
        self.camera_info_topic = self.get_parameter('camera_info_topic').get_parameter_value().string_value
        self.output_topic = self.get_parameter('output_topic').get_parameter_value().string_value
        self.laser_frame = self.get_parameter('laser_frame').get_parameter_value().string_value
        self.camera_frame = self.get_parameter('camera_frame').get_parameter_value().string_value
        self.lidar_max_range = self.get_parameter('lidar_max_range').get_parameter_value().double_value
        self.point_radius = int(self.get_parameter('lidar_point_radius').get_parameter_value().integer_value)

        self.bridge = CvBridge()
        self._depth_image = None
        self._depth_header = None
        self._scan = None
        self._camera_info = None
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._logged_no_tf = False
        self._logged_no_camera_info = False

        self.create_subscription(Image, self.depth_topic, self._depth_cb, 10)
        self.create_subscription(LaserScan, self.scan_topic, self._scan_cb, 10)
        self.create_subscription(CameraInfo, self.camera_info_topic, self._camera_info_cb, 10)
        self._pub = self.create_publisher(Image, self.output_topic, 10)
        self._timer = self.create_timer(0.033, self._fusion_cb)  # ~30 Hz

        self.get_logger().info(
            f'Fusion: {self.depth_topic} + {self.scan_topic} -> {self.output_topic}'
        )

    def _depth_cb(self, msg):
        try:
            self._depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self._depth_header = msg.header
        except Exception as e:
            self.get_logger().error(f'Depth callback: {e}')

    def _scan_cb(self, msg):
        self._scan = msg

    def _camera_info_cb(self, msg):
        self._camera_info = msg

    def _fusion_cb(self):
        if self._depth_image is None:
            return
        if self._scan is None:
            vis = self._depth_image.copy()
            h, w = vis.shape[:2]
            cv2.putText(
                vis, 'Waiting for /scan (LiDAR)',
                (w // 4, h // 2), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2
            )
            self._pub.publish(self.bridge.cv2_to_imgmsg(vis, encoding='bgr8'))
            return

        vis = self._depth_image.copy()
        header = self._depth_header
        if header is None:
            header = self._scan.header

        if self._camera_info is None:
            if not self._logged_no_camera_info:
                self.get_logger().warn('No camera_info yet - LiDAR overlay skipped')
                self._logged_no_camera_info = True
            self._pub.publish(self.bridge.cv2_to_imgmsg(vis, encoding='bgr8'))
            return

        K = self._camera_info.k
        fx, fy = K[0], K[4]
        cx, cy = K[2], K[5]
        h, w = vis.shape[:2]

        try:
            t = self._tf_buffer.lookup_transform(
                self.camera_frame,
                self._scan.header.frame_id,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.1)
            )
        except TransformException as ex:
            if not self._logged_no_tf:
                self.get_logger().warn(f'TF not available: {ex}')
                self._logged_no_tf = True
            self._pub.publish(self.bridge.cv2_to_imgmsg(vis, encoding='bgr8'))
            return

        # LiDAR: (angle, range) -> 3D in laser frame (x forward, y left, z up typical)
        n = len(self._scan.ranges)
        angles = np.arange(n, dtype=np.float64) * self._scan.angle_increment + self._scan.angle_min
        ranges = np.array(self._scan.ranges, dtype=np.float32)
        valid = np.isfinite(ranges) & (ranges > 0) & (ranges < self.lidar_max_range)
        angles = angles[valid]
        ranges = ranges[valid]

        # Laser frame: ROS convention often x forward, y left, z up -> x = r*cos(angle), y = r*sin(angle)
        x_l = ranges * np.cos(angles)
        y_l = ranges * np.sin(angles)
        z_l = np.zeros_like(ranges)

        # Transform to camera optical frame: quaternion (x,y,z,w) -> R, then p_c = R @ p_l + t
        qx = t.transform.rotation.x
        qy = t.transform.rotation.y
        qz = t.transform.rotation.z
        qw = t.transform.rotation.w
        R = np.array([
            [1 - 2*qy*qy - 2*qz*qz, 2*qx*qy - 2*qw*qz, 2*qx*qz + 2*qw*qy],
            [2*qx*qy + 2*qw*qz, 1 - 2*qx*qx - 2*qz*qz, 2*qy*qz - 2*qw*qx],
            [2*qx*qz - 2*qw*qy, 2*qy*qz + 2*qw*qx, 1 - 2*qx*qx - 2*qy*qy]
        ])
        p_l = np.stack([x_l, y_l, z_l], axis=0)
        p_c = R @ p_l
        p_c[0, :] += t.transform.translation.x
        p_c[1, :] += t.transform.translation.y
        p_c[2, :] += t.transform.translation.z

        # Project to image: only points in front of camera (z > 0.1)
        z_c = p_c[2, :]
        valid_z = z_c > 0.1
        if not np.any(valid_z):
            self._pub.publish(self.bridge.cv2_to_imgmsg(vis, encoding='bgr8'))
            return

        x_c = p_c[0, valid_z]
        y_c = p_c[1, valid_z]
        z_c = z_c[valid_z]

        u = (fx * x_c / z_c + cx).astype(np.int32)
        v = (fy * y_c / z_c + cy).astype(np.int32)
        in_frame = (u >= 0) & (u < w) & (v >= 0) & (v < h)
        u = u[in_frame]
        v = v[in_frame]

        for ui, vi in zip(u, v):
            cv2.circle(vis, (int(ui), int(vi)), self.point_radius, (0, 255, 255), -1)

        out_msg = self.bridge.cv2_to_imgmsg(vis, encoding='bgr8')
        out_msg.header = header
        self._pub.publish(out_msg)


def main(args=None):
    rclpy.init(args=args)
    node = LidarDepthFusionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
