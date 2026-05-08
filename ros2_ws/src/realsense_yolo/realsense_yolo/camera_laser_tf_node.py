#!/usr/bin/env python3
"""
================================================================================
FILE HIỆU CHỈNH TRỤC TỌA ĐỘ CAMERA – LIDAR
================================================================================
Dùng file này khi bạn muốn tự chỉnh vị trí và hướng trục giữa camera và LiDAR.

• Vị trí (translation): sửa tham số lidar_x, lidar_y, lidar_z (m) – vị trí gốc
  laser trong frame camera. Mặc định: (0, 0.05, 0) = camera trên lidar 5 cm.

• Xoay (rotation): sửa block "if align:" bên dưới:
  - align_axes = True: dùng quaternion mặc định (camera Z = laser X = hướng ra trước).
  - Tự chỉnh: đặt quaternion (rotation.x, .y, .z, .w) theo quy ước ROS (child->parent).
  Camera optical: X phải, Y xuống, Z ra trước. Laser (Hokuyo): X ra trước, Y trái, Z lên.

Launch fusion_3d đã dùng node này. Mặc định align_axes=true (đồng trục camera-lidar).
Để rotation có hiệu lực: chạy với align_axes:=true
  ros2 launch realsense_yolo fusion_3d.launch.py align_axes:=true
================================================================================
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros import StaticTransformBroadcaster


class CameraLaserTFNode(Node):
    def __init__(self):
        super().__init__('camera_laser_tf_node')
        self.declare_parameter('parent_frame', 'camera_depth_optical_frame')
        self.declare_parameter('child_frame', 'laser')
        self.declare_parameter('lidar_x', 0.0)
        self.declare_parameter('lidar_y', 0.05)
        self.declare_parameter('lidar_z', 0.0)
        self.declare_parameter('align_axes', True)

        parent = self.get_parameter('parent_frame').get_parameter_value().string_value
        child = self.get_parameter('child_frame').get_parameter_value().string_value
        lx = self.get_parameter('lidar_x').get_parameter_value().double_value
        ly = self.get_parameter('lidar_y').get_parameter_value().double_value
        lz = self.get_parameter('lidar_z').get_parameter_value().double_value
        # Launch substitutions may pass this as bool or string.
        align_param = self.get_parameter('align_axes').get_parameter_value()
        if align_param.type == 1:  # PARAMETER_BOOL
            align = bool(align_param.bool_value)
        elif align_param.type == 4:  # PARAMETER_STRING
            align = str(align_param.string_value).strip().lower() in ('true', '1', 'yes', 'on')
        else:
            align = bool(self.get_parameter('align_axes').value)

        self._broadcaster = StaticTransformBroadcaster(self)
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = parent
        t.child_frame_id = child
        t.transform.translation.x = float(lx)
        t.transform.translation.y = float(ly)
        t.transform.translation.z = float(lz)
        if align:
            # Rotation used only when align_axes:=true.
            # Coaxial default: camera Z(forward) == laser X(forward)
            # R_camera_laser: laser X->camera Z, laser Y->camera -X, laser Z->camera -Y.
            t.transform.rotation.x = 0.5
            t.transform.rotation.y = -0.5
            t.transform.rotation.z = 0.5
            t.transform.rotation.w = 0.5
        else:
            t.transform.rotation.x = 0.0
            t.transform.rotation.y = 0.0
            t.transform.rotation.z = 0.0
            t.transform.rotation.w = 1.0

        self._broadcaster.sendTransform(t)
        self.get_logger().info(
            f'Static TF: {parent} -> {child} | translation=({lx},{ly},{lz}) m | align_axes={align}'
        )


def main(args=None):
    rclpy.init(args=args)
    node = CameraLaserTFNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
