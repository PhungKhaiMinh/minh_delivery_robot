"""
Launch RealSense D435i + YOLO + 2D LiDAR (UTM-30LX) fusion.

- RealSense + YOLO: depth map với bounding box + khoảng cách
- LiDAR: /scan (urg_node2 qua USB)
- Fusion: overlay LiDAR lên ảnh depth+YOLO, publish /realsense_yolo/fused_depth_lidar
- Cửa sổ: Camera Color, YOLO Depth+BBoxes, Fused Depth+LiDAR+YOLO

Trước khi chạy:
  1. Kiểm tra thiết bị: ./scripts/check_devices.sh
  2. Sửa cổng LiDAR nếu cần: /dev/ttyACM0 hoặc /dev/ttyUSB0 trong urg_node2/config/params_serial.yaml
  3. TF laser -> camera: mặc định camera trên đầu LiDAR ~5cm -> (lidar_x=0, lidar_y=0.05, lidar_z=0)
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.conditions import IfCondition
from ament_index_python.packages import get_package_share_directory, PackageNotFoundError


def generate_launch_description():
    pkg = get_package_share_directory('realsense_yolo')
    urg_serial_launch = os.path.join(pkg, 'launch', 'urg_serial.launch.py')
    has_urg = os.path.isfile(urg_serial_launch)
    try:
        get_package_share_directory('urg_node2')
        has_urg_node2 = True
    except PackageNotFoundError:
        has_urg_node2 = False

    return LaunchDescription([
        DeclareLaunchArgument('align_depth', default_value='true'),
        DeclareLaunchArgument('color_width', default_value='640'),
        DeclareLaunchArgument('color_height', default_value='480'),
        DeclareLaunchArgument('depth_width', default_value='640'),
        DeclareLaunchArgument('depth_height', default_value='480'),
        DeclareLaunchArgument('color_fps', default_value='15.0'),
        DeclareLaunchArgument('depth_fps', default_value='15.0'),
        DeclareLaunchArgument('scan_topic', default_value='/scan', description='LiDAR scan topic'),
        DeclareLaunchArgument('laser_frame', default_value='laser'),
        DeclareLaunchArgument('lidar_x', default_value='0.0', description='Laser X in camera frame (m)'),
        DeclareLaunchArgument('lidar_y', default_value='0.05', description='Laser Y: camera on top of LiDAR ~5cm'),
        DeclareLaunchArgument('lidar_z', default_value='0.0', description='Laser Z in camera frame (m)'),
        DeclareLaunchArgument('align_axes', default_value='true', description='True = camera Z and LiDAR X forward (coaxial)'),
        # RealSense
        Node(
            package='realsense2_camera',
            executable='realsense2_camera_node',
            name='camera',
            namespace='camera',
            parameters=[{
                'align_depth': LaunchConfiguration('align_depth'),
                'enable_gyro': False,
                'enable_accel': False,
                'enable_infra1': False,
                'enable_infra2': False,
                'enable_sync': False,
                'color_width': LaunchConfiguration('color_width'),
                'color_height': LaunchConfiguration('color_height'),
                'depth_width': LaunchConfiguration('depth_width'),
                'depth_height': LaunchConfiguration('depth_height'),
                'color_fps': ParameterValue(LaunchConfiguration('color_fps'), value_type=float),
                'depth_fps': ParameterValue(LaunchConfiguration('depth_fps'), value_type=float),
                'initial_reset': True,
            }],
            output='screen',
        ),

        # TF: camera -> laser (chỉnh trong camera_laser_tf_node.py; bật rotation: align_axes:=true)
        Node(
            package='realsense_yolo',
            executable='camera_laser_tf_node',
            name='camera_to_laser_tf',
            parameters=[{
                'parent_frame': 'camera_depth_optical_frame',
                'child_frame': LaunchConfiguration('laser_frame'),
                'lidar_x': LaunchConfiguration('lidar_x'),
                'lidar_y': LaunchConfiguration('lidar_y'),
                'lidar_z': LaunchConfiguration('lidar_z'),
                'align_axes': LaunchConfiguration('align_axes'),
            }],
            output='screen',
        ),

        # LiDAR (urg_node2 USB) - chạy nếu có urg_node2
        # Nếu chưa build urg_node2: chạy LiDAR riêng hoặc: colcon build --packages-select urg_node2
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(urg_serial_launch),
            launch_arguments={}.items(),
            condition=IfCondition("true" if (has_urg and has_urg_node2) else "false"),
        ),

        # Sau 5s: YOLO + Fusion + Viewer
        TimerAction(
            period=5.0,
            actions=[
                Node(
                    package='realsense_yolo',
                    executable='realsense_yolo_node',
                    name='realsense_yolo_node',
                    parameters=[{
                        'color_topic': '/camera/color/image_raw',
                        'depth_topic': '/camera/aligned_depth_to_color/image_raw',
                        'imgsz': 640,
                    }],
                    output='screen',
                ),
                Node(
                    package='realsense_yolo',
                    executable='lidar_depth_fusion_node',
                    name='lidar_depth_fusion_node',
                    parameters=[{
                        'depth_bbox_topic': '/realsense_yolo/depth_with_bboxes',
                        'scan_topic': LaunchConfiguration('scan_topic'),
                        'camera_info_topic': '/camera/aligned_depth_to_color/camera_info',
                        'output_topic': '/realsense_yolo/fused_depth_lidar',
                        'laser_frame': LaunchConfiguration('laser_frame'),
                        'lidar_max_range': 10.0,
                    }],
                    output='screen',
                ),
                Node(
                    package='realsense_yolo',
                    executable='image_viewer_node',
                    name='image_viewer_node',
                    parameters=[{
                        'show_color': True,
                        'fused_topic': '/realsense_yolo/fused_depth_lidar',
                    }],
                    output='screen',
                ),
            ],
        ),
    ])
