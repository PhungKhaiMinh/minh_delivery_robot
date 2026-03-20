"""
Launch Fusion 3D: RealSense + YOLO + LiDAR trong không gian 3D, xem bằng RViz.

- Depth image -> PointCloud2 + Image (depth map 2D trong RViz)
- LiDAR /scan -> PointCloud2 trong frame camera
- YOLO detections -> 3D bounding box (MarkerArray)

Mặc định: camera đặt trên đầu LiDAR, cách mặt quét laser ~5cm.
TF: lidar_x=0, lidar_y=0.05, lidar_z=0 (trục Y camera hướng xuống, laser ở +Y 5cm).
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

    rviz_config = os.path.join(pkg, 'rviz', 'fusion_3d.rviz')

    return LaunchDescription([
        DeclareLaunchArgument('align_depth', default_value='true'),
        DeclareLaunchArgument('color_width', default_value='640'),
        DeclareLaunchArgument('color_height', default_value='480'),
        DeclareLaunchArgument('depth_width', default_value='640'),
        DeclareLaunchArgument('depth_height', default_value='480'),
        DeclareLaunchArgument('color_fps', default_value='15.0'),
        DeclareLaunchArgument('depth_fps', default_value='15.0'),
        DeclareLaunchArgument('laser_frame', default_value='laser'),
        DeclareLaunchArgument('lidar_x', default_value='0.0', description='Laser X in camera frame (m)'),
        DeclareLaunchArgument('lidar_y', default_value='0.05', description='Laser Y in camera frame: camera on top, ~5cm -> +Y=0.05'),
        DeclareLaunchArgument('lidar_z', default_value='0.0', description='Laser Z in camera frame (m)'),
        DeclareLaunchArgument('align_axes', default_value='true', description='True = camera Z and LiDAR X forward (coaxial)'),
        DeclareLaunchArgument('use_rviz', default_value='true', description='Mở RViz 3D'),

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

        # TF: camera -> laser (chỉnh trục trong realsense_yolo/camera_laser_tf_node.py; bật rotation: align_axes:=true)
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

        # LiDAR (nếu có urg_node2)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(urg_serial_launch),
            launch_arguments={}.items(),
            condition=IfCondition("true" if (has_urg and has_urg_node2) else "false"),
        ),

        # Sau 5s: YOLO (có publish detections) + Fusion 3D + RViz
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
                        'confidence_threshold': 0.45,
                        'iou_threshold': 0.35,
                        'duplicate_iou_threshold': 0.55,
                        'detections_topic': '/realsense_yolo/detections',
                    }],
                    output='screen',
                ),
                Node(
                    package='realsense_yolo',
                    executable='fusion_3d_node',
                    name='fusion_3d_node',
                    parameters=[{
                        'depth_topic': '/camera/aligned_depth_to_color/image_raw',
                        'camera_info_topic': '/camera/aligned_depth_to_color/camera_info',
                        'scan_topic': '/scan',
                        'detections_topic': '/realsense_yolo/detections',
                        'depth_pointcloud_topic': '/realsense_yolo/depth_pointcloud',
                        'lidar_pointcloud_topic': '/realsense_yolo/lidar_pointcloud',
                        'markers_3d_topic': '/realsense_yolo/detection_boxes_3d',
                        'depth_image_plane_topic': '/realsense_yolo/depth_image_plane',
                        'camera_frame': 'camera_depth_optical_frame',
                        'laser_frame': LaunchConfiguration('laser_frame'),
                        'depth_scale': 0.001,
                        'depth_max_m': 10.0,
                        'depth_step': 4,
                        'depth_plane_step': 10,
                        'report_period_s': 1.0,
                        'lidar_max_range': 10.0,
                    }],
                    output='screen',
                ),
                Node(
                    package='rviz2',
                    executable='rviz2',
                    name='rviz2',
                    arguments=['-d', rviz_config],
                    output='screen',
                    condition=IfCondition(LaunchConfiguration('use_rviz')),
                ),
            ],
        ),
    ])
