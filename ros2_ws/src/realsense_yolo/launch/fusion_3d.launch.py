"""
Launch Fusion 3D: RealSense + YOLO + LiDAR trong không gian 3D, xem bằng RViz.

Pipeline:
  1. Reset LiDAR serial (flush SCIP2 streaming left from previous kill)
  2. Start RealSense D435i + static TF (camera->laser)
  3. Start urg_node2 (lifecycle: unconfigured -> configure -> activate)
  4. After 5 s: YOLO node + Fusion 3D node + RViz

Calibration (camera trên đầu LiDAR, cùng hướng nhìn):
  lidar_x, lidar_y, lidar_z = vị trí gốc laser trong frame camera (m)
  Mặc định: (0, 0.05, 0) = laser ở dưới camera 5 cm.
  Quaternion (0.5, -0.5, 0.5, 0.5) = đồng trục camera Z forward = laser X forward.
"""

import os
import yaml
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, TimerAction, ExecuteProcess,
    RegisterEventHandler, EmitEvent,
)
from launch.substitutions import LaunchConfiguration
from launch.event_handlers import OnProcessStart, OnProcessExit
from launch.events import matches_action
from launch_ros.actions import Node, LifecycleNode
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from launch.conditions import IfCondition
from lifecycle_msgs.msg import Transition
from ament_index_python.packages import get_package_share_directory, PackageNotFoundError


def generate_launch_description():
    pkg = get_package_share_directory('realsense_yolo')

    has_urg_node2 = False
    urg_params = {
        'serial_port': '/dev/ttyACM0',
        'serial_baud': 115200,
        'frame_id': 'laser',
        'calibrate_time': False,
        'angle_min': -3.14,
        'angle_max': 3.14,
    }
    try:
        urg_pkg = get_package_share_directory('urg_node2')
        has_urg_node2 = True
        yaml_path = os.path.join(urg_pkg, 'config', 'params_serial.yaml')
        if os.path.isfile(yaml_path):
            with open(yaml_path, 'r') as f:
                urg_params = yaml.safe_load(f)['urg_node2']['ros__parameters']
    except (PackageNotFoundError, Exception):
        pass

    serial_port = urg_params.get('serial_port', '/dev/ttyACM0')
    rviz_config = os.path.join(pkg, 'rviz', 'fusion_3d.rviz')

    ld = LaunchDescription()

    ld.add_action(DeclareLaunchArgument('rgb_profile', default_value='640x480x15'))
    ld.add_action(DeclareLaunchArgument('depth_profile', default_value='640x480x15'))
    ld.add_action(DeclareLaunchArgument('laser_frame', default_value='laser'))
    ld.add_action(DeclareLaunchArgument(
        'lidar_x', default_value='0.0',
        description='Laser origin X in camera frame (m). Right positive.',
    ))
    ld.add_action(DeclareLaunchArgument(
        'lidar_y', default_value='0.05',
        description='Laser origin Y in camera frame (m). Down positive. 0.05 = laser 5cm below camera.',
    ))
    ld.add_action(DeclareLaunchArgument(
        'lidar_z', default_value='0.0',
        description='Laser origin Z in camera frame (m). Forward positive.',
    ))
    ld.add_action(DeclareLaunchArgument('use_rviz', default_value='true'))

    # ── RealSense ────────────────────────────────────────────────
    ld.add_action(Node(
        package='realsense2_camera',
        executable='realsense2_camera_node',
        name='camera', namespace='camera',
        parameters=[{
            'align_depth.enable': True,
            'enable_gyro': False, 'enable_accel': False,
            'enable_infra1': False, 'enable_infra2': False,
            'enable_sync': True,
            'rgb_camera.profile': LaunchConfiguration('rgb_profile'),
            'depth_module.profile': LaunchConfiguration('depth_profile'),
            'initial_reset': True,
        }],
        output='screen',
    ))

    # ── TF: camera_depth_optical_frame -> laser ──────────────────
    # Quaternion (0.5, -0.5, 0.5, 0.5) aligns:
    #   laser X (forward) → camera Z (forward)
    #   laser Y (left)    → camera -X (left)
    #   laser Z (up)      → camera -Y (up)
    ld.add_action(Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_to_laser_tf',
        arguments=[
            LaunchConfiguration('lidar_x'),
            LaunchConfiguration('lidar_y'),
            LaunchConfiguration('lidar_z'),
            '0.5', '-0.5', '0.5', '0.5',
            'camera_depth_optical_frame',
            LaunchConfiguration('laser_frame'),
        ],
        output='screen',
    ))

    # ── LiDAR: reset serial then start urg_node2 lifecycle ───────
    if has_urg_node2:
        lidar_reset = ExecuteProcess(
            cmd=['python3', '-c',
                 f"import os,termios,time\n"
                 f"try:\n"
                 f"  fd=os.open('{serial_port}',os.O_RDWR|os.O_NOCTTY|os.O_NONBLOCK)\n"
                 f"  termios.tcflush(fd,termios.TCIOFLUSH)\n"
                 f"  os.write(fd,b'QT\\n');time.sleep(0.3)\n"
                 f"  termios.tcflush(fd,termios.TCIOFLUSH)\n"
                 f"  os.close(fd);print('[lidar_reset] OK')\n"
                 f"except Exception as e: print(f'[lidar_reset] skip: {{e}}')\n"],
            output='screen',
        )
        ld.add_action(lidar_reset)

        lidar_node = LifecycleNode(
            package='urg_node2',
            executable='urg_node2_node',
            name='urg_node2',
            parameters=[urg_params],
            namespace='',
            output='screen',
        )

        # Start urg_node2 AFTER the serial reset finishes
        ld.add_action(RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=lidar_reset,
                on_exit=[lidar_node],
            ),
        ))

        # Lifecycle: process started → configure
        ld.add_action(RegisterEventHandler(
            event_handler=OnProcessStart(
                target_action=lidar_node,
                on_start=[EmitEvent(event=ChangeState(
                    lifecycle_node_matcher=matches_action(lidar_node),
                    transition_id=Transition.TRANSITION_CONFIGURE,
                ))],
            ),
        ))

        # Lifecycle: configured (inactive) → activate
        ld.add_action(RegisterEventHandler(
            event_handler=OnStateTransition(
                target_lifecycle_node=lidar_node,
                start_state='configuring',
                goal_state='inactive',
                entities=[EmitEvent(event=ChangeState(
                    lifecycle_node_matcher=matches_action(lidar_node),
                    transition_id=Transition.TRANSITION_ACTIVATE,
                ))],
            ),
        ))

    # ── After 5 s: YOLO + Fusion 3D + RViz ──────────────────────
    ld.add_action(TimerAction(
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
                additional_env={'LD_PRELOAD': '/lib/aarch64-linux-gnu/libgomp.so.1'},
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
                additional_env={'LD_PRELOAD': '/lib/aarch64-linux-gnu/libgomp.so.1'},
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
    ))

    return ld
