"""
Launch RealSense D435i + YOLO + OpenCV Image Viewer (no RViz).

Press 'q' or ESC in the image window to quit all nodes.

Camera starts first; YOLO and viewer start after 5s to allow DDS discovery.

Override profile:  ros2 launch ... rgb_profile:=1280x720x15 depth_profile:=1280x720x15
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction, Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('rgb_profile', default_value='640x480x15'),
        DeclareLaunchArgument('depth_profile', default_value='640x480x15'),
        DeclareLaunchArgument('yolo_model', default_value='yolov8n.pt'),
        DeclareLaunchArgument('imgsz', default_value='640'),
        DeclareLaunchArgument('show_color', default_value='true'),

        Node(
            package='realsense2_camera',
            executable='realsense2_camera_node',
            name='camera',
            namespace='camera',
            parameters=[{
                'align_depth.enable': True,
                'enable_gyro': False,
                'enable_accel': False,
                'enable_infra1': False,
                'enable_infra2': False,
                'enable_sync': True,
                'rgb_camera.profile': LaunchConfiguration('rgb_profile'),
                'depth_module.profile': LaunchConfiguration('depth_profile'),
                'initial_reset': True,
            }],
            output='screen',
        ),

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
                        'yolo_model': LaunchConfiguration('yolo_model'),
                        'imgsz': LaunchConfiguration('imgsz', default='640'),
                    }],
                    output='screen',
                    additional_env={'LD_PRELOAD': '/lib/aarch64-linux-gnu/libgomp.so.1'},
                ),
                Node(
                    package='realsense_yolo',
                    executable='image_viewer_node',
                    name='image_viewer_node',
                    parameters=[{
                        'show_color': LaunchConfiguration('show_color', default='true'),
                    }],
                    output='screen',
                    additional_env={'LD_PRELOAD': '/lib/aarch64-linux-gnu/libgomp.so.1'},
                    on_exit=Shutdown(reason='User closed viewer window'),
                ),
            ],
        ),
    ])
