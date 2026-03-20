"""
Launch RealSense D435i + YOLO + OpenCV Image Viewer (no RViz).

Use this for reliable image display. Press 'q' or ESC in the image window to quit.

Camera starts first; YOLO and viewer start after 5s to allow DDS discovery.

Default 640x480: reliable USB bandwidth. For max resolution use:
  color_width:=1280 color_height:=720 depth_width:=1280 depth_height:=720
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'align_depth',
            default_value='true',
            description='Align depth to color (required for bbox mapping)',
        ),
        DeclareLaunchArgument(
            'color_width',
            default_value='640',
            description='Color image width (640 reliable, 1280 max)',
        ),
        DeclareLaunchArgument(
            'color_height',
            default_value='480',
            description='Color image height (480 reliable, 720 max)',
        ),
        DeclareLaunchArgument(
            'depth_width',
            default_value='640',
            description='Depth image width (must match color for align)',
        ),
        DeclareLaunchArgument(
            'depth_height',
            default_value='480',
            description='Depth image height (must match color for align)',
        ),
        DeclareLaunchArgument(
            'color_fps',
            default_value='30.0',
            description='Color/depth FPS',
        ),
        DeclareLaunchArgument('yolo_model', default_value='yolov8n.pt'),
        DeclareLaunchArgument(
            'imgsz',
            default_value='640',
            description='YOLO input size: 416 faster, 640 balanced, 1280 accurate',
        ),
        DeclareLaunchArgument(
            'show_color',
            default_value='true',
            description='Show color camera window',
        ),
        Node(
            package='realsense2_camera',
            executable='realsense2_camera_node',
            name='camera',
            namespace='camera',
            parameters=[{
                'align_depth': LaunchConfiguration('align_depth', default='true'),
                'enable_gyro': False,
                'enable_accel': False,
                'enable_sync': False,
                'color_width': LaunchConfiguration('color_width', default='640'),
                'color_height': LaunchConfiguration('color_height', default='480'),
                'depth_width': LaunchConfiguration('depth_width', default='640'),
                'depth_height': LaunchConfiguration('depth_height', default='480'),
                'color_fps': LaunchConfiguration('color_fps', default='30.0'),
                'depth_fps': LaunchConfiguration('color_fps', default='30.0'),
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
                ),
                Node(
                    package='realsense_yolo',
                    executable='image_viewer_node',
                    name='image_viewer_node',
                    parameters=[{
                        'show_color': LaunchConfiguration('show_color', default='true'),
                    }],
                    output='screen',
                ),
            ],
        ),
    ])
