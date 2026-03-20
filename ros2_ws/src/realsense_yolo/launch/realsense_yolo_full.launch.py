"""
Launch RealSense D435i + YOLO + optional RViz + optional OpenCV viewer.

Recommended: use_rviz:=false use_image_viewer:=true for reliable image display.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression

from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    rviz_config = os.path.join(
        get_package_share_directory('realsense_yolo'),
        'rviz', 'realsense_yolo.rviz'
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'align_depth',
            default_value='true',
            description='Align depth to color (required for bbox mapping)',
        ),
        DeclareLaunchArgument('yolo_model', default_value='yolov8n.pt'),
        DeclareLaunchArgument(
            'use_rviz',
            default_value='false',
            description='Launch RViz (use_image_viewer often works better)',
        ),
        DeclareLaunchArgument(
            'use_image_viewer',
            default_value='true',
            description='Launch OpenCV image viewer - reliable display',
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
                'color_width': 1280,
                'color_height': 720,
                'depth_width': 1280,
                'depth_height': 720,
                'color_fps': 30.0,
                'depth_fps': 30.0,
                'color_qos': 'DEFAULT',
                'depth_qos': 'DEFAULT',
            }],
            output='screen',
        ),

        Node(
            package='realsense_yolo',
            executable='realsense_yolo_node',
            name='realsense_yolo_node',
            parameters=[{
                'color_topic': '/camera/color/image_raw',
                'depth_topic': '/camera/aligned_depth_to_color/image_raw',
                'yolo_model': LaunchConfiguration('yolo_model'),
            }],
            output='screen',
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],
            output='screen',
            condition=IfCondition(
                PythonExpression(["'", LaunchConfiguration('use_rviz'), "' == 'true'"])
            ),
        ),

        Node(
            package='realsense_yolo',
            executable='image_viewer_node',
            name='image_viewer_node',
            parameters=[{'show_color': True}],
            output='screen',
            condition=IfCondition(
                PythonExpression(["'", LaunchConfiguration('use_image_viewer'), "' == 'true'"])
            ),
        ),
    ])
