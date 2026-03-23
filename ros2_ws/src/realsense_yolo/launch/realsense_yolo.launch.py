"""
Launch YOLO detection node + RViz for RealSense D435i.

Run RealSense driver separately first:
  ros2 launch realsense2_camera rs_launch.py align_depth.enable:=true

Then run this launch, or use realsense_yolo_full.launch.py to start both.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

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
            'color_topic',
            default_value='/camera/color/image_raw',
            description='RGB image topic',
        ),
        DeclareLaunchArgument(
            'depth_topic',
            default_value='/camera/aligned_depth_to_color/image_raw',
            description='Aligned depth image topic',
        ),
        DeclareLaunchArgument(
            'yolo_model',
            default_value='yolov8n.pt',
            description='YOLO model',
        ),
        DeclareLaunchArgument(
            'confidence',
            default_value='0.5',
            description='Detection confidence threshold',
        ),

        Node(
            package='realsense_yolo',
            executable='realsense_yolo_node',
            name='realsense_yolo_node',
            parameters=[{
                'color_topic': LaunchConfiguration('color_topic'),
                'depth_topic': LaunchConfiguration('depth_topic'),
                'yolo_model': LaunchConfiguration('yolo_model'),
                'confidence_threshold': LaunchConfiguration('confidence'),
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
        ),
    ])
