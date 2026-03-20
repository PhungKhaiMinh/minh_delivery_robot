from setuptools import setup

package_name = 'realsense_yolo'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', [
            'launch/realsense_yolo.launch.py',
            'launch/realsense_yolo_full.launch.py',
            'launch/realsense_yolo_view.launch.py',
            'launch/urg_serial.launch.py',
            'launch/fusion_view.launch.py',
            'launch/fusion_3d.launch.py',
        ]),
        ('share/' + package_name + '/rviz', ['rviz/realsense_yolo.rviz', 'rviz/fusion_3d.rviz']),
    ],
    install_requires=['setuptools', 'numpy', 'opencv-python', 'ultralytics'],
    zip_safe=True,
    maintainer='Minh',
    maintainer_email='you@example.com',
    description='YOLO object detection + depth mapping for RealSense D435i',
    license='BSD',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'realsense_yolo_node = realsense_yolo.realsense_yolo_node:main',
            'image_viewer_node = realsense_yolo.image_viewer_node:main',
            'lidar_depth_fusion_node = realsense_yolo.lidar_depth_fusion_node:main',
            'fusion_3d_node = realsense_yolo.fusion_3d_node:main',
            'camera_laser_tf_node = realsense_yolo.camera_laser_tf_node:main',
            'check_topics = realsense_yolo.check_topics:main',
        ],
    },
)
