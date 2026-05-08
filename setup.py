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
            'launch/fusion_3d.launch.py',
        ]),
        ('share/' + package_name + '/rviz', ['rviz/fusion_3d.rviz']),
        ('share/' + package_name + '/config', ['config/botsort_deli.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Minh',
    maintainer_email='you@example.com',
    description='YOLO11 + RealSense + URG fusion for obstacle avoidance (fusion_3d)',
    license='BSD',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'realsense_yolo_node = realsense_yolo.realsense_yolo_node:main',
            'fusion_3d_node = realsense_yolo.fusion_3d_node:main',
            'export_engine = realsense_yolo.export_engine:main',
        ],
    },
)
