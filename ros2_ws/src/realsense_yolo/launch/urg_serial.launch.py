# Launch urg_node2 với kết nối USB (params_serial) cho LiDAR Hokuyo UTM-30LX.
# Cần có urg_node2 trong workspace và build: colcon build --packages-select urg_node2

import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, EmitEvent
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.event_handlers import OnProcessStart
from launch.events import matches_action
from launch_ros.actions import LifecycleNode
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition


def generate_launch_description():
    try:
        pkg_share = get_package_share_directory('urg_node2')
        config_path = os.path.join(pkg_share, 'config', 'params_serial.yaml')
    except Exception:
        config_path = None

    if config_path and os.path.isfile(config_path):
        with open(config_path, 'r') as f:
            config_params = yaml.safe_load(f)['urg_node2']['ros__parameters']
    else:
        config_params = {
            'serial_port': '/dev/ttyACM0',
            'serial_baud': 115200,
            'frame_id': 'laser',
            'calibrate_time': False,
            'angle_min': -3.14,
            'angle_max': 3.14,
        }

    lifecycle_node = LifecycleNode(
        package='urg_node2',
        executable='urg_node2_node',
        name=LaunchConfiguration('node_name', default='urg_node2'),
        parameters=[config_params],
        namespace='',
        output='screen',
    )

    configure_handler = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=lifecycle_node,
            on_start=[
                EmitEvent(event=ChangeState(
                    lifecycle_node_matcher=matches_action(lifecycle_node),
                    transition_id=Transition.TRANSITION_CONFIGURE,
                )),
            ],
        ),
        condition=IfCondition(LaunchConfiguration('auto_start', default='true')),
    )

    activate_handler = RegisterEventHandler(
        event_handler=OnStateTransition(
            target_lifecycle_node=lifecycle_node,
            start_state='configuring',
            goal_state='inactive',
            entities=[
                EmitEvent(event=ChangeState(
                    lifecycle_node_matcher=matches_action(lifecycle_node),
                    transition_id=Transition.TRANSITION_ACTIVATE,
                )),
            ],
        ),
        condition=IfCondition(LaunchConfiguration('auto_start', default='true')),
    )

    return LaunchDescription([
        DeclareLaunchArgument('auto_start', default_value='true'),
        DeclareLaunchArgument('node_name', default_value='urg_node2'),
        lifecycle_node,
        configure_handler,
        activate_handler,
    ])
