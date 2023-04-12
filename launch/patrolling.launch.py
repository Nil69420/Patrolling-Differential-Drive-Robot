import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    tracking_dir = get_package_share_directory('tracking')

    tracking_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(tracking_dir, 'launch', 'tracking.launch.py')))
    
    patrolling_cmd = Node(
        package='bt_patrolling',
        executable='patrolling_main',
        parameters=[{
            'use_sim_time': True
        }],
        remappings=[
            ('input_scan', '/scan_raw')
            ('output_scan', '/nav_vel')
        ],
        output='screen'
    )

    ld = LaunchDescription()
    ld.add_action(tracking_cmd)
    ld.add_action(patrolling_cmd)

    return ld