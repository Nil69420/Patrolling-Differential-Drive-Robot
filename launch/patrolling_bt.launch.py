# Copyright 2026 Nil
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from ament_index_python.packages import PackageNotFoundError, get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _include_tracking_if_available(context):
    if LaunchConfiguration('start_tracking').perform(context).lower() != 'true':
        return []

    try:
        tracking_dir = get_package_share_directory('tracking')
    except PackageNotFoundError:
        return [
            LogInfo(
                msg=(
                    'tracking package not found in current overlay. '
                    'Continuing without tracking launch.'
                )
            )
        ]

    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [tracking_dir, '/launch/tracking.launch.py']
            )
        )
    ]


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    cmd_vel_topic = LaunchConfiguration('cmd_vel_topic')

    patrolling_cmd = Node(
        package='bt_patrolling',
        executable='patrolling_main',
        parameters=[{
            'use_sim_time': use_sim_time
        }],
        remappings=[
            ('/output_vel', cmd_vel_topic)
        ],
        output='screen'
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation clock if available',
        ),
        DeclareLaunchArgument(
            'cmd_vel_topic',
            default_value='/mobile_base_controller/cmd_vel_unstamped',
            description='Velocity command topic for the mobile base',
        ),
        DeclareLaunchArgument(
            'start_tracking',
            default_value='false',
            description='Start tracking package launch if available',
        ),
        OpaqueFunction(function=_include_tracking_if_available),
        patrolling_cmd,
    ])
