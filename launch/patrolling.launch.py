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
from launch_ros.substitutions import FindPackageShare


def _include_simulation_if_available(context):
    if LaunchConfiguration('start_tiago_gazebo').perform(context).lower() != 'true':
        return []

    simulation_backend = LaunchConfiguration('simulation_backend').perform(context).lower()
    if simulation_backend not in ('auto', 'ignition', 'classic'):
        simulation_backend = 'auto'

    world_name = LaunchConfiguration('world_name').perform(context)
    ignition_world = LaunchConfiguration('ignition_world').perform(context)

    if simulation_backend in ('auto', 'ignition'):
        try:
            ros_ign_share = get_package_share_directory('ros_ign_gazebo')
            return [
                LogInfo(msg='Launching Ignition Gazebo simulation via ros_ign_gazebo.'),
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        f'{ros_ign_share}/launch/ign_gazebo.launch.py'
                    ),
                    launch_arguments={'ign_args': f'-r {ignition_world}'}.items(),
                )
            ]
        except PackageNotFoundError:
            pass

    try:
        tiago_share = get_package_share_directory('tiago_gazebo')
        return [
            LogInfo(msg='Launching TIAGo Gazebo classic simulation.'),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(f'{tiago_share}/launch/tiago_gazebo.launch.py'),
                launch_arguments={'world_name': world_name}.items(),
            )
        ]
    except PackageNotFoundError:
        pass

    try:
        gazebo_ros_dir = get_package_share_directory('gazebo_ros')
        return [
            LogInfo(
                msg=(
                    'Neither ros_ign_gazebo nor tiago_gazebo found. '
                    'Launching gazebo_ros fallback simulation.'
                )
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(f'{gazebo_ros_dir}/launch/gazebo.launch.py')
            )
        ]
    except PackageNotFoundError:
        return [
            LogInfo(
                msg=(
                    'No simulation package found '
                    '(ros_ign_gazebo, tiago_gazebo, or gazebo_ros). '
                    'Continuing with BT node only.'
                )
            )
        ]


def generate_launch_description():
    bt_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare('bt_patrolling'), '/launch/patrolling_bt.launch.py']
        ),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'cmd_vel_topic': LaunchConfiguration('cmd_vel_topic'),
            'start_tracking': LaunchConfiguration('start_tracking'),
        }.items(),
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
        DeclareLaunchArgument(
            'start_tiago_gazebo',
            default_value='true',
            description='Start simulator before BT node',
        ),
        DeclareLaunchArgument(
            'simulation_backend',
            default_value='ignition',
            description='Simulation backend preference: ignition, classic, or auto',
        ),
        DeclareLaunchArgument(
            'ignition_world',
            default_value='empty.sdf',
            description='Ignition world passed through ros_ign_gazebo ign_args',
        ),
        DeclareLaunchArgument(
            'world_name',
            default_value='small_office',
            description='Gazebo world for tiago_gazebo launch',
        ),
        OpaqueFunction(function=_include_simulation_if_available),
        bt_launch,
    ])
