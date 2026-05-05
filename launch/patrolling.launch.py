from ament_index_python.packages import PackageNotFoundError, get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare


def _include_tiago_gazebo_if_available(context):
    if LaunchConfiguration('start_tiago_gazebo').perform(context).lower() != 'true':
        return []

    world_name = LaunchConfiguration('world_name').perform(context)

    try:
        tiago_share = get_package_share_directory('tiago_gazebo')
        return [
            LogInfo(msg='Launching TIAGo Gazebo simulation.'),
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
            LogInfo(msg='tiago_gazebo package not found. Launching gazebo_ros fallback simulation.'),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(f'{gazebo_ros_dir}/launch/gazebo.launch.py')
            )
        ]
    except PackageNotFoundError:
        return [
            LogInfo(msg='No simulation package found (tiago_gazebo or gazebo_ros). Continuing with BT node only.')
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
            description='Start TIAGo Gazebo simulation before BT node',
        ),
        DeclareLaunchArgument(
            'world_name',
            default_value='small_office',
            description='Gazebo world for tiago_gazebo launch',
        ),
        OpaqueFunction(function=_include_tiago_gazebo_if_available),
        bt_launch,
    ])