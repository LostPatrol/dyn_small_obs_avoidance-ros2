"""Launch the planner and optionally supplied ROS 2 sensor launch files."""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def include_sensor_launches(context):
    """Include only explicit ROS 2 FAST-LIO and Livox launch paths."""
    actions = []
    for argument in ('fast_lio_launch', 'livox_driver_launch'):
        path = LaunchConfiguration(argument).perform(context)
        if path:
            actions.append(IncludeLaunchDescription(AnyLaunchDescriptionSource(path)))
    return actions


def generate_launch_description():
    """Reuse the planner launch without assuming external sensor versions."""
    share = Path(get_package_share_directory('path_planning'))
    return LaunchDescription([
        DeclareLaunchArgument('fast_lio_launch', default_value='',
                              description='Absolute path to a ROS 2 FAST-LIO launch file'),
        DeclareLaunchArgument('livox_driver_launch', default_value='',
                              description='Absolute path to a ROS 2 Livox driver launch file'),
        OpaqueFunction(function=include_sensor_launches),
        IncludeLaunchDescription(AnyLaunchDescriptionSource(
            str(share / 'launch' / 'demo_withNOlidar.launch.py'))),
    ])
