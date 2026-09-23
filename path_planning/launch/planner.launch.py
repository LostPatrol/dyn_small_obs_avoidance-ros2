"""Launch only the standalone planner; remap externally registered cloud/odometry/goal inputs."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from pathlib import Path


def generate_launch_description():
    config = str(Path(get_package_share_directory('path_planning')) / 'config/planner.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('config', default_value=config),
        DeclareLaunchArgument('cloud', default_value='/cloud_registered'),
        DeclareLaunchArgument('odom', default_value='/Odometry'),
        DeclareLaunchArgument('goal', default_value='/goal'),
        Node(package='path_planning', executable='path_planning_node', name='path_planning',
             output='screen', parameters=[LaunchConfiguration('config')],
             remappings=[('cloud', LaunchConfiguration('cloud')), ('odom', LaunchConfiguration('odom')),
                         ('goal', LaunchConfiguration('goal'))]),
    ])
