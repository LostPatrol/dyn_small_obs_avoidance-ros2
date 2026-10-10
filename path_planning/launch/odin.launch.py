"""Standalone Odin bench planning: registered odom-frame inputs, no flight outputs."""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_planner(context):
    """Apply a radius override only when supplied; otherwise retain YAML/node defaults."""
    parameters = {
        'planning_frame': 'odom',
        # Odin uses device uptime. Reception freshness cannot detect upstream delay.
        'stamp_clock': 'receive',
        # Metres in the startup-relative frame; include a ground-level starting pose.
        # This is a search envelope, not a measured floor or flight clearance.
        'search.lower': [-50.0, -50.0, -2.0],
    }
    radius = LaunchConfiguration('blind_radius').perform(context).strip()
    if radius:
        parameters['cloud.blind_radius'] = float(radius)
    return [Node(package='path_planning', executable='path_planning_node', name='path_planning',
                 output='screen', parameters=[LaunchConfiguration('config'), parameters],
                 remappings=[('cloud', LaunchConfiguration('cloud')),
                             ('odom', LaunchConfiguration('odom')),
                             ('goal', LaunchConfiguration('goal'))])]


def generate_launch_description():
    config = str(Path(get_package_share_directory('path_planning')) / 'config/planner.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('config', default_value=config),
        DeclareLaunchArgument('cloud', default_value='/odin1/cloud_slam'),
        DeclareLaunchArgument('odom', default_value='/odin1/odometry'),
        DeclareLaunchArgument('goal', default_value='/goal'),
        DeclareLaunchArgument('blind_radius', default_value='',
                             description='Optional override; empty uses YAML, then the node default.'),
        OpaqueFunction(function=launch_planner),
    ])
