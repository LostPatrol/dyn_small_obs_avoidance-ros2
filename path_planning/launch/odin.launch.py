"""Standalone Odin bench planning: registered odom-frame inputs, no flight outputs."""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    """Declare inputs and apply a radius override only when explicitly supplied."""
    config = str(Path(get_package_share_directory('path_planning')) / 'config/planner.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('config', default_value=config),
        DeclareLaunchArgument('cloud', default_value='/odin1/cloud_slam_down'),
        DeclareLaunchArgument('odom', default_value='/odin1/odometry'),
        DeclareLaunchArgument('goal', default_value='/goal'),
        DeclareLaunchArgument('time_index', default_value='/demo_node/trajectory_time_index'),
        DeclareLaunchArgument('blind_radius', default_value='',
                              description='Override cloud.blind_radius; omitted uses YAML or node default 0.'),
        OpaqueFunction(function=_launch_node),
    ])


def _launch_node(context):
    """Keep the YAML radius unless the command line explicitly overrides it."""
    overrides = {}
    if LaunchConfiguration('blind_radius').perform(context):
        overrides['cloud.blind_radius'] = ParameterValue(
            LaunchConfiguration('blind_radius'), value_type=float)
    return [Node(package='path_planning', executable='path_planning_node', name='path_planning',
             output='screen', parameters=[LaunchConfiguration('config'), {
                 'planning_frame': 'odom',
                 # Odin uses device uptime. Reception freshness cannot detect upstream delay.
                 'stamp_clock': 'receive',
                 # Metres in the startup-relative frame; include a ground-level starting pose.
                 # This is a search envelope, not a measured floor or flight clearance.
                 'search.lower': [-50.0, -50.0, -2.0],
             }, overrides], remappings=[('cloud', LaunchConfiguration('cloud')),
                             ('odom', LaunchConfiguration('odom')),
                             ('goal', LaunchConfiguration('goal')),
                             ('time_index', LaunchConfiguration('time_index'))]),
    ]
