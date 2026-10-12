"""Launch the standalone planner with CLI > YAML > default parameter precedence."""
from pathlib import Path
import runpy

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _launch_node(context):
    """Resolve the selected YAML and explicit arguments before constructing the node."""
    helper = runpy.run_path(str(Path(get_package_share_directory('path_planning')) /
                                'launch/parameter_config.py'))
    data = helper['load_config'](LaunchConfiguration('config').perform(context))
    parameters = helper['node_parameters'](
        data, ('/**', 'path_planning', '/path_planning'), {'planning_frame': 'odom', 'stamp_clock': 'receive',
         'search.lower': [-50.0, -50.0, -2.0]})
    parameters = helper['planner_overrides'](context, parameters)
    topics = helper['node_parameters'](data, ('odin_launch',), {
        'cloud': '/odin1/cloud_slam_down', 'odom': '/odin1/odometry',
        'goal': '/goal', 'time_index': '/demo_node/trajectory_time_index',
    })
    return [Node(package='path_planning', executable='path_planning_node', name='path_planning',
                 output='screen', parameters=[parameters],
                 remappings=[(name, helper['launch_value'](context, name, value))
                             for name, value in topics.items()])]


def generate_launch_description():
    """Declare optional arguments; empty values defer to the selected YAML configuration."""
    config = str(Path(get_package_share_directory('path_planning')) / 'config/planner.yaml')
    arguments = [DeclareLaunchArgument('config', default_value=config)]
    arguments.extend(DeclareLaunchArgument(name, default_value='') for name in (
        'cloud', 'odom', 'goal', 'time_index', 'blind_radius', 'map_mode',
        'planning_frame', 'stamp_clock', 'search_lower',
    ))
    return LaunchDescription([*arguments, OpaqueFunction(function=_launch_node)])
