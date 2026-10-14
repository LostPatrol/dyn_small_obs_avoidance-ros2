"""Launch the ROS 2 planner with an existing point-cloud source."""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Use the configured sensor topics and original search settings."""
    share = Path(get_package_share_directory('path_planning'))
    return LaunchDescription([
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        Node(
            package='path_planning', executable='path_planning_node',
            name='path_planning_node1', output='screen',
            parameters=[str(share / 'config' / 'planner.yaml'),
                        {'use_sim_time': LaunchConfiguration('use_sim_time')}],
            remappings=[
                ('/cloud_registered', '/odin1/cloud_slam_down'),
                ('/mavros/odometry/out', '/odin1/odometry_highfreq'),
            ],
        ),
        Node(
            package='rviz2', executable='rviz2', name='rviz',
            arguments=['-d', str(share / 'launch' / 'demo.rviz')],
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
            condition=IfCondition(LaunchConfiguration('rviz')),
        ),
    ])
