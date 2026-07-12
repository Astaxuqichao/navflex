"""Launch the ROG map in standalone or composed mode."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, LifecycleNode, Node
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    """Create a Nav2-style lifecycle launch description."""
    params = os.path.join(
        get_package_share_directory('navflex_rog_map'), 'config', 'rog_map.yaml')
    use_composition = LaunchConfiguration('use_composition')
    autostart = LaunchConfiguration('autostart')

    return LaunchDescription([
        DeclareLaunchArgument('use_composition', default_value='False'),
        DeclareLaunchArgument('autostart', default_value='True'),
        LifecycleNode(
            condition=UnlessCondition(use_composition),
            package='navflex_rog_map',
            executable='rog_map_server',
            name='rog_map',
            parameters=[params],
            output='screen'),
        ComposableNodeContainer(
            condition=IfCondition(use_composition),
            package='rclcpp_components',
            executable='component_container_mt',
            name='rog_map_container',
            output='screen',
            composable_node_descriptions=[
                ComposableNode(
                    package='navflex_rog_map',
                    plugin='navflex_rog_map::RogMapROS',
                    name='rog_map',
                    parameters=[params])
            ]),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_rog_map',
            output='screen',
            parameters=[{
                'autostart': autostart,
                'node_names': ['rog_map']
            }])
    ])
