from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    navigation_type = LaunchConfiguration("navigation_type")
    params_file = LaunchConfiguration("params_file")
    return LaunchDescription([
        DeclareLaunchArgument("navigation_type", default_value="costmap"),
        DeclareLaunchArgument(
            "params_file",
            default_value=PathJoinSubstitution([
                FindPackageShare("navflex_nav"), "config", "navflex_nav.yaml"
            ]),
        ),
        Node(
            package="navflex_nav",
            executable="navflex_nav_node",
            name="navflex_nav",
            output="screen",
            parameters=[params_file, {"navigation_type": navigation_type}],
        ),
    ])
