import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    exploration_dir = get_package_share_directory('navflex_autonomous_exploration_bt')

    default_bt_xml = os.path.join(
        exploration_dir, 'behavior_trees', 'fael_frontier_exploration.xml')
    default_bt_params_file = os.path.join(
        exploration_dir, 'params', 'fael_exploration_bt_navigator.yaml')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true',
                              description='Use simulation clock'),
        DeclareLaunchArgument('bt_params_file', default_value=default_bt_params_file,
                              description='BT navigator parameter file for exploration'),
        DeclareLaunchArgument('default_nav_to_pose_bt_xml', default_value=default_bt_xml,
                              description='Autonomous exploration behavior tree XML'),
        DeclareLaunchArgument('use_exploration_bt_navigator', default_value='true',
                              description='Start the standalone exploration BT navigator'),
        DeclareLaunchArgument('exploration_start_topic', default_value='exploration/start',
                              description='std_msgs/Empty topic used to start exploration'),
        DeclareLaunchArgument('exploration_stop_topic', default_value='exploration/stop',
                              description='std_msgs/Empty topic used to stop exploration'),
        Node(
            condition=IfCondition(LaunchConfiguration('use_exploration_bt_navigator')),
            package='navflex_autonomous_exploration_bt',
            executable='exploration_bt_navigator',
            name='exploration_bt_navigator',
            output='screen',
            parameters=[
                LaunchConfiguration('bt_params_file'),
                {
                    'use_sim_time': LaunchConfiguration('use_sim_time'),
                    'bt_xml': LaunchConfiguration('default_nav_to_pose_bt_xml'),
                    'start_topic': LaunchConfiguration('exploration_start_topic'),
                    'stop_topic': LaunchConfiguration('exploration_stop_topic'),
                },
            ]),
    ])
