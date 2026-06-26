from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from os.path import join


def launch_setup(context, *args, **kwargs):
    del args, kwargs
    semantic_params_file = LaunchConfiguration('semantic_params_file').perform(context)
    semantic_parameters = [{
        'global_frame': LaunchConfiguration('global_frame'),
    }]
    if semantic_params_file:
        semantic_parameters.append(semantic_params_file)

    return [
        Node(
            package='navflex_instruction_server',
            executable='navflex_instruction_server_node.py',
            name='navflex_instruction_server',
            output='screen',
            parameters=[{
                'global_frame': LaunchConfiguration('global_frame'),
                'planner_id': LaunchConfiguration('planner_id'),
                'controller_id': LaunchConfiguration('controller_id'),
                'goal_checker_id': LaunchConfiguration('goal_checker_id'),
                'behavior_id': LaunchConfiguration('behavior_id'),
                'action_timeout': LaunchConfiguration('action_timeout'),
            }],
        ),
        Node(
            package='navflex_instruction_server',
            executable='navflex_semantic_map_server.py',
            name='navflex_semantic_map_server',
            output='screen',
            parameters=semantic_parameters,
        ),
        Node(
            package='navflex_instruction_server',
            executable='navflex_task_server.py',
            name='navflex_task_server',
            output='screen',
            parameters=[{
                'action_timeout': LaunchConfiguration('action_timeout'),
                'default_execute': LaunchConfiguration('default_execute'),
            }],
        ),
        Node(
            package='navflex_instruction_server',
            executable='navflex_vln_bridge.py',
            name='navflex_vln_bridge',
            output='screen',
            parameters=[{
                'action_timeout': LaunchConfiguration('action_timeout'),
            }],
        ),
    ]


def generate_launch_description():
    default_semantic_params = join(
        get_package_share_directory('navflex_instruction_server'),
        'params',
        'semantic_landmarks.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('global_frame', default_value='map'),
        DeclareLaunchArgument('planner_id', default_value='GridBased'),
        DeclareLaunchArgument('controller_id', default_value='FollowPath'),
        DeclareLaunchArgument('goal_checker_id', default_value='general_goal_checker'),
        DeclareLaunchArgument('behavior_id', default_value='cmd_behavior'),
        DeclareLaunchArgument('action_timeout', default_value='60.0'),
        DeclareLaunchArgument('default_execute', default_value='false'),
        DeclareLaunchArgument(
            'semantic_params_file',
            default_value=default_semantic_params),
        OpaqueFunction(function=launch_setup),
    ])
