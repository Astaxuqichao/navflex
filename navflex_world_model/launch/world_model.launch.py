from os.path import join

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    del args, kwargs
    params_file = LaunchConfiguration('params_file').perform(context)

    # Launch-argument overrides win over the params file, so the backend can be
    # flipped on from the command line without editing yaml.
    overrides = {
        'backend': LaunchConfiguration('backend'),
        'critic': LaunchConfiguration('critic'),
        'lingbot_url': LaunchConfiguration('lingbot_url'),
        'frame_num': LaunchConfiguration('frame_num'),
        'image_topic': LaunchConfiguration('image_topic'),
        'unavailable_verdict': LaunchConfiguration('unavailable_verdict'),
    }
    parameters = [params_file, overrides] if params_file else [overrides]

    return [
        Node(
            package='navflex_world_model',
            executable='navflex_world_model_node.py',
            name='navflex_world_model',
            output='screen',
            parameters=parameters,
        ),
    ]


def generate_launch_description():
    default_params = join(
        get_package_share_directory('navflex_world_model'),
        'params', 'world_model.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=default_params),
        DeclareLaunchArgument('backend', default_value='null'),
        DeclareLaunchArgument('critic', default_value='null'),
        DeclareLaunchArgument('lingbot_url', default_value='http://127.0.0.1:8100'),
        DeclareLaunchArgument('frame_num', default_value='81'),
        DeclareLaunchArgument('image_topic', default_value='/image_raw/compressed'),
        DeclareLaunchArgument('unavailable_verdict', default_value='needs_confirmation'),
        OpaqueFunction(function=launch_setup),
    ])
