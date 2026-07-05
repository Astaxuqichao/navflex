from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('map_topic',        default_value='map'),
        DeclareLaunchArgument('global_frame',     default_value='map'),
        DeclareLaunchArgument('lidar_frame',      default_value='base_link'),
        DeclareLaunchArgument('min_angle',        default_value='-3.14159265'),
        DeclareLaunchArgument('max_angle',        default_value='3.14159265'),
        DeclareLaunchArgument('min_distance',     default_value='0.05'),
        DeclareLaunchArgument('max_distance',     default_value='30.0'),
        DeclareLaunchArgument('size',             default_value='400'),
        DeclareLaunchArgument('rate',             default_value='10.0'),
        DeclareLaunchArgument('noise',            default_value='0.02'),
        DeclareLaunchArgument('cloud_wall_height', default_value='1.0'),
        DeclareLaunchArgument('cloud_wall_step',   default_value='0.1'),
        DeclareLaunchArgument('use_sim_time',     default_value='true'),
        DeclareLaunchArgument('use_topic_odom',   default_value='false'),
        DeclareLaunchArgument('odom_topic',       default_value='/odom'),

        Node(
            package='simulation_lidar',
            executable='simulation_lidar_node',
            name='simulation_lidar_node',
            output='screen',
            parameters=[{
                'stage_map_topic': LaunchConfiguration('map_topic'),
                'global_frame':    LaunchConfiguration('global_frame'),
                'lidar_frame':     LaunchConfiguration('lidar_frame'),
                'min_angle':       LaunchConfiguration('min_angle'),
                'max_angle':       LaunchConfiguration('max_angle'),
                'min_distance':    LaunchConfiguration('min_distance'),
                'max_distance':    LaunchConfiguration('max_distance'),
                'size':            LaunchConfiguration('size'),
                'rate':            LaunchConfiguration('rate'),
                'noise':           LaunchConfiguration('noise'),
                'cloud_wall_height': LaunchConfiguration('cloud_wall_height'),
                'cloud_wall_step':  LaunchConfiguration('cloud_wall_step'),
                'use_sim_time':    LaunchConfiguration('use_sim_time'),
                'use_topic_odom':  LaunchConfiguration('use_topic_odom'),
                'odom_topic':      LaunchConfiguration('odom_topic'),
            }],
        ),
    ])
