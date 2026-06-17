import os
from ament_index_python.packages import get_package_share_directory, get_package_prefix
import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch.actions import SetEnvironmentVariable




def generate_launch_description():
    # Launch configuration variables
    map_file = LaunchConfiguration('map_file')
    rviz_config_file = LaunchConfiguration('rviz_config_file')

    omni_launch_path = os.path.join(
        get_package_share_directory('omni_fake_node'),
        'launch',
        'omni_fake_node.launch.py'
    )

    sim_lidar_launch_path = os.path.join(
        get_package_share_directory('simulation_lidar'),
        'launch',
        'simulation_lidar.launch.py'
    )
    

    return LaunchDescription([
        DeclareLaunchArgument(
        'rviz_config_file',
        default_value=os.path.join(
            get_package_share_directory('omni_fake_node'), 'rviz', 'nav2_default_view.rviz'),
        description='Full path to the RVIZ config file to use'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('omni_fake_node'), 'launch' , 'rviz_launch.py')),
                condition=IfCondition('True'),
                launch_arguments={'namespace': "",
                            'use_namespace': 'False',
                            'rviz_config': rviz_config_file}.items()),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(omni_launch_path)
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(sim_lidar_launch_path)
        ),
        # Declare launch argument
        DeclareLaunchArgument(
            'map_file',
            default_value=os.path.join(
                get_package_share_directory('omni_fake_node'), 'maps', 'map1.yaml'),
            description='Path to the map file'
        ),

        # Map Server (Lifecycle Node)
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{'yaml_filename': map_file}],
        ),

        # Lifecycle Manager to bring map_server to active state
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_map',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'autostart': True,  # Automatically configure and activate
                'node_names': ['map_server']
            }]
        ),

        # Static transform between map and odom
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_transform_publisher',
            output='screen',
            arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom']
        ),

        # Log message
        LogInfo(
            msg="Map Server and Lifecycle Manager launched."
        )
    ])
