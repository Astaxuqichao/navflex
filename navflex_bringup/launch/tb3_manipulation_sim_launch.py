import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    AppendEnvironmentVariable,
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
    RegisterEventHandler,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    navflex_bringup_dir = get_package_share_directory('navflex_bringup')
    tb3_manip_gazebo_dir = get_package_share_directory('turtlebot3_manipulation_gazebo')
    gazebo_ros_dir = get_package_share_directory('gazebo_ros')

    world = LaunchConfiguration('world')
    map_yaml_file = LaunchConfiguration('map')
    verbose = LaunchConfiguration('verbose')
    gazebo_master_uri = LaunchConfiguration('gazebo_master_uri')
    gui = LaunchConfiguration('gui')
    rviz = LaunchConfiguration('rviz')
    rviz_config_file = LaunchConfiguration('rviz_config')
    spawn_controllers = LaunchConfiguration('spawn_controllers')
    use_sim_time = LaunchConfiguration('use_sim_time')
    x_pose = LaunchConfiguration('x_pose')
    y_pose = LaunchConfiguration('y_pose')
    z_pose = LaunchConfiguration('z_pose')
    roll = LaunchConfiguration('roll')
    pitch = LaunchConfiguration('pitch')
    yaw = LaunchConfiguration('yaw')

    robot_description = Command([
        'xacro ',
        os.path.join(
            tb3_manip_gazebo_dir, 'urdf', 'turtlebot3_manipulation.urdf.xacro'),
        ' prefix:=""',
        ' use_sim:=true',
        ' use_fake_hardware:=false',
        ' fake_sensor_commands:=false',
    ])

    gzserver_launch = os.path.join(gazebo_ros_dir, 'launch', 'gzserver.launch.py')
    gzclient_launch = os.path.join(gazebo_ros_dir, 'launch', 'gzclient.launch.py')

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': use_sim_time,
        }],
        output='screen')

    gzserver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gzserver_launch),
        launch_arguments={
            'verbose': verbose,
            'world': world,
        }.items())

    gzclient = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gzclient_launch),
        condition=IfCondition(gui))

    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'turtlebot3_manipulation_system',
            '-x', x_pose,
            '-y', y_pose,
            '-z', z_pose,
            '-R', roll,
            '-P', pitch,
            '-Y', yaw,
        ],
        output='screen')

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        output='screen')

    imu_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['imu_broadcaster'],
        output='screen')

    arm_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['arm_controller'],
        output='screen')

    gripper_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['gripper_controller'],
        output='screen')

    spawn_controllers_after_spawn = RegisterEventHandler(
        condition=IfCondition(spawn_controllers),
        event_handler=OnProcessExit(
            target_action=spawn_entity,
            on_exit=[joint_state_broadcaster_spawner]))

    spawn_imu_after_joint_state = RegisterEventHandler(
        condition=IfCondition(spawn_controllers),
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[imu_broadcaster_spawner]))

    spawn_arm_after_imu = RegisterEventHandler(
        condition=IfCondition(spawn_controllers),
        event_handler=OnProcessExit(
            target_action=imu_broadcaster_spawner,
            on_exit=[arm_controller_spawner]))

    spawn_gripper_after_arm = RegisterEventHandler(
        condition=IfCondition(spawn_controllers),
        event_handler=OnProcessExit(
            target_action=arm_controller_spawner,
            on_exit=[gripper_controller_spawner]))

    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'yaml_filename': map_yaml_file,
        }])

    lifecycle_manager_map_server = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_map_server',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': True,
            'node_names': ['map_server'],
        }])

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen',
        condition=IfCondition(rviz))

    return LaunchDescription([
        DeclareLaunchArgument(
            'gazebo_master_uri',
            default_value='http://127.0.0.1:11346',
            description='Gazebo master URI used by gzserver and gzclient'),
        AppendEnvironmentVariable(
            'GAZEBO_RESOURCE_PATH',
            '/usr/share/gazebo-11'),
        AppendEnvironmentVariable(
            'GAZEBO_MODEL_PATH',
            '/usr/share/gazebo-11/models'),
        AppendEnvironmentVariable(
            'GAZEBO_MODEL_PATH',
            os.path.join(tb3_manip_gazebo_dir, 'models')),
        AppendEnvironmentVariable(
            'GAZEBO_PLUGIN_PATH',
            '/opt/ros/humble/lib'),
        SetEnvironmentVariable(
            'GAZEBO_MODEL_DATABASE_URI',
            ''),
        SetEnvironmentVariable(
            'GAZEBO_MASTER_URI',
            gazebo_master_uri),
        DeclareLaunchArgument(
            'world',
            default_value=os.path.join(
                navflex_bringup_dir, 'worlds', 'large_bright_room.world'),
            description='Gazebo world file'),
        DeclareLaunchArgument(
            'map',
            default_value=os.path.join(
                navflex_bringup_dir, 'maps', 'large_bright_room.yaml'),
            description='Map yaml file loaded by nav2_map_server'),
        DeclareLaunchArgument(
            'verbose',
            default_value='true',
            description='Print verbose Gazebo server output'),
        DeclareLaunchArgument(
            'gui',
            default_value='false',
            description='Start Gazebo client'),
        DeclareLaunchArgument(
            'rviz',
            default_value='true',
            description='Start RViz'),
        DeclareLaunchArgument(
            'rviz_config',
            default_value=os.path.join(
                navflex_bringup_dir, 'rviz', 'nav2_default_view.rviz'),
            description='RViz config file'),
        DeclareLaunchArgument(
            'spawn_controllers',
            default_value='true',
            description='Spawn ros2_control controllers if controller_manager is installed'),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation clock'),
        DeclareLaunchArgument(
            'x_pose',
            default_value='0.0',
            description='Initial robot x position in map/world'),
        DeclareLaunchArgument(
            'y_pose',
            default_value='0.0',
            description='Initial robot y position in map/world'),
        DeclareLaunchArgument(
            'z_pose',
            default_value='0.01',
            description='Initial robot z position in map/world'),
        DeclareLaunchArgument(
            'roll',
            default_value='0.0',
            description='Initial robot roll'),
        DeclareLaunchArgument(
            'pitch',
            default_value='0.0',
            description='Initial robot pitch'),
        DeclareLaunchArgument(
            'yaw',
            default_value='0.0',
            description='Initial robot yaw in map/world'),

        robot_state_publisher,
        map_server,
        lifecycle_manager_map_server,
        gzserver,
        gzclient,
        spawn_entity,
        spawn_controllers_after_spawn,
        spawn_imu_after_joint_state,
        spawn_arm_after_imu,
        spawn_gripper_after_arm,
        rviz_node,

        Node(
            package='navflex_bringup',
            executable='odom_fake_localization.py',
            name='odom_fake_localization',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'odom_topic': 'odom',
                'map_frame': 'map',
                'odom_frame': 'odom',
                'base_frame': 'base_footprint',
                'initial_pose.x': ParameterValue(x_pose, value_type=float),
                'initial_pose.y': ParameterValue(y_pose, value_type=float),
                'initial_pose.z': ParameterValue(z_pose, value_type=float),
                'initial_pose.yaw': ParameterValue(yaw, value_type=float),
            }]),

        LogInfo(msg='TurtleBot3 Manipulation Gazebo simulation launched.'),
    ])
