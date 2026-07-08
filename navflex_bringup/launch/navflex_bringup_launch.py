import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, SetEnvironmentVariable
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LoadComposableNodes, Node
from launch_ros.descriptions import ComposableNode, ParameterFile
from nav2_common.launch import RewrittenYaml


def launch_setup(context, *args, **kwargs):
    bringup_dir = get_package_share_directory('navflex_bringup')
    bt_dir = get_package_share_directory('navflex_bt_navigator')
    nav2_route_dir = get_package_share_directory('nav2_route')

    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    chassis_model = LaunchConfiguration('chassis_model')
    params_file_arg = LaunchConfiguration('params_file')
    bt_params_file = LaunchConfiguration('bt_params_file')
    bt_xml = LaunchConfiguration('default_nav_to_pose_bt_xml')
    use_respawn = LaunchConfiguration('use_respawn')
    use_composition = LaunchConfiguration('use_composition')
    use_intra_process_comms = LaunchConfiguration('use_intra_process_comms')
    use_bt_navigator = LaunchConfiguration('use_bt_navigator')
    container_name = LaunchConfiguration('container_name')
    log_level = LaunchConfiguration('log_level')
    graph_filepath = LaunchConfiguration('graph_filepath')
    use_route_server = LaunchConfiguration('use_route_server')

    with_route = use_route_server.perform(context).lower() in ('true', '1', 'yes')
    with_bt_navigator = use_bt_navigator.perform(context).lower() in ('true', '1', 'yes')
    selected_chassis = chassis_model.perform(context).lower()
    params_file = params_file_arg.perform(context)

    if not params_file:
        if selected_chassis == 'omni':
            params_file = os.path.join(bringup_dir, 'params', 'nav2_params.yaml')
        elif selected_chassis == 'diff':
            params_file = os.path.join(bringup_dir, 'params', 'nav2_params_tb3_diff.yaml')
        else:
            raise ValueError(
                f'Unsupported chassis_model "{selected_chassis}". '
                'Expected "omni" or "diff".')

    lifecycle_nodes = ['navflex_costmap_nav']
    if with_route:
        lifecycle_nodes.append('route_server')
    lifecycle_nodes.append('velocity_smoother')
    if with_bt_navigator:
        lifecycle_nodes.append('bt_navigator')

    remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]

    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=params_file,
            root_key=namespace,
            param_rewrites={
                'use_sim_time': use_sim_time,
                'autostart': autostart,
            },
            convert_types=True),
        allow_substs=True)

    composable_nodes = [
        ComposableNode(
            package='navflex_costmap_nav',
            plugin='navflex_costmap_nav::CostmapNavNode',
            name='navflex_costmap_nav',
            parameters=[
                {'use_sim_time': use_sim_time},
                configured_params,
            ],
            extra_arguments=[
                {'use_intra_process_comms': use_intra_process_comms},
            ],
            remappings=remappings),
        ComposableNode(
            package='nav2_velocity_smoother',
            plugin='nav2_velocity_smoother::VelocitySmoother',
            name='velocity_smoother',
            parameters=[configured_params],
            extra_arguments=[
                {'use_intra_process_comms': use_intra_process_comms},
            ],
            remappings=remappings + [('cmd_vel', 'cmd_vel_nav'), ('cmd_vel_smoothed', 'cmd_vel')]),
    ]

    if with_bt_navigator:
        composable_nodes.append(ComposableNode(
            package='nav2_bt_navigator',
            plugin='nav2_bt_navigator::BtNavigator',
            name='bt_navigator',
            parameters=[
                bt_params_file,
                {
                    'use_sim_time': use_sim_time,
                    'default_nav_to_pose_bt_xml': bt_xml,
                },
            ],
            remappings=remappings))

    composable_nodes.append(ComposableNode(
        package='nav2_lifecycle_manager',
        plugin='nav2_lifecycle_manager::LifecycleManager',
        name='lifecycle_manager_navflex',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'autostart': autostart},
            {'node_names': lifecycle_nodes},
            {'bond_timeout': 10.0},
            {'bond_heartbeat_period': 0.1},
            {'attempt_respawn_reconnection': True},
        ]))

    nodes = [
        Node(
            condition=UnlessCondition(use_composition),
            package='navflex_costmap_nav',
            executable='costmap_nav_node',
            name='navflex_costmap_nav',
            output='screen',
            respawn=use_respawn,
            respawn_delay=2.0,
            parameters=[
                {'use_sim_time': use_sim_time},
                configured_params,
            ],
            arguments=['--ros-args', '--log-level', log_level],
            remappings=remappings),

        Node(
            condition=IfCondition(use_composition),
            package='rclcpp_components',
            executable='component_container_isolated',
            name=container_name,
            output='screen',
            parameters=[configured_params],
            arguments=['--ros-args', '--log-level', log_level],
            remappings=remappings),

        LoadComposableNodes(
            condition=IfCondition(use_composition),
            target_container=container_name,
            composable_node_descriptions=composable_nodes),
    ]

    if with_route:
        nodes.append(Node(
            package='nav2_route',
            executable='route_server',
            name='route_server',
            output='screen',
            respawn=use_respawn,
            respawn_delay=2.0,
            arguments=['--ros-args', '--log-level', log_level],
            parameters=[
                {'use_sim_time': use_sim_time},
                {'graph_filepath': graph_filepath},
                {'route_frame': 'map'},
                {'base_frame': 'base_link'},
                {'max_iterations': 0},
                {'min_prune_dist_from_start': 1.0},
                {'min_prune_dist_from_goal': 1.0},
            ]))

    nodes.extend([
        Node(
            condition=UnlessCondition(use_composition) if with_bt_navigator else IfCondition('false'),
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=[
                bt_params_file,
                {
                    'use_sim_time': use_sim_time,
                    'default_nav_to_pose_bt_xml': bt_xml,
                },
            ],
            arguments=['--ros-args', '--log-level', log_level],
            remappings=remappings),

        Node(
            condition=UnlessCondition(use_composition),
            package='nav2_velocity_smoother',
            executable='velocity_smoother',
            name='velocity_smoother',
            output='screen',
            parameters=[configured_params],
            arguments=['--ros-args', '--log-level', log_level],
            remappings=remappings + [('cmd_vel', 'cmd_vel_nav'), ('cmd_vel_smoothed', 'cmd_vel')]),

        Node(
            condition=UnlessCondition(use_composition),
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navflex',
            output='screen',
            arguments=['--ros-args', '--log-level', log_level],
            parameters=[
                {'use_sim_time': use_sim_time},
                {'autostart': autostart},
                {'node_names': lifecycle_nodes},
                {'bond_timeout': 10.0},
                {'bond_heartbeat_period': 0.1},
                {'attempt_respawn_reconnection': True},
            ]),
    ])

    return nodes


def generate_launch_description():
    bringup_dir = get_package_share_directory('navflex_bringup')
    bt_dir = get_package_share_directory('navflex_bt_navigator')
    nav2_route_dir = get_package_share_directory('nav2_route')

    default_bt_params_file = os.path.join(bt_dir, 'params', 'navflex_bt_navigator.yaml')
    default_bt_xml = os.path.join(bt_dir, 'behavior_trees', 'test_bt_navigator.xml')
    default_graph = os.path.join(nav2_route_dir, 'graphs', 'sample_graph.geojson')

    return LaunchDescription([
        SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'),
        DeclareLaunchArgument('namespace', default_value='', description='Top-level namespace'),
        DeclareLaunchArgument('use_sim_time', default_value='true', description='Use simulation clock'),
        DeclareLaunchArgument('chassis_model', default_value='omni',
                              description='Chassis model selecting navigation parameters: omni or diff'),
        DeclareLaunchArgument('params_file', default_value='',
                              description='Optional parameter file override. Empty selects by chassis_model'),
        DeclareLaunchArgument('bt_params_file', default_value=default_bt_params_file,
                              description='Full path to the bt_navigator parameters file'),
        DeclareLaunchArgument('default_nav_to_pose_bt_xml', default_value=default_bt_xml,
                              description='Full path to the behavior tree XML file'),
        DeclareLaunchArgument('autostart', default_value='true',
                              description='Automatically configure and activate lifecycle nodes'),
        DeclareLaunchArgument('use_respawn', default_value='False',
                              description='Respawn navflex_costmap_nav if it exits'),
        DeclareLaunchArgument('use_composition', default_value='true',
                              description='Load NavFlex lifecycle nodes into a component container'),
        DeclareLaunchArgument('use_bt_navigator', default_value='true',
                              description='Launch the standard Nav2 bt_navigator'),
        DeclareLaunchArgument('use_intra_process_comms', default_value='true',
                              description='Use intra-process communication for compatible composable data-path nodes'),
        DeclareLaunchArgument('container_name', default_value='navflex_container',
                              description='Component container name when use_composition is True'),
        DeclareLaunchArgument('log_level', default_value='info', description='Log level'),
        DeclareLaunchArgument('graph_filepath', default_value=default_graph,
                              description='Full path to the navigation route graph file'),
        DeclareLaunchArgument('use_route_server', default_value='False',
                              description='Whether to launch nav2_route with the navflex stack'),
        OpaqueFunction(function=launch_setup),
    ])
