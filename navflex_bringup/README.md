# navflex_bringup

`navflex_bringup` is the launch package for the Navflex navigation stack. It provides one launch file for the navigation stack itself and one launch file for local simulation.

## Launch Files

### navflex_bringup_launch.py

Starts the Navflex navigation runtime:

- `navflex_costmap_nav`
- `bt_navigator`
- `lifecycle_manager_navflex`
- optional `nav2_route` route server

The lifecycle manager activates nodes in this order:

1. `navflex_costmap_nav`
2. `route_server`, only when `use_route_server:=True`
3. `bt_navigator`

This ordering lets the action servers from `navflex_costmap_nav` become available before `bt_navigator` loads the behavior tree.

```bash
ros2 launch navflex_bringup navflex_bringup_launch.py
```

Common arguments:

| Argument | Default | Description |
| --- | --- | --- |
| `use_sim_time` | `false` | Use simulation time. |
| `params_file` | `navflex_bringup/params/nav2_params.yaml` | Parameters for `navflex_costmap_nav`. |
| `bt_params_file` | `navflex_bt_navigator/params/navflex_bt_navigator.yaml` | Parameters for `bt_navigator`. |
| `default_nav_to_pose_bt_xml` | `navflex_bt_navigator/behavior_trees/test_bt_navigator.xml` | Behavior tree XML used by NavigateToPose. |
| `autostart` | `true` | Automatically configure and activate lifecycle nodes. |
| `use_respawn` | `False` | Respawn `navflex_costmap_nav` if it exits. Only applies when `use_composition:=False`. |
| `use_composition` | `False` | Load `navflex_costmap_nav`, `bt_navigator`, and `lifecycle_manager_navflex` into one component container. |
| `container_name` | `navflex_container` | Component container name when `use_composition:=True`. |
| `log_level` | `info` | ROS log level. |
| `use_route_server` | `False` | Start `nav2_route` with the Navflex stack. |
| `graph_filepath` | `nav2_route/graphs/sample_graph.geojson` | Route graph file used when `use_route_server:=True`. |
| `namespace` | empty | Top-level namespace. |

Example with simulation time:

```bash
ros2 launch navflex_bringup navflex_bringup_launch.py use_sim_time:=true
```

Example with composition:

```bash
ros2 launch navflex_bringup navflex_bringup_launch.py \
  use_composition:=True \
  container_name:=navflex_container
```

When composition is enabled, `navflex_costmap_nav`, `bt_navigator`, and
`lifecycle_manager_navflex` run in the same container. The optional
`route_server` remains a separate process.

Example with route server:

```bash
ros2 launch navflex_bringup navflex_bringup_launch.py \
  use_route_server:=True \
  graph_filepath:=/path/to/graph.geojson
```

### sim_local_launch.py

Starts a local simulation environment. It does not start the Navflex navigation stack.

It launches:

- RViz through `nav2_bringup`
- `omni_fake_node`
- `simulation_lidar`
- `map_server`
- `lifecycle_manager_map`
- static transform `map -> odom`

```bash
ros2 launch navflex_bringup sim_local_launch.py
```

Common arguments:

| Argument | Default | Description |
| --- | --- | --- |
| `map_file` | `navflex_bringup/maps/map1.yaml` | Map loaded by `map_server`. |
| `rviz_config_file` | `navflex_bringup/rviz/nav2_default_view.rviz` | RViz config file synced from `nav2_bringup`. |
| `use_sim_time` | `true` | Use simulation time. |
| `autostart` | `true` | Automatically configure and activate lifecycle nodes. |

Use a custom map:

```bash
ros2 launch navflex_bringup sim_local_launch.py map_file:=/path/to/map.yaml
```

## Typical Workflow

Build and source the workspace:

```bash
cd /home/robot/ros2/nav_ws
colcon build --packages-select navflex_bringup
source install/setup.bash
```

Start local simulation:

```bash
ros2 launch navflex_bringup sim_local_launch.py
```

Start navigation in another terminal after map, TF, odometry, and sensor topics are available:

```bash
ros2 launch navflex_bringup navflex_bringup_launch.py
```

## Notes

- `navflex_bringup` owns bringup-level launch files, maps, RViz config, and bringup parameters.
- `sim_local_launch.py` was moved here from `navflex_costmap_nav` because it starts multiple packages and is not specific to the costmap navigation node.
- If `bt_navigator` reports that `compute_path_to_pose` or `follow_path` is unavailable, verify that `navflex_costmap_nav` reached the active lifecycle state first.
