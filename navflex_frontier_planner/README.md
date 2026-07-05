# navflex_frontier_planner

`navflex_frontier_planner` provides two ROS 2 `nav2_core::GlobalPlanner`
plugins for autonomous exploration in `navflex_costmap_nav`.

The exploration logic is based on FAEL-style frontier reasoning, not on a
Nav2 costmap:

- build an internal UFOMap occupancy map from point clouds;
- use TF to transform sensor points into the global frame;
- record changed voxels while inserting each point cloud;
- detect frontiers from free voxels whose XY neighbors include unknown space;
- update local frontiers from changed voxels and their XY neighbors;
- maintain global frontiers by re-validating old frontiers inside sensor range
  and preserving old frontiers outside sensor range;
- sample candidate viewpoints around the robot;
- attach visible frontiers to viewpoints and select a high-gain candidate;
- for `FrontierAStar`, plan to that candidate through the FAEL-style topology
  road graph.

## Plugins

### `navflex_frontier_planner/CandidateFrontierPlanner`

Runs frontier detection and candidate viewpoint selection. The returned path is
the scored candidate viewpoints ordered from best to worst. Each pose in the
path is one candidate point.

Use this plugin when you want another module to consume all candidate points.

### `navflex_frontier_planner/FrontierAStarPlanner`

Runs the same candidate selection, then computes a path over the FAEL-style
topology road graph.

Use this plugin when you want the planner result to include a path to the
selected frontier viewpoint.

## Inputs

This package does not subscribe to odometry directly.

Required input:

- `point_cloud_topic` (`sensor_msgs/msg/PointCloud2`), default: `point_cloud`
- TF from `frame_id` to the point cloud frame (`cloud.header.frame_id`)

The point cloud is transformed with:

```text
frame_id <- cloud.header.frame_id
```

The TF translation is also used as the sensor origin for ray insertion. Make
sure the TF tree can provide this transform at the point cloud timestamp.

## Outputs

Both planner plugins share the same UFOMap, candidate set, topology graph, and
visualization outputs. The selected candidate is published as:

```text
frontier_exploration/selected_candidate
```

Message type:

```text
geometry_msgs/msg/PoseStamped
```

```text
/frontier_exploration/selected_candidate
```

The package also publishes RViz visualization markers on:

```text
/frontier_exploration/candidates
/frontier_exploration/topology_map
```

`topology_map` is a FAEL-style combined topology visualization. It contains:

- `fael_current_position`: current robot/start node.
- `fael_viewpoints`: candidate viewpoints.
- `fael_attached_frontiers`: frontier cells attached to each viewpoint.
- `fael_viewpoint_frontier_links`: visibility links from viewpoints to their
  attached frontier cells.
- `fael_topology_graph_nodes` and `fael_topology_graph_edges`: the traversable
  FAEL-style road graph sampled from local free space and used by the A*
  planner. These nodes are independent from the candidate viewpoint set.
- `fael_selected_viewpoint`: the currently selected candidate target.
- `fael_best_topology_path`: the graph path to the selected target. This path
  is computed and published only when `FrontierAStar` is triggered.

The accumulated UFOMap can be inspected through debug PointCloud2 topics:

```text
/frontier_exploration/ufomap_occupied_cloud
/frontier_exploration/ufomap_free_cloud
```

For example:

```bash
ros2 topic echo /frontier_exploration/ufomap_occupied_cloud --once
ros2 topic hz /frontier_exploration/ufomap_free_cloud
```

In RViz, add two `PointCloud2` displays and set their topics to those two
clouds. The fixed frame should match `frame_id`, usually `map`.

## navflex Configuration Example

Add one or both planners to the `navflex_planner_server` parameters:

```yaml
navflex_planner_server:
  ros__parameters:
    planner_plugins: ["FrontierCandidate", "FrontierAStar"]

    FrontierCandidate:
      plugin: "navflex_frontier_planner/CandidateFrontierPlanner"
      frame_id: "map"
      point_cloud_topic: "/point_cloud"
      resolution: 0.4
      depth_levels: 16
      insert_depth: 0
      insert_discrete: true
      simple_ray_casting: false
      early_stopping: 0
      publish_map_clouds: true
      map_publish_period: 1.0
      max_range: 12.0
      sample_dist: 1.0
      local_range: 12.0
      candidate_visibility_range: 11.5
      reuse_cached_candidates: true
      cache_robot_move_threshold: 1.0
      candidate_recompute_period: 1.0
      frontier_attach_grid_size: 0.4
      global_frontier_revalidate_max_cells: 5000

    FrontierAStar:
      plugin: "navflex_frontier_planner/FrontierAStarPlanner"
      frame_id: "map"
      point_cloud_topic: "/point_cloud"
      resolution: 0.4
      depth_levels: 16
      insert_depth: 0
      insert_discrete: true
      simple_ray_casting: false
      early_stopping: 0
      publish_map_clouds: true
      map_publish_period: 1.0
      max_range: 12.0
      sample_dist: 1.0
      local_range: 12.0
      candidate_visibility_range: 11.5
      reuse_cached_candidates: true
      cache_robot_move_threshold: 1.0
      candidate_recompute_period: 1.0
      frontier_attach_grid_size: 0.4
      global_frontier_revalidate_max_cells: 5000
```

If your global frame is `world` or `odom`, set `frame_id` accordingly.

In the default omni local simulation from `navflex_bringup`, the simulated
LiDAR publishes PointCloud2 on:

```text
/scan_cloud
```

The omni bringup parameter file `nav2_params.yaml` therefore sets
`point_cloud_topic: "/scan_cloud"` for both frontier plugins by default.

## Trigger With ROS 2 Action

The planners are triggered through the `navflex_costmap_nav` planning action:

```text
/compute_path_to_pose
```

Action type:

```text
nav2_msgs/action/ComputePathToPose
```

The frontier plugins do not use the requested `goal` as the exploration target.
The `goal` field is still required by the action definition, but the plugin
selects the real goal from detected frontier candidates.

### Trigger candidate selection only

Use planner ID `FrontierCandidate`:

```bash
ros2 action send_goal /compute_path_to_pose nav2_msgs/action/ComputePathToPose "{
  goal: {
    header: {frame_id: map},
    pose: {
      position: {x: 0.0, y: 0.0, z: 0.0},
      orientation: {w: 1.0}
    }
  },
  start: {
    header: {frame_id: map},
    pose: {
      position: {x: 0.0, y: 0.0, z: 0.0},
      orientation: {w: 1.0}
    }
  },
  planner_id: FrontierCandidate,
  use_start: false,
  tolerance: 0.5
}"
```

This returns candidate points in `result.path.poses`, ordered from worst score
to best score, and publishes the shared candidate/topology outputs:

```text
/frontier_exploration/selected_candidate
/frontier_exploration/candidates
/frontier_exploration/topology_map
```

`FrontierCandidate` does not compute the best topology route. It only refreshes
candidate selection and topology graph visualization.

### Trigger frontier candidate plus topology planning

Use planner ID `FrontierAStar`:

```bash
ros2 action send_goal /compute_path_to_pose nav2_msgs/action/ComputePathToPose "{
  goal: {
    header: {frame_id: map},
    pose: {
      position: {x: 0.0, y: 0.0, z: 0.0},
      orientation: {w: 1.0}
    }
  },
  start: {
    header: {frame_id: map},
    pose: {
      position: {x: 0.0, y: 0.0, z: 0.0},
      orientation: {w: 1.0}
    }
  },
  planner_id: FrontierAStar,
  use_start: false,
  tolerance: 0.5
}"
```

This first refreshes the same shared candidate/topology outputs, then plans to
the best candidate using the topology road graph. If the start or selected
candidate cannot connect to nearby topology nodes, planning returns
`NO_PATH_FOUND`.
It publishes:

```text
/frontier_exploration/selected_candidate
/frontier_exploration/candidates
/frontier_exploration/topology_map
```

If `use_start` is `false`, `navflex_costmap_nav` uses the current robot pose as
the planning start. If you set `use_start: true`, fill the `start` field with a
valid pose in `frame_id`.

Check the selected candidate topic directly with:

```bash
ros2 topic echo /navflex_planner_server/FrontierAStar/selected_candidate
ros2 topic echo /frontier_exploration/selected_candidate
```

Common failure outcomes:

- `56 NO_PATH_FOUND`: no frontier candidate was found, or the topology graph
  could not connect to the selected candidate.
- `52 INVALID_START`: the start pose frame is empty.
- `53 INVALID_GOAL`: the required action goal frame is empty.

## Parameters

| Parameter | Default | Meaning |
| --- | ---: | --- |
| `frame_id` | `map` | Global frame used for the internal exploration map. |
| `point_cloud_topic` | `point_cloud` | Point cloud input topic. |
| `resolution` | `0.4` | UFOMap resolution in meters, matching FAEL `UFOMap.resolution`. |
| `depth_levels` | `16` | UFOMap tree depth levels. |
| `insert_depth` | `0` | UFOMap insertion depth. |
| `insert_discrete` | `true` | Use UFOMap discrete point cloud insertion, matching FAEL. |
| `simple_ray_casting` | `false` | Forwarded to UFOMap ray casting. |
| `early_stopping` | `0` | Forwarded to UFOMap insertion. |
| `publish_map_clouds` | `true` | Publish accumulated UFOMap free/occupied debug clouds. |
| `map_publish_period` | `1.0` | Minimum period in seconds between map cloud publications. |
| `max_range` | `12.0` | Maximum sensor insertion and frontier visibility range, matching FAEL `UFOMap.max_range`. |
| `sample_dist` | `1.0` | Viewpoint sampling spacing around the robot. |
| `local_range` | `12.0` | Radius used for candidate viewpoint sampling; this follows FAEL's effective viewpoint range from `max_range`. |
| `candidate_visibility_range` | `11.5` | Maximum range used when attaching frontiers to candidate viewpoints, matching FAEL's `max_range - 0.5` check. |
| `reuse_cached_candidates` | `true` | Reuse candidate/topology results when the map is unchanged and the robot has barely moved. |
| `cache_robot_move_threshold` | `1.0` | Maximum robot XY movement, in meters, allowed before recomputing cached candidates. |
| `candidate_recompute_period` | `1.0` | Minimum cache reuse window in seconds, even if new point clouds updated the map. |
| `frontier_attach_grid_size` | `0.4` | Grid size used to compact dense frontier cells into FAEL-style representative frontiers before attachment. |
| `global_frontier_revalidate_max_cells` | `5000` | Maximum nearby old frontier cells revalidated per planning cycle; distant/overflow frontiers are retained like FAEL. |
| `road_graph_dist` | `3.0` | Maximum connection distance for the FAEL-style free-space road graph, matching `Roadmap.connectable_range`. |
| `road_graph_connectable_num` | `3` | Maximum nearby road-graph neighbors connected from each node, matching `Roadmap.connectable_num`. |
| `viewpoint_gain_threshold` | `2.0` | Minimum FAEL-style information gain for a candidate viewpoint, matching `ViewpointManager.viewpoint_gain_thre`. |
| `min_frontier_area` | `0.05` | Reject viewpoints covering less frontier area, in square meters. |
| `candidate_separation` | `1.0` | Non-maximum-suppression distance between kept candidates. |
| `frontier_distance_weight` | `0.0` | Distance decay applied when scoring visible frontiers; FAEL counts attached/visible frontiers directly. |
| `min_candidate_count` | `8` | Target minimum candidate count; lower if not enough valid candidates exist. |
| `max_candidate_count` | `10` | Maximum candidates kept after scoring and suppression. |
| `frontier_gain` | `100.0` | Gain multiplier for visible frontiers, matching FAEL `RapidCoverPlanner.frontier_gain`. |
| `unknown_gain_range` | `1.5` | Distance sampled beyond each frontier to confirm unknown space. |
| `unknown_gain_step` | `0.2` | Step size for unknown-space gain sampling beyond frontiers. |
| `min_unknown_gain` | `0.0` | Minimum unknown cells beyond a frontier before it contributes gain. |
| `distance_weight` | `0.1` | Penalty weight for distance from robot to candidate. |
| `visited_radius` | `1.5` | Radius around previous robot poses treated as already visited. |
| `visited_penalty` | `1000.0` | Score penalty for candidates inside visited areas. |
| `known_gain_penalty` | `0.02` | Small score penalty for candidates surrounded by already-free cells. |
| `min_candidate_dist` | `0.5` | Reject candidate viewpoints too close to the robot. |
| `min_robot_frontier_dist` | `0.6` | Ignore frontiers too close to the robot. |
| `robot_clear_radius` | `0.3` | Clearance radius around occupied voxels. |
| `unknown_clear_radius` | `0.0` | Reject viewpoints too close to unknown space. |
| `viewpoint_free_z_min` | `0.0` | Lower vertical offset checked when deciding whether a sampled viewpoint is in free space. |
| `viewpoint_free_z_max` | `0.8` | Upper vertical offset checked when deciding whether a sampled viewpoint is in free space. |
| `viewpoint_free_z_step` | `0.1` | Vertical step for viewpoint free-space checks. |
| `sensor_height` | `0.45` | Height offset used when sampling viewpoints, matching FAEL `UFOMap.sensor_height`. |
| `frontier_slope_deg` | `89.0` | Slope filter for frontier detection; permissive for ROS2 ground simulation. |
| `viewpoint_slope_deg` | `15.0` | FAEL-like slope filter for frontier visibility. |

## Notes

- The `GlobalPlanner::configure()` interface still receives a
  `Costmap2DROS` pointer because this is required by Nav2's plugin API. This
  package does not read the costmap for exploration decisions.
- UFOMap is vendored under `third_party/ufomap` and built inside this package,
  so the ROS 1 FAEL tree is not required at build time.
- The internal map is built online from incoming point clouds. Candidate
  selection will fail until enough point cloud data and TF are available.
- Frontier maintenance follows the original FAEL local/global pattern:
  changed voxels produce local frontiers, old global frontiers are rechecked
  near the current sensor range, and distant global frontiers are retained.
- If TF is missing, the plugin logs a throttled warning and skips that cloud.

## Build

```bash
colcon build --packages-select navflex_frontier_planner \
  --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
```
