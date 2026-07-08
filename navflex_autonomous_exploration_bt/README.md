# navflex_autonomous_exploration_bt

This package provides a standalone behavior-tree configuration for autonomous
FAEL-style frontier exploration with NavFlex.

The tree repeatedly:

1. calls `NavflexGetPathAction` on `/compute_path_to_pose` with planner ID
   `FrontierAStar`;
2. lets `navflex_frontier_planner/FrontierAStarPlanner` select and plan to the
   best FAEL-style frontier viewpoint;
3. sends the returned path to `NavflexExePathAction` on `/follow_path` with
   controller ID `FollowPath`;
4. performs costmap clearing plus simple rotate/wait/back-up recovery actions
   when planning or following fails.

When the frontier planner can no longer produce a path after recovery retries,
the repeated exploration loop exits and the wrapping `ForceSuccess` lets the
NavigateToPose action finish cleanly.

## Build

```bash
colcon build --packages-select navflex_autonomous_exploration_bt
```

## Launch

```bash
ros2 launch navflex_autonomous_exploration_bt fael_exploration.launch.py
```

The launch file wraps `navflex_bringup navflex_bringup_launch.py` and replaces
only the `bt_navigator` XML and plugin parameter file. The default NavFlex
bringup parameters already include the `FrontierAStar` planner and use
`/scan_cloud` as the point-cloud input.

## Trigger exploration

By default the launch file starts `exploration_bt_navigator`, a standalone BT
runner for this exploration tree. It does not require sending a Nav2
`NavigateToPose` goal. Start and stop exploration with:

```bash
ros2 topic pub --once /exploration/start std_msgs/msg/Empty "{}"
ros2 topic pub --once /exploration/stop std_msgs/msg/Empty "{}"
```

The runner loads `fael_frontier_exploration.xml` directly and ticks it until the
frontier search finishes or `/exploration/stop` requests cancellation. The
standard Nav2 `bt_navigator` is disabled by this launch file unless
`use_bt_navigator:=true` is set. To disable the standalone runner, use
`use_exploration_bt_navigator:=false`.

If `use_bt_navigator:=true` is enabled, you can still call
`bt_navigator` directly through the standard `NavigateToPose` action. Any valid
pose in the global frame can be used:

```bash
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose "{
  pose: {
    header: {frame_id: map},
    pose: {
      position: {x: 0.0, y: 0.0, z: 0.0},
      orientation: {w: 1.0}
    }
  }
}"
```

Useful visualization topics:

```text
/frontier_exploration/selected_candidate
/frontier_exploration/candidates
/frontier_exploration/topology_map
/frontier_exploration/ufomap_occupied_cloud
/frontier_exploration/ufomap_free_cloud
```
