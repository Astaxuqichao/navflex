# navflex_rogmap_planner

ROS 2/Navflex plugin adaptation of the original ROG-Map `rog_astar` example.
The planner implements `navflex_rogmap_core::GlobalPlanner` and consumes only
`navflex_rog_map::RogMap::ConstPtr`.

Preserved algorithm behavior includes the bounded three-dimensional voxel
buffer, 6/26-neighbor expansion, diagonal/Manhattan/Euclidean heuristics,
inflated or probability-map collision checks, unknown-space policy, and a
per-request planning time limit.

Plugin type:

```text
navflex_rogmap_planner/RogAStarPlanner
```

See `config/rog_astar.yaml` for a `navflex_nav` configuration example.
