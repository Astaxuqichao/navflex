# navflex_rogmap_planner

ROS 2/Navflex plugin adaptation of the original ROG-Map `rog_astar` example.
The planner implements `navflex_rogmap_core::GlobalPlanner` and consumes only
`navflex_rog_map::RogMap::ConstPtr`.

The bounded three-dimensional A* buffer is indexed at the persistent global
map resolution. State validation uses global raw or inflated occupancy and the
same pose-aware global footprint collision API, including the configured
unknown-space policy. The planner retains 6/26-neighbor expansion,
diagonal/Manhattan/Euclidean heuristics, and a per-request planning time limit.

Plugin type:

```text
navflex_rogmap_planner/RogAStarPlanner
```

See `config/rog_astar.yaml` for a `navflex_nav` configuration example.
