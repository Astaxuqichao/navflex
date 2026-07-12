# navflex_rogmap_core

Core contracts for a Nav2-style Navflex 3D navigation stack.

## Architecture

```text
PointCloud / depth / lidar
          |
    3D map server
          |
  shared navflex_rog_map::RogMap
     /        |        \
 planner   controller  recovery
     |        |          |
Trajectory3D TwistStamped Result
```

`navflex_rog_map` owns all map types, sensor input, TF, ray casting, probability
updates, ESDF and sliding-window behavior. This package defines navigation
plugin interfaces only. Every plugin receives the same read-only
`navflex_rog_map::RogMap::ConstPtr`; no map representation is duplicated here.

## Plugin contracts

- `navflex_rogmap_core::GlobalPlanner`: produces a `Trajectory3D` from start and goal.
- `navflex_rogmap_core::Controller`: tracks a trajectory and outputs full 3D velocity.
- `navflex_rogmap_core::Recovery`: executes a named recovery using the same map.

All contracts use the Nav2 lifecycle sequence: `configure`, `activate`,
`deactivate`, `cleanup`. Long-running operations expose cancellation or stop.

Recommended pluginlib base class names:

```xml
base_class_type="navflex_rogmap_core::GlobalPlanner"
base_class_type="navflex_rogmap_core::Controller"
base_class_type="navflex_rogmap_core::Recovery"
```

Map occupancy and ESDF semantics are defined exclusively by `navflex_rog_map`.
