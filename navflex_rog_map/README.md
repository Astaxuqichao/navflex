# navflex_rog_map

ROS 2/Navflex integration of the HKU MaRS Lab ROG-Map algorithms.

- Global map: sparse probabilistic occupancy grid at a configurable coarse resolution.
- Local map: original sliding circular/hash map centered at the current sensor origin.
- Raycasting: original cached and batched raycasting with probabilistic hit/miss updates.
- Inflation: original `CounterMap`/`InfMap` occupied and unknown-space inflation.
- Frontier: original local frontier classification and extraction.
- ESDF: original dimensional Euclidean distance transform with first and second gradients.
- API: `navflex_rog_map::RogMap` is the single map type consumed by Navflex ROG plugins.
- Footprint: pose-aware `sphere`, `cylinder`, `box`, and front/rear `double_sphere`
  collision checks shared by planners and controllers.
- Server: lifecycle node consuming `sensor_msgs/PointCloud2` and TF.

## Nav2-style loading

`RogMapROS` follows the `nav2_costmap_2d::Costmap2DROS` ownership model. It can
run as a standalone lifecycle node, as an rclcpp component, or as a child node
owned by a Navflex navigation server.

```cpp
auto rog_map = std::make_shared<navflex_rog_map::RogMapROS>(
  "rog_map", get_namespace(), "rog_map");
rog_map->configure();
auto map_thread = std::make_unique<nav2_util::NodeThread>(rog_map);
rog_map->activate();
```

The owner must deactivate and clean up the child map before destroying its
`NodeThread`, matching the existing global/local costmap lifecycle ordering.

Standalone mode:

```bash
ros2 launch navflex_rog_map rog_map.launch.py
```

Composed mode:

```bash
ros2 launch navflex_rog_map rog_map.launch.py use_composition:=true
```

The mapping algorithms are vendored from the ROS 2 ROG-Map implementation and wrapped
with Navflex-owned ROS 2 lifecycle, input, query, and plugin-facing APIs. Upstream ROS
callbacks and visualization nodes are intentionally replaced by `RogMapROS`.

ROG-Map source: https://github.com/hku-mars/ROG-Map
