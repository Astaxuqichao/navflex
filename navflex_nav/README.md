# navflex_nav

Unified Navflex navigation package. It contains the shared `navflex_base`
execution framework and both costmap and ROG-Map backends in one library.

```bash
ros2 launch navflex_nav navflex_nav.launch.py navigation_type:=costmap
ros2 launch navflex_nav navflex_nav.launch.py navigation_type:=rogmap
```

`costmap` creates `CostmapNavNode` and loads `nav2_core` plugins. `rogmap`
creates `RogMapNavNode` and loads `navflex_rogmap_core` plugins. Both backends
use the same Action classes, execution state machines, cancellation handling,
retry logic, and robot information layer through internal plugin adapters. In
both modes the top-level lifecycle node is named `navflex_nav`.

Source layout:

```text
include/navflex_base/  shared Action, Execution, and plugin adapters
include/navflex_nav/costmap_nav/  Nav2 costmap backend interfaces
include/navflex_nav/rogmap_nav/   ROG-Map backend interfaces
src/navflex_base/      single shared base implementation
src/costmap_nav/       Nav2 costmap backend
src/rogmap_nav/        ROG-Map backend
```

The previous `navflex_nav` and `navflex_nav` packages are marked
with `COLCON_IGNORE`; their active implementation now lives here.
