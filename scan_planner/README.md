# scan_planner

Three-dimensional SCAN local controller implemented as a
`navflex_rogmap_core::Controller` plugin.

The controller consumes `RogMap::ConstPtr` directly. It does not subscribe to
point clouds or maintain a duplicate voxel map. Local trajectory validation
uses ROG-Map inflated occupancy, continuous 3D footprint checks, and ESDF
distance queries. Supported footprint types are `sphere`, `cylinder`, `box`,
and `double_sphere`. The double-sphere model uses independent front and rear
sphere offsets and radii in the robot frame. Velocity output retains SCAN feed-forward tracking,
position/yaw feedback, vertical velocity limits, acceleration limits, dynamic
trajectory rebuilding, cancellation, and speed-limit support.

Plugin type:

```text
scan_planner/ScanController
```

See `config/scan_controller.yaml` for the `navflex_nav` ROG-Map mode parameters.
