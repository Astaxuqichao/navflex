// Copyright 2026 Navflex contributors

#ifndef NAVFLEX_ROGMAP_CORE__TYPES_HPP_
#define NAVFLEX_ROGMAP_CORE__TYPES_HPP_

#include "nav_msgs/msg/path.hpp"

namespace navflex_rogmap_core
{
struct Trajectory3D {nav_msgs::msg::Path path; double duration{0.0}; double max_velocity{0.0};
  double max_acceleration{0.0};};
}  // namespace navflex_rogmap_core
#endif  // NAVFLEX_ROGMAP_CORE__TYPES_HPP_
