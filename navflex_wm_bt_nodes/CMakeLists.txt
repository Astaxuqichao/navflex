cmake_minimum_required(VERSION 3.10)
project(navflex_wm_bt_nodes)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav2_behavior_tree REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(navflex_world_model REQUIRED)   # provides srv/EvaluatePlan C++ types

set(DEPS
  rclcpp
  nav2_behavior_tree
  nav_msgs
  geometry_msgs
  behaviortree_cpp_v3
  navflex_world_model
)

include_directories(include)

# ── BT node shared library ────────────────────────────────────────────────────
# ONE library, ONE BT_REGISTER_NODES (in evaluate_plan_action.cpp) that
# registers BOTH node types. Loaded at runtime via BehaviorTreeEngine /
# bt_navigator's plugin_lib_names — NOT via pluginlib.
#
#   NavflexEvaluatePlanAction  – world-model + critic gate on a candidate viewpoint
#   NavflexPathEndPose         – path endpoint -> PoseStamped (feeds the gate a goal)
#
# Deliberately a SEPARATE package from navflex_bt_nodes so the original
# exploration framework carries no dependency on the world model.
add_library(navflex_wm_bt_nodes SHARED
  src/evaluate_plan_action.cpp
  src/path_end_pose.cpp
)
ament_target_dependencies(navflex_wm_bt_nodes ${DEPS})
target_compile_definitions(navflex_wm_bt_nodes PRIVATE BT_PLUGIN_EXPORT)

# ── Install ───────────────────────────────────────────────────────────────────
install(TARGETS navflex_wm_bt_nodes
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

install(FILES navflex_wm_tree_nodes.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(navflex_wm_bt_nodes)
ament_export_dependencies(${DEPS})

ament_package()
