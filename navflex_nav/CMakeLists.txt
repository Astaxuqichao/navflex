cmake_minimum_required(VERSION 3.10)
project(navflex_nav)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(angles REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_2d_msgs REQUIRED)
find_package(nav_2d_utils REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(navflex_rog_map REQUIRED)
find_package(navflex_rogmap_core REQUIRED)
find_package(navflex_utility REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(tf2_ros REQUIRED)

set(NAVFLEX_NAV_SOURCES
  src/costmap_nav/behavior_action_costmap_server.cpp
  src/costmap_nav/controller_action_costmap_server.cpp
  src/costmap_nav/navflex_costmap_nav.cpp
  src/costmap_nav/planner_action_costmap_server.cpp
  src/rogmap_nav/behavior_action_rogmap_server.cpp
  src/rogmap_nav/controller_action_rogmap_server.cpp
  src/rogmap_nav/navflex_rogmap_nav.cpp
  src/rogmap_nav/planner_action_rogmap_server.cpp
  src/navflex_base/behavior_action.cpp
  src/navflex_base/behavior_execution.cpp
  src/navflex_base/controller_action.cpp
  src/navflex_base/controller_execution.cpp
  src/navflex_base/navflex_execution_base.cpp
  src/navflex_base/planner_action.cpp
  src/navflex_base/planner_execution.cpp)

set(NAVFLEX_NAV_DEPS
  angles geometry_msgs nav_2d_msgs nav_2d_utils nav2_core nav2_costmap_2d
  nav2_msgs nav2_util nav_msgs navflex_rog_map navflex_rogmap_core
  navflex_utility pluginlib rclcpp rclcpp_action rclcpp_components
  rclcpp_lifecycle tf2_ros)

add_library(${PROJECT_NAME} SHARED ${NAVFLEX_NAV_SOURCES})
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(${PROJECT_NAME} ${NAVFLEX_NAV_DEPS})
rclcpp_components_register_nodes(${PROJECT_NAME}
  "navflex_nav::CostmapNavNode"
  "navflex_nav::RogMapNavNode")

add_executable(navflex_nav_node src/main.cpp)
target_compile_features(navflex_nav_node PUBLIC cxx_std_17)
ament_target_dependencies(navflex_nav_node ${NAVFLEX_NAV_DEPS})
target_link_libraries(navflex_nav_node ${PROJECT_NAME})

install(TARGETS ${PROJECT_NAME} navflex_nav_node EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_uncrustify REQUIRED)
  ament_uncrustify(LANGUAGE CPP include src)
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_dependencies(${NAVFLEX_NAV_DEPS})
ament_package()
