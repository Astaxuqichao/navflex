cmake_minimum_required(VERSION 3.5)
project(navflex_frontier_planner)

find_package(ament_cmake REQUIRED)
find_package(nav2_common REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav2_util REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(LZ4 REQUIRED liblz4)
find_package(TBB REQUIRED)

nav2_package()

include_directories(include)

set(library_name navflex_frontier_planner)
set(ufomap_library_name navflex_frontier_ufomap)
set(dependencies
  geometry_msgs
  nav2_core
  nav2_costmap_2d
  nav2_msgs
  nav2_util
  pluginlib
  rclcpp
  rclcpp_lifecycle
  sensor_msgs
  tf2
  tf2_ros
  visualization_msgs
)

add_library(${ufomap_library_name} STATIC
  third_party/ufomap/src/geometry/bounding_volume.cpp
  third_party/ufomap/src/geometry/collision_checks.cpp
  third_party/ufomap/src/map/occupancy_map.cpp
  third_party/ufomap/src/map/occupancy_map_color.cpp
)

target_include_directories(${ufomap_library_name}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/third_party/ufomap/include>
    $<INSTALL_INTERFACE:include/${PROJECT_NAME}/third_party/ufomap>
)

target_compile_features(${ufomap_library_name} PUBLIC cxx_std_17)
target_compile_options(${ufomap_library_name} PUBLIC -fPIC)
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  target_compile_options(${ufomap_library_name} PRIVATE -Wno-error=unused-parameter -Wno-unused-parameter)
endif()
target_link_libraries(${ufomap_library_name}
  PUBLIC
    ${LZ4_LIBRARIES}
    TBB::tbb
)

add_library(${library_name} SHARED
  src/frontier_tools.cpp
  src/candidate_frontier_planner.cpp
  src/frontier_astar_planner.cpp
)

ament_target_dependencies(${library_name}
  ${dependencies}
)

target_link_libraries(${library_name}
  ${ufomap_library_name}
)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  target_compile_options(${library_name} PRIVATE
    -Wno-error=deprecated-copy
    -Wno-error=unused-parameter
    -Wno-deprecated-copy
    -Wno-unused-parameter)
endif()

pluginlib_export_plugin_description_file(nav2_core global_planner_plugin.xml)

install(TARGETS ${library_name} ${ufomap_library_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include/
)

install(DIRECTORY third_party/ufomap/include/
  DESTINATION include/${PROJECT_NAME}/third_party/ufomap
)

install(FILES global_planner_plugin.xml
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_libraries(${library_name} ${ufomap_library_name})
ament_export_dependencies(${dependencies})
ament_package()
