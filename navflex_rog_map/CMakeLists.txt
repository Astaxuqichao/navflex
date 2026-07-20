cmake_minimum_required(VERSION 3.8)
project(navflex_rog_map)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED QUIET COMPONENTS common io)
find_package(nav2_util REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(yaml-cpp REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/footprint.cpp
  src/rog_map.cpp
  src/rog_map_ros.cpp
  third_party/rog_map/include/rog_map/rog_map_core/raycaster.cpp
  third_party/rog_map/src/rog_map/counter_map.cpp
  third_party/rog_map/src/rog_map/esdf_map.cpp
  third_party/rog_map/src/rog_map/inf_map.cpp
  third_party/rog_map/src/rog_map/prob_map.cpp
  third_party/rog_map/src/rog_map/rog_map.cpp
  third_party/rog_map/src/rog_map/sliding_map.cpp)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/third_party/rog_map/include>
  $<INSTALL_INTERFACE:include>)
target_compile_definitions(${PROJECT_NAME} PRIVATE
  ORIGIN_AT_CORNER
  USE_ROS2
  ROOT_DIR="${CMAKE_CURRENT_SOURCE_DIR}/")
ament_target_dependencies(${PROJECT_NAME}
  geometry_msgs nav2_util pcl_conversions rclcpp rclcpp_components
  rclcpp_lifecycle sensor_msgs tf2 tf2_ros)
target_link_libraries(${PROJECT_NAME} Eigen3::Eigen ${PCL_LIBRARIES} yaml-cpp)
rclcpp_components_register_nodes(${PROJECT_NAME} "navflex_rog_map::RogMapROS")

add_executable(rog_map_server src/rog_map_server.cpp)
target_compile_features(rog_map_server PUBLIC cxx_std_17)
ament_target_dependencies(rog_map_server
  geometry_msgs nav2_util rclcpp rclcpp_lifecycle sensor_msgs tf2_ros)
target_link_libraries(rog_map_server ${PROJECT_NAME})

install(TARGETS ${PROJECT_NAME} EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
install(TARGETS rog_map_server RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY third_party/rog_map/include/ DESTINATION include)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_cpplint REQUIRED)
  find_package(ament_cmake_uncrustify REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(AMENT_LINT_AUTO_EXCLUDE ament_cmake_cpplint ament_cmake_uncrustify)
  ament_lint_auto_find_test_dependencies()
  ament_cpplint(include src)
  ament_uncrustify(include src)
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_dependencies(geometry_msgs nav2_util pcl_conversions rclcpp_components sensor_msgs tf2)
ament_package()
