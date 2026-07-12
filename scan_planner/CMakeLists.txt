cmake_minimum_required(VERSION 3.8)
project(scan_planner)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav2_util REQUIRED)
find_package(navflex_rog_map REQUIRED)
find_package(navflex_rogmap_core REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

add_library(scan_controller SHARED src/scan_controller.cpp)
target_include_directories(scan_controller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(scan_controller PUBLIC cxx_std_17)
ament_target_dependencies(scan_controller
  geometry_msgs nav2_core nav2_msgs nav2_util navflex_rog_map
  navflex_rogmap_core pluginlib rclcpp rclcpp_lifecycle tf2 tf2_ros)
pluginlib_export_plugin_description_file(navflex_rogmap_core controller_plugin.xml)

install(TARGETS scan_controller EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)
install(FILES controller_plugin.xml DESTINATION share/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_uncrustify REQUIRED)
  ament_uncrustify(LANGUAGE CPP include src)
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_dependencies(navflex_rogmap_core)
ament_package()
