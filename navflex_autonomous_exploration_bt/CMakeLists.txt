cmake_minimum_required(VERSION 3.10)
project(navflex_autonomous_exploration_bt)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_behavior_tree REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

set(DEPS
  ament_index_cpp
  behaviortree_cpp_v3
  geometry_msgs
  nav2_behavior_tree
  rclcpp
  std_msgs
)

add_executable(exploration_bt_navigator
  src/exploration_bt_navigator.cpp)
ament_target_dependencies(exploration_bt_navigator ${DEPS})

install(TARGETS exploration_bt_navigator
  RUNTIME DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY behavior_trees
  DESTINATION share/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

install(DIRECTORY params
  DESTINATION share/${PROJECT_NAME})


install(FILES README.md
  DESTINATION share/${PROJECT_NAME})

ament_package()
