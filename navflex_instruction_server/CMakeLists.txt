cmake_minimum_required(VERSION 3.10)
project(navflex_instruction_server)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/ExecuteInstruction.srv"
  "srv/ExecuteTask.srv"
  "srv/GetCapabilities.srv"
  "srv/InterpretVln.srv"
  "srv/ListSemanticLandmarks.srv"
  "srv/QuerySemanticTarget.srv"
  "srv/UpdateLandmark.srv"
  DEPENDENCIES builtin_interfaces geometry_msgs nav2_msgs nav_msgs std_msgs
)

install(PROGRAMS
  src/navflex_instruction_server_node.py
  src/navflex_semantic_map_server.py
  src/navflex_task_server.py
  src/navflex_instruction_web.py
  src/navflex_vln_bridge.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch params
  DESTINATION share/${PROJECT_NAME}
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
