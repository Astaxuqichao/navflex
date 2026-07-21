cmake_minimum_required(VERSION 3.10)
project(navflex_object_nav)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/DetectObject.srv"
  "srv/ApproachTarget.srv"
)

# The helper modules live in navflex_obj, NOT navflex_object_nav: rosidl owns the
# package name in site-packages and would clash over __init__.py (same reason
# navflex_world_model puts its helpers in navflex_wm).
ament_python_install_package(navflex_obj)

install(PROGRAMS
  src/object_grounder_node.py
  src/object_approach_node.py
  src/object_goal_navigator_node.py
  src/object_target_cli.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
