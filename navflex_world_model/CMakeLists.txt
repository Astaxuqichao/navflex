cmake_minimum_required(VERSION 3.10)
project(navflex_world_model)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/EvaluatePlan.srv"
  "srv/RankViewpoints.srv"
  DEPENDENCIES builtin_interfaces geometry_msgs
)

# The helper modules live in navflex_wm, not navflex_world_model: rosidl owns
# the latter name in site-packages and would clash over __init__.py.
ament_python_install_package(navflex_wm)

install(PROGRAMS
  src/navflex_world_model_node.py
  DESTINATION lib/${PROJECT_NAME}
)

# The sidecar and demo scripts run outside the ROS node -- the sidecar under
# its own conda env, the demo drivers as plain scripts. Installed for
# discoverability; see docs/world_model_gate_demo.md.
install(PROGRAMS
  scripts/lingbot_server.py
  scripts/gate_demo_seeds.py
  scripts/gate_demo_run.py
  scripts/gate_demo_page.py
  DESTINATION share/${PROJECT_NAME}/scripts
)

install(DIRECTORY launch params
  DESTINATION share/${PROJECT_NAME}
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
