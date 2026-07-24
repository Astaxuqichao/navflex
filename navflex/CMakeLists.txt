cmake_minimum_required(VERSION 3.10)
project(navflex)

find_package(ament_cmake REQUIRED)

install(FILES README.md
  DESTINATION share/${PROJECT_NAME})

install(DIRECTORY docs
  DESTINATION share/${PROJECT_NAME})

ament_package()
