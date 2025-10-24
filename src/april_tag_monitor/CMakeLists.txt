cmake_minimum_required(VERSION 3.5)
project(april_tag_monitor)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
 
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
