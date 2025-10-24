cmake_minimum_required(VERSION 3.5)
project(april_tag_monitor)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME})

# Install resource files
install(
  DIRECTORY resource/
  DESTINATION share/${PROJECT_NAME}
)

# Explicitly install the package.xml as a marker for ament
install(
  FILES package.xml
  DESTINATION share/${PROJECT_NAME}
)

# Install the ament index marker
ament_package()
