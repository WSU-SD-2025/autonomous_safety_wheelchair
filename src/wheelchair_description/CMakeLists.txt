cmake_minimum_required(VERSION 3.8)
project(wheelchair_description)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(urdf REQUIRED)
find_package(xacro REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()


find_program(XACRO_EXE xacro)
if(XACRO_EXE)
  add_custom_target(check_robot_xacro ALL
    COMMAND ${XACRO_EXE} ${CMAKE_CURRENT_SOURCE_DIR}/wheelchair/urdf/robot.xacro
            > ${CMAKE_CURRENT_BINARY_DIR}/_robot.urdf
    DEPENDS 
      ${CMAKE_CURRENT_SOURCE_DIR}/wheelchair/urdf/robot.xacro
      ${CMAKE_CURRENT_SOURCE_DIR}/wheelchair/urdf/materials.xacro
      ${CMAKE_CURRENT_SOURCE_DIR}/wheelchair/urdf/macros/wheels.xacro
      ${CMAKE_CURRENT_SOURCE_DIR}/wheelchair/urdf/macros/lidar.xacro
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    COMMENT "Checking xacro: robot.xacro -> _robot.urdf"
  )
endif()


install(
  DIRECTORY
    rviz
    launch
    wheelchair
    worlds
  
  DESTINATION
    share/${PROJECT_NAME}/
)

ament_export_dependencies(urdf xacro)
ament_package()
