cmake_minimum_required(VERSION 3.8)
project(wheelchair_mux)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# find_package(rclcpp REQUIRED)
# find_package(geometry_msgs REQUIRED)
# find_package(std_msgs REQUIRED)
# find_package(tf2 REQUIRED)
# find_package(tf2_geometry_msgs REQUIRED)



# include_directories(include)



# add_executable(guardian_mode_node src/guardian_mode.cpp)
# ament_target_dependencies(guardian_mode_node rclcpp geometry_msgs std_msgs tf2 tf2_geometry_msgs)



# install(TARGETS
#   guardian_mode_node

#   DESTINATION lib/${PROJECT_NAME}
# )


# install(DIRECTORY include/
#   DESTINATION include
# )


install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config)


install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)



ament_package()
