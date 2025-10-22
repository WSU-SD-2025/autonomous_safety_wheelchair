cmake_minimum_required(VERSION 3.16 FATAL_ERROR)

project(actor_movement_plugin)

find_package(gz-sim7 REQUIRED)

add_library(ActorMovementPlugin SHARED
  src/ActorMovementPlugin.cc
)

target_include_directories(ActorMovementPlugin PUBLIC
  ${GZ_SIM_INCLUDE_DIRS}
)

target_link_libraries(ActorMovementPlugin PUBLIC
  ${GZ_SIM_LIBRARIES}
)