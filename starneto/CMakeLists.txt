cmake_minimum_required(VERSION 2.8.3)
project(starneto_mems)

add_compile_options(-std=c++11)

set(PROJECT_DEPS
  roscpp
  std_msgs
  starneto_ros_msgs
  serial
)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  starneto_ros_msgs
  serial
)

catkin_package(
  INCLUDE_DIRS
  LIBRARIES
  CATKIN_DEPENDS
  DEPENDS
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${roscpp_INCLUDE_DIRS}
)

# Each node in the package must be declared like this
add_executable(${PROJECT_NAME}
  src/starneto_mems.cpp
  src/main.cpp
  )
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  )
