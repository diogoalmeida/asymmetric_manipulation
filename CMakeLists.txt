cmake_minimum_required(VERSION 2.8.3)
project(coordination_experiments)

find_package(
  catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  generic_control_toolbox
)

add_definitions(-std=c++11)
link_directories(${catkin_LIBRARY_DIRS})

catkin_package(
  LIBRARIES ects
  CATKIN_DEPENDS roscpp std_msgs sensor_msgs generic_control_toolbox
  INCLUDE_DIRS include
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(ects src/ects.cpp src/algorithm_base.cpp)
target_link_libraries(ects ${catkin_LIBRARIES})
add_dependencies(ects ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencfg)
