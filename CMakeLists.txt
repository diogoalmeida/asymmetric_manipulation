cmake_minimum_required(VERSION 2.8.3)
project(coordination_experiments)

find_package(
  catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  generic_control_toolbox
  actionlib_msgs
  message_generation
  baxter_description
  robot_kinematic_simulation
  rviz_visual_tools
  object_server
)

add_definitions(-std=c++11)
link_directories(${catkin_LIBRARY_DIRS})

add_message_files(
  FILES
  Controllers.msg
  Inputs.msg
  WorkspaceLimits.msg
)

add_action_files(
  DIRECTORY action
  FILES CoordinationController.action RunSimulation.action
)

generate_messages(
  DEPENDENCIES
  actionlib_msgs
  geometry_msgs
)

catkin_package(
  LIBRARIES ects
  CATKIN_DEPENDS roscpp std_msgs sensor_msgs generic_control_toolbox actionlib_msgs message_generation baxter_description robot_kinematic_simulation object_server
  INCLUDE_DIRS include
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(ects src/ects.cpp src/algorithm_base.cpp)
target_link_libraries(ects ${catkin_LIBRARIES})
add_dependencies(ects ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencfg)

add_library(ext_rel_jac src/ext_rel_jac.cpp src/algorithm_base.cpp)
target_link_libraries(ext_rel_jac ${catkin_LIBRARIES})
add_dependencies(ext_rel_jac ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencfg)

add_library(rel_jac src/rel_jac.cpp src/algorithm_base.cpp)
target_link_libraries(rel_jac ${catkin_LIBRARIES})
add_dependencies(rel_jac ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencfg)

add_library(rel_jac_abs_lim src/rel_jac_abs_lim.cpp src/algorithm_base.cpp)
target_link_libraries(rel_jac_abs_lim ${catkin_LIBRARIES})
add_dependencies(rel_jac_abs_lim ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencfg)

set(
  CONTROLLERS
  coordination_controller
  ects
  ext_rel_jac
  rel_jac
  rel_jac_abs_lim
)

add_executable(coordination_controller src/coordination_controller.cpp)
target_link_libraries(${CONTROLLERS} ${catkin_LIBRARIES})
add_dependencies(coordination_controller ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencfg)
