cmake_minimum_required(VERSION 2.8.3)
project(poli_pan_tilt)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  dynamixel_sdk
  dynamixel_workbench_msgs
  dynamixel_workbench_toolbox
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES poli_pan_tilt
  CATKIN_DEPENDS roscpp rospy std_msgs dynamixel_workbench_msgs dynamixel_workbench_toolbox
)

include_directories(
 include
  ${catkin_INCLUDE_DIRS}
)

add_executable(poli_pan_position_control src/position_control.cpp)
add_dependencies(poli_pan_position_control ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(poli_pan_position_control ${catkin_LIBRARIES})
