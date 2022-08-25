cmake_minimum_required(VERSION 2.8.3)
project(poli_msgs)

# add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS actionlib_msgs)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
add_service_files(
  FILES
  LedEye.srv
  LedEar.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs actionlib_msgs 
)

catkin_package( 
  CATKIN_DEPENDS actionlib_msgs 
)

include_directories(
# include
# ${catkin_INCLUDE_DIRS}
)
