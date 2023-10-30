cmake_minimum_required(VERSION 3.10.2)
project(vision_pose)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  # message_generation
  geometry_msgs
)

#add_message_files(
#  FILES
#)

#generate_messages(
#  DEPENDENCIES
#  std_msgs
#  geometry_msgs
#)

catkin_package(
  INCLUDE_DIRS 
# include
# LIBRARIES vision_pose
  CATKIN_DEPENDS 
  # message_generation 
  message_runtime roscpp rospy std_msgs
# DEPENDS system_lib
)

include_directories(
  # include
  ${catkin_INCLUDE_DIRS}
)

add_executable(vision_pose src/vision_pose.cpp)
add_dependencies(vision_pose ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(vision_pose ${catkin_LIBRARIES})

