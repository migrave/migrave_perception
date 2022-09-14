cmake_minimum_required(VERSION 3.0.2)
project(migrave_action_recognition)

find_package(catkin REQUIRED COMPONENTS
  rospy
  roslint
  actionlib
  actionlib_msgs
  genmsg
  message_generation
  geometry_msgs
)

roslint_python()
catkin_python_setup()

add_action_files(
  DIRECTORY ros/action
  FILES
  LearnAction.action
  ContinualActionLearning.action
)

generate_messages(
  DEPENDENCIES
  actionlib_msgs
  geometry_msgs
)

catkin_package(
 CATKIN_DEPENDS
   rospy
   actionlib
   actionlib_msgs
   message_runtime
   geometry_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

install(PROGRAMS
  ros/scripts/recognize_action_demo
  ros/scripts/recognize_action_client
  ros/scripts/save_skeleton_data
  ros/scripts/learn_action_demo
  ros/scripts/learn_action_client
  ros/scripts/continual_action_server
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}/scripts
)
