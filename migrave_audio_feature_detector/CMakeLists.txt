cmake_minimum_required(VERSION 3.0.2)
project(migrave_audio_feature_detector)

find_package(catkin REQUIRED COMPONENTS
  rospy
  roslint
  audio_common_msgs
  migrave_ros_msgs
)

roslint_python()
catkin_python_setup()
catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)
