cmake_minimum_required(VERSION 3.0.2)
project(migrave_skeleton_tracking)

# PCL 1.10 requires c++14
add_definitions(-std=c++14 -g)

find_package(catkin REQUIRED COMPONENTS
  rospy
  roslint
  cv_bridge
  std_msgs
  sensor_msgs
  message_filters
  migrave_ros_msgs
  migrave_common
)

## Make sure 'FindGFlags.cmake' and 'FindGlog.cmake' are visible to cmake
## Based on https://github.com/ravijo/ros_openpose
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake/Modules")

## Add OpenPose and other dependencies
find_package(GFlags)
find_package(Glog)
find_package(OpenCV REQUIRED)
find_package(OpenPose REQUIRED)
find_package(Eigen3 REQUIRED)

# ROS noetic comes with pcl 1.10
find_package(PCL 1.10 REQUIRED)

## Pass in the version of OpenPose
add_definitions( -DOpenPose_VERSION_MAJOR=${OpenPose_VERSION_MAJOR} )
add_definitions( -DOpenPose_VERSION_MINOR=${OpenPose_VERSION_MINOR} )
add_definitions( -DOpenPose_VERSION_PATCH=${OpenPose_VERSION_PATCH} )

## Print OpenPose version in the terminal
MESSAGE(STATUS "OpenPose VERSION: " ${OpenPose_VERSION})

catkin_package(
  INCLUDE_DIRS 
    ros/include
  LIBRARIES
    ${PROJECT_NAME}
  CATKIN_DEPENDS
    visualization_msgs
)

include_directories(
  ros/include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
  ${OpenPose_INCLUDE_DIRS}
  ${GFLAGS_INCLUDE_DIR}
  ${GLOG_INCLUDE_DIR}
)

### LIRBARIES
## Path to Caffe lib i.e., libcaffe.so
set(CAFFE_LIB_FOLDER /opt/openpose/caffe/lib)
link_directories(
  ${catkin_LIBRARY_DIRS}
  ${CAFFE_LIB_FOLDER}
)

### Executables 
add_executable(openpose_ros
  ros/src/openpose_ros.cpp
)

add_dependencies(openpose_ros
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(openpose_ros
  ${catkin_LIBRARIES}
  ${OpenPose_LIBS}
  ${GFLAGS_LIBRARY}
  ${GLOG_LIBRARY}
  ${OpenCV_LIBS}
  ${PCL_LIBRARIES}
)
