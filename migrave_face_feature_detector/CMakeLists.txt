cmake_minimum_required(VERSION 3.0.2)
project(migrave_face_feature_detector)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  rospy
  roscpp
  roslint
  sensor_msgs
  geometry_msgs
  migrave_ros_msgs
  migrave_common
  cv_bridge
  tf2
  tf2_ros
)

find_package(PCL 1.7 REQUIRED)
find_package(Boost 1.71.0 REQUIRED COMPONENTS filesystem system)
find_package(OpenCV REQUIRED)

find_library(OPENFACE_LD_LIB LandmarkDetector)
find_library(OPENFACE_FA_LIB FaceAnalyser)
find_library(OPENFACE_GA_LIB GazeAnalyser)
find_library(OPENFACE_UT_LIB Utilities)

set(OPENFACE_LIBS ${OPENFACE_LD_LIB} ${OPENFACE_FA_LIB} ${OPENFACE_GA_LIB} ${OPENFACE_UT_LIB})
find_path(OPENFACE_INCLUDE_DIR LandmarkDetectorModel.h PATH_SUFFIXES OpenFace)

catkin_package(
  INCLUDE_DIRS
    ros/include
  LIBRARIES
    migrave_face_feature_detector
    ${PROJECT_NAME}  
  CATKIN_DEPENDS
    migrave_ros_msgs
)

include_directories(
  ros/include
  ${catkin_INCLUDE_DIRS}
  ${OPENFACE_INCLUDE_DIR}
  ${OpenBLAS_INCLUDE_DIR}
  ${Boost_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

roslint_python()
catkin_python_setup()
roslint_cpp()

add_definitions(-fpermissive)

### EXECUTABLES
add_executable(openface_ros
  ros/src/openface_ros.cpp
)
add_dependencies(openface_ros
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS} 
)
target_link_libraries(openface_ros
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${OPENFACE_LIBS}
  openblas
  dlib
  ### add the following to solve this error
  ### undefined reference to symbol '_ZNK5boost10filesystem4path4stemEv'
  -lboost_filesystem
  -lboost_system
  ###
)

