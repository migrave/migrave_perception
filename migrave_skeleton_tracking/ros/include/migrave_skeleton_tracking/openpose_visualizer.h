/*
 * Author: Mohammad Wasil
 * OpenPose 3D visualization
 */
#ifndef MIGRAVE_SKELETON_TRACKING_OPENPOSEVISUALIZER_H
#define MIGRAVE_SKELETON_TRACKING_OPENPOSEVISUALIZER_H

#include <string>
#include <ros/ros.h>
#include <migrave_ros_msgs/Skeletons.h>
#include <migrave_ros_msgs/BodyPart.h>
#include <visualization_msgs/Marker.h>

namespace openpose
{
namespace visualization
{
class OpenPoseVisualizer
{
  public:
    OpenPoseVisualizer(const boost::shared_ptr<ros::NodeHandle> &nh,
                       const std::string &topic_name, 
                       bool check_subscribers = true);

    OpenPoseVisualizer(const std::string &topic_name, 
                       bool check_subscribers = true);

    int getNumSubscribers();

    void publish(const migrave_ros_msgs::Skeletons &skeletons);

  private:
    ros::Publisher marker_publisher_;

    bool check_subscribers_;

    bool hasConnection(const migrave_ros_msgs::BodyPart &bodypart1,
                       const migrave_ros_msgs::BodyPart &bodypart2,
                       visualization_msgs::Marker &skeleton);

    bool isValidPoint(const migrave_ros_msgs::BodyPart &bodypart);

};

} // namespace visualization

} // namespace openpose

// #include <impl/openpose_visualizer.hpp>

#endif  // MIGRAVE_SKELETON_TRACKING_OPENPOSEVISUALIZER_H