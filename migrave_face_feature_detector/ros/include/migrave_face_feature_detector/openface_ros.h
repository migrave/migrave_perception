/*
 * Copyright 2021, MigrAVE project
 * Hochschule Bonn-Rhein-Sieg
 *
 * Author: Mohammad Wasil
 *
 */

#ifndef MIGRAVE_FACE_FEATURE_DETECTOR_OPENFACE_ROS_H
#define MIGRAVE_FACE_FEATURE_DETECTOR_OPENFACE_ROS_H

#include <vector>

#include <opencv2/core/core.hpp>

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>

#include <OpenFace/LandmarkCoreIncludes.h>
#include <OpenFace/FaceAnalyser.h>
#include <OpenFace/Visualizer.h>

#include <tf2_ros/transform_broadcaster.h>

static geometry_msgs::Quaternion toQuaternion(double pitch, double roll, double yaw)
{
  double t0 = std::cos(yaw * 0.5f);
  double t1 = std::sin(yaw * 0.5f);
  double t2 = std::cos(roll * 0.5f);
  double t3 = std::sin(roll * 0.5f);
  double t4 = std::cos(pitch * 0.5f);
  double t5 = std::sin(pitch * 0.5f);

  geometry_msgs::Quaternion q;
  q.w = t0 * t2 * t4 + t1 * t3 * t5;
  q.x = t0 * t3 * t4 - t1 * t2 * t5;
  q.y = t0 * t2 * t5 + t1 * t3 * t4;
  q.z = t1 * t2 * t4 - t0 * t3 * t5;
  return q;
}

static geometry_msgs::Quaternion operator *(const geometry_msgs::Quaternion &a, 
                                            const geometry_msgs::Quaternion &b)
{
  geometry_msgs::Quaternion q;
  
  q.w = a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z;  // 1
  q.x = a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y;  // i
  q.y = a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x;  // j
  q.z = a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w;  // k
  return q;
}

/**
 * \brief OpenFace ros wrapper. This wrapper requires OpenFace 2.2.0 version.
 * This node publishes action units, head pose, landmarks, gaze, detection_confidence 
 * of the detected faces. The message is defined in migrave_ros_msgs::Face
 * 
 * \author Mohammad Wasil
 */

class OpenFaceROS
{
  public:
    OpenFaceROS(ros::NodeHandle &nh);

    virtual ~OpenFaceROS();

    void update();

  private:
    ros::NodeHandle nh_;
    ros::Subscriber sub_event_in_;
    ros::Publisher pub_event_out_;

    ros::Subscriber image_sub_;
    ros::Subscriber cam_info_sub_;
    ros::Publisher pub_debug_img_;
    ros::Publisher pub_faces_;

    int max_faces_;
    std::string camera_name_;
    std::string rgb_image_topic_;
    std::string cam_info_topic_;
    bool image_msg_received_;
    bool camera_info_received_;
    bool continuous_tracking_;
    sensor_msgs::ImageConstPtr image_msg_;

    tf2_ros::TransformBroadcaster tf2_br_;

    // OpenFace
    typedef std::shared_ptr<FaceAnalysis::FaceAnalyser> FaceAnalyserPtr;
    FaceAnalyserPtr face_analyser_;
    std::vector<LandmarkDetector::CLNF> face_models_;
    std::vector<bool> active_models_;
    std::vector<LandmarkDetector::FaceModelParameters> det_parameters_;
    typedef std::shared_ptr<Utilities::Visualizer> OFVisualizerPtr;
    OFVisualizerPtr of_visualizer_;
    Utilities::FpsTracker of_fps_tracker_;

    // Default camera parameters
    double fx_ = 0.0;
    double fy_ = 0.0;
    double cx_ = 0.0;
    double cy_ = 0.0;

    /** \brief Event in callback
     */
    void eventCallback(const std_msgs::String::ConstPtr &msg);

    void camInfoCallback(const sensor_msgs::CameraInfo::ConstPtr &msg);

    void imageCallback(const sensor_msgs::ImageConstPtr &image);

    void initializeOpenFace();

    double getIOU(cv::Rect_<float> rect1, cv::Rect_<float> rect2);

    void nonOverlapingDetections(const std::vector<LandmarkDetector::CLNF>& clnf_models, 
                                 std::vector<cv::Rect_<float> >& face_detections);

    void removeOverlapingModels(std::vector<LandmarkDetector::CLNF>& face_models, 
                                std::vector<bool>& active_models);

    void trackFaces();

};

#endif  // MIGRAVE_FACE_FEATURE_DETECTOR_OPENFACE_ROS_H
