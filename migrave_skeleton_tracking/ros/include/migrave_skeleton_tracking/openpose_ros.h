/*
 * Copyright 2021, MigrAVE project
 * Hochschule Bonn-Rhein-Sieg
 *
 * Author: Mohammad Wasil
 *
 */
#ifndef MIGRAVE_SKELETON_TRACKING_OPENPOSE_ROS_H
#define MIGRAVE_SKELETON_TRACKING_OPENPOSE_ROS_H

#include <eigen3/Eigen/Dense>
#include <vector>
#include <string>
#include <iostream>
#include <fstream>

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <tf/transform_listener.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <dynamic_reconfigure/server.h>

#include <openpose/flags.hpp>
#include <openpose/headers.hpp>
#include <migrave_ros_msgs/BodyPart.h>
#include <migrave_skeleton_tracking/impl/openpose_visualizer.hpp>

typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloud;

/** \brief MigrAVE ROS skeleton tracking with OpenPose 1.7
 * 
 * \author Mohammad Wasil
**/

using openpose::visualization::OpenPoseVisualizer;

class OpenPoseROS
{
  public:
    /** 
     * \brief Constructor
     * \param[in] NodeHandle 
     * \param[in] OpenPose Wrapper
     */
    OpenPoseROS(ros::NodeHandle* nh, op::Wrapper* op_wrapper);

    /** \brief Destructor */
    virtual ~OpenPoseROS();

    /** 
     * \brief Track skeletons if enough image and cloud msgs are received
     */
    void update();

  private:
    ros::NodeHandle* nh_;
    ros::Subscriber sub_event_in_;
    ros::Publisher pub_event_out_;
    ros::Publisher pub_skeletons_;
    ros::Publisher pub_debug_img_;
    ros::Subscriber sub_depth_;

    // Synchronize callback for image and cloud
    message_filters::Subscriber<sensor_msgs::Image> *image_sub_;
    message_filters::Subscriber<sensor_msgs::PointCloud2> *cloud_sub_;
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, 
                                        sensor_msgs::PointCloud2> msgSyncPolicy;
    message_filters::Synchronizer<msgSyncPolicy> *msg_sync_;
    void synchronizeCallback(const sensor_msgs::ImageConstPtr &image, 
                             const sensor_msgs::PointCloud2ConstPtr &cloud);

    
    std::string point_cloud_topic_;
    std::string rgb_image_topic_;
    std::string pub_skeleton_topic_;
    bool continuous_tracking_;

    // Used to store depth and image received from callback
    sensor_msgs::ImageConstPtr image_msg_;
    sensor_msgs::PointCloud2ConstPtr cloud_msg_;
    PointCloud::Ptr cloud_;

    // Flags for depth and image subscription
    int cloud_msg_received_count_;
    int image_msg_received_count_;
    
    // Parameters
    std::string target_frame_id_;

    // OpenPose
    op::Wrapper* op_wrapper_;

    // Visualization
    OpenPoseVisualizer op_visualizer_;

  private:
    /** 
     * \brief Event in callback
     *
     * \param[in] String message e_start or e_stop
     */
    void eventCallback(const std_msgs::String::ConstPtr &msg);

    /**
     * \brief Get 3D pose of each key point
     * 
     * \param[in] key points
     * \param[in] index of key points
     * \param[in] Point cloud
     * \param[out] Body part 
     */
    template <typename key_points>
    void get3DPose(const key_points& kp, 
                   const int& i,
                   const PointCloud::ConstPtr &cloud,
                   migrave_ros_msgs::BodyPart &part);

    /**
     * \brief Get skeletons
     * 
     * \param[in] RGB image
     * \param[in] Point cloud of the corresponding image 
     */
    void getSkeleton(const sensor_msgs::ImageConstPtr &image, 
                     const PointCloud::ConstPtr &cloud);

};

#endif  // MIGRAVE_SKELETON_TRACKING_OPENPOSE_ROS_H
