#ifndef OPENPOSEVISUALIZER_HPP
#define OPENPOSEVISUALIZER_HPP

#include <ros/ros.h>
#include <migrave_ros_msgs/Skeletons.h>
#include <migrave_ros_msgs/BodyPart.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <migrave_skeleton_tracking/color.h>
#include <migrave_skeleton_tracking/openpose_visualizer.h> 
#include <geometry_msgs/Point.h>

namespace openpose
{
namespace visualization
{
OpenPoseVisualizer::OpenPoseVisualizer(
    const boost::shared_ptr<ros::NodeHandle> &nh, 
    const std::string &topic_name,
    bool check_subscribers)
    : check_subscribers_(check_subscribers)
{
  marker_publisher_ = nh->advertise<visualization_msgs::MarkerArray>(topic_name, 1);
}

OpenPoseVisualizer::OpenPoseVisualizer(
    const std::string &topic_name,
    bool check_subscribers)
    : check_subscribers_(check_subscribers)
{
  ros::NodeHandle nh("~");
  marker_publisher_ = nh.advertise<visualization_msgs::MarkerArray>(topic_name, 1);
}

void OpenPoseVisualizer::publish(const migrave_ros_msgs::Skeletons &skeletons)
{
  visualization_msgs::MarkerArray markers;

  for (auto p=0; p<skeletons.skeletons.size(); p++)
  { 
    // Joint marker
    visualization_msgs::Marker jmarker;
    jmarker.header.frame_id = skeletons.header.frame_id;
    jmarker.id = p;
    jmarker.ns = "joints";
    jmarker.header.stamp = ros::Time();
    jmarker.type = visualization_msgs::Marker::SPHERE_LIST;
    jmarker.action = visualization_msgs::Marker::ADD;
    jmarker.scale.x = 0.05;
    jmarker.scale.y = 0.05;
    jmarker.scale.z = 0.05;
    jmarker.color = Color(Color::IVORY);
    jmarker.lifetime = ros::Duration(1);

    // Skeleton marker with line
    visualization_msgs::Marker smarker;
    smarker.id  = p;
    smarker.header.frame_id = skeletons.header.frame_id;
    smarker.ns = "skeleton";
    smarker.header.stamp = ros::Time();
    smarker.type = visualization_msgs::Marker::LINE_LIST;
    smarker.action = visualization_msgs::Marker::ADD;
    smarker.scale.x = 0.02;
    smarker.scale.y = 0.02;
    smarker.scale.z = 0.02;
    smarker.color = Color(Color::SEA_GREEN);
    smarker.lifetime = ros::Duration(1);

    // Add markers for every skeleton (person)
    // body parts consisting of 25 poses (BODY_25)
    for (auto i=0; i<skeletons.skeletons[p].body_parts.size(); i++)
    {
      const migrave_ros_msgs::BodyPart &curr_pose = skeletons.skeletons[p].body_parts[i];
      if (i == 0)
      {
        if (OpenPoseVisualizer::isValidPoint(curr_pose))
        {
          jmarker.points.push_back(curr_pose.point);
          // pose0 connection: pose15,pose16
          OpenPoseVisualizer::hasConnection(curr_pose, 
                                            skeletons.skeletons[p].body_parts[1],
                                            smarker);
          OpenPoseVisualizer::hasConnection(curr_pose, 
                                            skeletons.skeletons[p].body_parts[15],
                                            smarker);
          OpenPoseVisualizer::hasConnection(curr_pose, 
                                            skeletons.skeletons[p].body_parts[16],
                                            smarker);
        }
      }
      else if (i == 1)
      {
        if (OpenPoseVisualizer::isValidPoint(curr_pose))
        {
          jmarker.points.push_back(curr_pose.point);
          // pose 1 conn: pose0,pose2,pose5,pose8
          OpenPoseVisualizer::hasConnection(curr_pose, 
                                            skeletons.skeletons[p].body_parts[0],
                                            smarker);
          OpenPoseVisualizer::hasConnection(curr_pose, 
                                            skeletons.skeletons[p].body_parts[2],
                                            smarker); 
          OpenPoseVisualizer::hasConnection(curr_pose, 
                                            skeletons.skeletons[p].body_parts[5],
                                            smarker); 
          OpenPoseVisualizer::hasConnection(curr_pose, 
                                            skeletons.skeletons[p].body_parts[8],
                                            smarker); 
        }
      }
      else if (i == 2)
      {
        if (OpenPoseVisualizer::isValidPoint(curr_pose))
        {
          jmarker.points.push_back(curr_pose.point);
          // pose 2 conn: pose1,pose3.
          // skip pose 2 to 1 connection as it has been connected before???
          OpenPoseVisualizer::hasConnection(curr_pose, 
                                            skeletons.skeletons[p].body_parts[1],
                                            smarker);
          OpenPoseVisualizer::hasConnection(curr_pose, 
                                            skeletons.skeletons[p].body_parts[3],
                                            smarker);
        }
      }
      else if (i == 3)
      {
        if (OpenPoseVisualizer::isValidPoint(curr_pose))
        {
          jmarker.points.push_back(curr_pose.point);
          // pose 3 conn: pose2,pose4.
          OpenPoseVisualizer::hasConnection(curr_pose, 
                                            skeletons.skeletons[p].body_parts[2],
                                            smarker);
          OpenPoseVisualizer::hasConnection(curr_pose, 
                                            skeletons.skeletons[p].body_parts[4],
                                            smarker);
        }
      }
      else if (i == 4)
      {
        if (OpenPoseVisualizer::isValidPoint(curr_pose))
        {
          jmarker.points.push_back(curr_pose.point);
          OpenPoseVisualizer::hasConnection(curr_pose, 
                                            skeletons.skeletons[p].body_parts[3],
                                            smarker);
        }
      }
      else if (i == 5)
      {
        if (OpenPoseVisualizer::isValidPoint(curr_pose))
        {
          jmarker.points.push_back(curr_pose.point);
          OpenPoseVisualizer::hasConnection(curr_pose, 
                                            skeletons.skeletons[p].body_parts[1],
                                            smarker);
          OpenPoseVisualizer::hasConnection(curr_pose, 
                                            skeletons.skeletons[p].body_parts[6],
                                            smarker);
        }
      }
      else if (i == 6)
      {
        if (OpenPoseVisualizer::isValidPoint(curr_pose))
        {
          jmarker.points.push_back(curr_pose.point);
          OpenPoseVisualizer::hasConnection(curr_pose, 
                                            skeletons.skeletons[p].body_parts[5],
                                            smarker);
          OpenPoseVisualizer::hasConnection(curr_pose, 
                                            skeletons.skeletons[p].body_parts[7],
                                            smarker);
        }
      }
      else if (i == 7)
      {
        if (OpenPoseVisualizer::isValidPoint(curr_pose))
        {
          jmarker.points.push_back(curr_pose.point);
          OpenPoseVisualizer::hasConnection(curr_pose, 
                                            skeletons.skeletons[p].body_parts[6],
                                            smarker);
        }
      }
      else if (i == 8)
      {
        if (OpenPoseVisualizer::isValidPoint(curr_pose))
        {
          jmarker.points.push_back(curr_pose.point);
          OpenPoseVisualizer::hasConnection(curr_pose, 
                                            skeletons.skeletons[p].body_parts[9],
                                            smarker);
          OpenPoseVisualizer::hasConnection(curr_pose, 
                                            skeletons.skeletons[p].body_parts[12],
                                            smarker);
        }
      }
      else if (i == 9)
      {
        if (OpenPoseVisualizer::isValidPoint(curr_pose))
        {
          jmarker.points.push_back(curr_pose.point);
          OpenPoseVisualizer::hasConnection(curr_pose, 
                                            skeletons.skeletons[p].body_parts[8],
                                            smarker);
          OpenPoseVisualizer::hasConnection(curr_pose, 
                                            skeletons.skeletons[p].body_parts[10],
                                            smarker);
        }
      }
      else if (i == 10)
      {
        if (OpenPoseVisualizer::isValidPoint(curr_pose))
        {
          jmarker.points.push_back(curr_pose.point);
          OpenPoseVisualizer::hasConnection(curr_pose, 
                                            skeletons.skeletons[p].body_parts[9],
                                            smarker);
          OpenPoseVisualizer::hasConnection(curr_pose, 
                                            skeletons.skeletons[p].body_parts[11],
                                            smarker);
        }
      }
      else if (i == 11)
      {
        if (OpenPoseVisualizer::isValidPoint(curr_pose))
        {
          jmarker.points.push_back(curr_pose.point);
          OpenPoseVisualizer::hasConnection(curr_pose, 
                                            skeletons.skeletons[p].body_parts[22],
                                            smarker);
          OpenPoseVisualizer::hasConnection(curr_pose, 
                                            skeletons.skeletons[p].body_parts[24],
                                            smarker);                                            
        }
      }
      else if (i == 12)
      {
        if (OpenPoseVisualizer::isValidPoint(curr_pose))
        {
          jmarker.points.push_back(curr_pose.point);
          OpenPoseVisualizer::hasConnection(curr_pose, 
                                            skeletons.skeletons[p].body_parts[8],
                                            smarker);
          OpenPoseVisualizer::hasConnection(curr_pose, 
                                            skeletons.skeletons[p].body_parts[23],
                                            smarker);
        }
      }
      else if (i == 13)
      {
        if (OpenPoseVisualizer::isValidPoint(curr_pose))
        {
          jmarker.points.push_back(curr_pose.point);
          OpenPoseVisualizer::hasConnection(curr_pose, 
                                            skeletons.skeletons[p].body_parts[12],
                                            smarker);
          OpenPoseVisualizer::hasConnection(curr_pose, 
                                            skeletons.skeletons[p].body_parts[14],
                                            smarker);
        }
      }
      else if (i == 14)
      {
        if (OpenPoseVisualizer::isValidPoint(curr_pose))
        {
          jmarker.points.push_back(curr_pose.point);
          OpenPoseVisualizer::hasConnection(curr_pose, 
                                            skeletons.skeletons[p].body_parts[19],
                                            smarker);
          OpenPoseVisualizer::hasConnection(curr_pose, 
                                            skeletons.skeletons[p].body_parts[21],
                                            smarker);
        }
      }
      else if (i == 15)
      {
        if (OpenPoseVisualizer::isValidPoint(curr_pose))
        {
          jmarker.points.push_back(curr_pose.point);
          OpenPoseVisualizer::hasConnection(curr_pose, 
                                            skeletons.skeletons[p].body_parts[0],
                                            smarker);
          OpenPoseVisualizer::hasConnection(curr_pose, 
                                            skeletons.skeletons[p].body_parts[17],
                                            smarker);
        }
      }
      else if (i == 16)
      {
        if (OpenPoseVisualizer::isValidPoint(curr_pose))
        {
          jmarker.points.push_back(curr_pose.point);
          OpenPoseVisualizer::hasConnection(curr_pose, 
                                            skeletons.skeletons[p].body_parts[0],
                                            smarker);
          OpenPoseVisualizer::hasConnection(curr_pose, 
                                            skeletons.skeletons[p].body_parts[18],
                                            smarker);
        }
      }
      else if (i == 17)
      {
        if (OpenPoseVisualizer::isValidPoint(curr_pose))
        {
          jmarker.points.push_back(curr_pose.point);
          OpenPoseVisualizer::hasConnection(curr_pose, 
                                            skeletons.skeletons[p].body_parts[15],
                                            smarker);
        }
      }
      else if (i == 18)
      {
        if (OpenPoseVisualizer::isValidPoint(curr_pose))
        {
          jmarker.points.push_back(curr_pose.point);
          OpenPoseVisualizer::hasConnection(curr_pose, 
                                            skeletons.skeletons[p].body_parts[16],
                                            smarker);
        }
      }
      else if (i == 19)
      {
        if (OpenPoseVisualizer::isValidPoint(curr_pose))
        {
          jmarker.points.push_back(curr_pose.point);
          OpenPoseVisualizer::hasConnection(curr_pose, 
                                            skeletons.skeletons[p].body_parts[20],
                                            smarker);
        }
      }
      else if (i == 20)
      {
        if (OpenPoseVisualizer::isValidPoint(curr_pose))
        {
          jmarker.points.push_back(curr_pose.point);
          OpenPoseVisualizer::hasConnection(curr_pose, 
                                            skeletons.skeletons[p].body_parts[19],
                                            smarker);
        }
      }
      else if (i == 21)
      {
        if (OpenPoseVisualizer::isValidPoint(curr_pose))
        {
          jmarker.points.push_back(curr_pose.point);
          OpenPoseVisualizer::hasConnection(curr_pose, 
                                            skeletons.skeletons[p].body_parts[14],
                                            smarker);
        }
      }
      else if (i == 22)
      {
        if (OpenPoseVisualizer::isValidPoint(curr_pose))
        {
          jmarker.points.push_back(curr_pose.point);
          OpenPoseVisualizer::hasConnection(curr_pose, 
                                            skeletons.skeletons[p].body_parts[11],
                                            smarker);
          OpenPoseVisualizer::hasConnection(curr_pose, 
                                            skeletons.skeletons[p].body_parts[23],
                                            smarker);
        }
      }
      else if (i == 23)
      {
        if (OpenPoseVisualizer::isValidPoint(curr_pose))
        {
          jmarker.points.push_back(curr_pose.point);
          OpenPoseVisualizer::hasConnection(curr_pose, 
                                            skeletons.skeletons[p].body_parts[22],
                                            smarker);
        }
      }
      else if (i == 24)
      {
        if (OpenPoseVisualizer::isValidPoint(curr_pose))
        {
          jmarker.points.push_back(curr_pose.point);
          OpenPoseVisualizer::hasConnection(curr_pose, 
                                            skeletons.skeletons[p].body_parts[11],
                                            smarker);
        }
      }
    }
    // hand parts
    // for (int bp=0; bp<skeletons.skeletons[p].body_parts.size(); bp++)
    // {
    //   if (isValidPoint(skeletons.skeletons[p].body_parts[bp]))
    //     smarker.points.push_back(skeletons.skeletons[p].body_parts[bp].point);
    // }
    markers.markers.push_back(jmarker);
    markers.markers.push_back(smarker);
  }
  marker_publisher_.publish(markers);
}

bool OpenPoseVisualizer::hasConnection(const migrave_ros_msgs::BodyPart &bodypart1,
                                       const migrave_ros_msgs::BodyPart &bodypart2,
                                       visualization_msgs::Marker &skeleton)
{
  if (OpenPoseVisualizer::isValidPoint(bodypart2))
  {
    skeleton.points.push_back(bodypart1.point);
    skeleton.points.push_back(bodypart2.point);
  }
}

bool OpenPoseVisualizer::isValidPoint(const migrave_ros_msgs::BodyPart &bodypart)
{
  if (!std::isnan(bodypart.point.x)  &&
      !std::isnan(bodypart.point.y)  && 
      !std::isnan(bodypart.point.z)  &&
      bodypart.point.z > 0.0 &&
      !std::isnan(bodypart.keypoint_score)
     ) 
  {
    return (true);
  }
  else{
    return (false);
  }
}

} // namespace visualization

} // namespace openpose

#endif  // OPENPOSEVISUALIZER_HPP
