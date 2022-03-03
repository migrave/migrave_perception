#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>
#include <opencv2/core/core.hpp>
#include <openpose/flags.hpp>
#include <openpose/headers.hpp>

#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>

#include <migrave_ros_msgs/BodyPart.h>
#include <migrave_ros_msgs/Skeletons.h>

#include <migrave_skeleton_tracking/openpose_ros.h>
#include <migrave_skeleton_tracking/impl/openpose_helper.hpp>
#include <migrave_skeleton_tracking/impl/openpose_visualizer.hpp>

OpenPoseROS::OpenPoseROS(ros::NodeHandle* nh, op::Wrapper* op_wrapper):
  nh_(nh),
  op_wrapper_(op_wrapper),
  cloud_msg_received_count_(0),
  image_msg_received_count_(0),
  op_visualizer_("skeleton_visualization")
{
  // Load params
  nh_->param<std::string>("point_cloud_topic", point_cloud_topic_, 
                         "/camera/depth_registered/points");
  nh_->param<std::string>("rgb_image_topic", rgb_image_topic_, 
                         "/camera/rgb/image_raw");
  nh_->param<std::string>("pub_skeleton_topic", pub_skeleton_topic_, 
                         "skeletons");
  nh_->param<std::string>("target_frame_id", target_frame_id_, "base_link");
  nh_->param<bool>("continuous_tracking", continuous_tracking_, false);

  sub_event_in_ = nh_->subscribe("event_in", 1, &OpenPoseROS::eventCallback, this);
  pub_event_out_ = nh_->advertise<std_msgs::String>("event_out", 1);
  pub_debug_img_ = nh_->advertise<sensor_msgs::Image>("debug_image", 1);

  // Initialize frame publisher
  pub_skeletons_ = nh_->advertise<migrave_ros_msgs::Skeletons>(pub_skeleton_topic_, 10);
}

OpenPoseROS::~OpenPoseROS(){}

void OpenPoseROS::eventCallback(const std_msgs::String::ConstPtr &msg)
{
  std_msgs::String event_out;
  if (msg->data == "e_start")
  {
    // Synchronize callback
    image_sub_ = new message_filters::Subscriber<sensor_msgs::Image> (*nh_, rgb_image_topic_, 1);
    cloud_sub_ = new message_filters::Subscriber<sensor_msgs::PointCloud2> (*nh_, point_cloud_topic_, 1);
    msg_sync_ = new message_filters::Synchronizer<msgSyncPolicy> (msgSyncPolicy(10), *image_sub_, *cloud_sub_);
    msg_sync_->registerCallback(boost::bind(&OpenPoseROS::synchronizeCallback, this, _1, _2));

    event_out.data = "e_started";
    pub_event_out_.publish(event_out);
  }
  if (msg->data == "e_stop")
  {
    // FIXME: if message filter subscribe has not been initialized and e_stop is sent, 
    // this node will die
    if (image_sub_) image_sub_->unsubscribe();
    if (cloud_sub_) cloud_sub_->unsubscribe();

    event_out.data = "e_stopped";
    pub_event_out_.publish(event_out); 
  }
  {
    return;
  }
}

void OpenPoseROS::synchronizeCallback(const sensor_msgs::ImageConstPtr &image,
                                      const sensor_msgs::PointCloud2ConstPtr &cloud)
{
  if (image_msg_received_count_ < 1)
  {
    ROS_DEBUG("Received enough messages");
    cloud_msg_ = cloud;
    cloud_msg_received_count_ += 1;

    image_msg_ = image;
    image_msg_received_count_ += 1;
  }
}

void OpenPoseROS::update()
{
  if (cloud_msg_received_count_ > 0 && image_msg_received_count_ > 0)
  {
    ROS_DEBUG("Received %d image and cloud msgs", cloud_msg_received_count_);
    // Reset msg received flag
    cloud_msg_received_count_ = 0;
    image_msg_received_count_ = 0;

    if (!continuous_tracking_)
    {
      image_sub_->unsubscribe();
      cloud_sub_->unsubscribe();
    }

    // Start tracking
    double start_time = ros::Time::now().toSec();
    PointCloud::Ptr cloud(new PointCloud);
    pcl::fromROSMsg(*cloud_msg_, *cloud);
    OpenPoseROS::getSkeleton(image_msg_, cloud);
    double end_time = ros::Time::now().toSec();
    ROS_DEBUG("Total processing time: %f", end_time - start_time);
  }
}

template <typename key_points>
void OpenPoseROS::get3DPose(const key_points &kp, 
                            const int &i,
                            const PointCloud::ConstPtr &cloud,
                            migrave_ros_msgs::BodyPart &part) 
{
  // Only take key point with prob > 0
  if (kp[i+2] <= 0.0) return;

  // Check if the key point is within the image res, otherwise skip
  if (kp[i] > 640 || kp[i+1] > 480) return;

  part.keypoint_x = kp[i];
  part.keypoint_y = kp[i+1];
  part.keypoint_score = kp[i+2];

  PointT pcl_point = cloud->at(part.keypoint_x, part.keypoint_y);
  if ((!std::isnan(pcl_point.x)) && 
      (!std::isnan(pcl_point.y)) && 
      (!std::isnan(pcl_point.z))) 
  {
    part.point.x = pcl_point.x;
    part.point.y = pcl_point.y;
    part.point.z = pcl_point.z;
    
  }
  else
  {
    //ROS_WARN("PCL point contains nan");
    return;
  }
}

void OpenPoseROS::getSkeleton(const sensor_msgs::ImageConstPtr &image_msg, 
                              const PointCloud::ConstPtr &cloud)
{
  migrave_ros_msgs::Skeletons skeletons;
  skeletons.header.stamp = ros::Time::now();
  skeletons.header.frame_id = image_msg->header.frame_id;
  skeletons.skeletons.clear();

  cv::Mat image;
  try
  {
    image = cv_bridge::toCvShare(image_msg_, sensor_msgs::image_encodings::BGR8)->image;
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  // Fill datum and get tracked skeletons
  auto datum_ptr = op_wrapper_->emplaceAndPop(OP_CV2OPCONSTMAT(image));
  if (datum_ptr == nullptr) return;

  const auto& pose_kp = datum_ptr->at(0)->poseKeypoints;
  const auto& hand_kp = datum_ptr->at(0)->handKeypoints;

  // Get the number of persons, size of body and hand parts
  const auto num_persons = pose_kp.getSize(0);
  const auto body_part_count = pose_kp.getSize(1);
  const auto hand_part_count = hand_kp[0].getSize(1);

  skeletons.skeletons.resize(num_persons);
  int i;
  for (auto p = 0; p < num_persons; p++) {
    auto& curr_person = skeletons.skeletons[p];

    skeletons.skeletons[p].body_parts.resize(body_part_count);
    skeletons.skeletons[p].left_hand_parts.resize(hand_part_count);
    skeletons.skeletons[p].right_hand_parts.resize(hand_part_count);

    // Fill body parts
    for (auto bp = 0; bp < body_part_count; bp++) {
      i = pose_kp.getSize(2) * (p * body_part_count + bp);
      OpenPoseROS::get3DPose(pose_kp, i, cloud, skeletons.skeletons[p].body_parts[bp]);
    }

    // Fill left and right hands
    for (auto hp = 0; hp < hand_part_count; hp++) {
      i = hand_kp[0].getSize(2) * (p * hand_part_count + hp);

      OpenPoseROS::get3DPose(hand_kp[0], i, cloud, skeletons.skeletons[p].left_hand_parts[hp]);
      OpenPoseROS::get3DPose(hand_kp[1], i, cloud, skeletons.skeletons[p].left_hand_parts[hp]);
    }
  }
  // Publish and visualize skeletons
  pub_skeletons_.publish(skeletons);
  op_visualizer_.publish(skeletons);

  // Publish debug image
  sensor_msgs::ImagePtr debug_img = cv_bridge::CvImage(std_msgs::Header(), 
                                                       "bgr8", 
                                                       OP_OP2CVCONSTMAT(datum_ptr->at(0)->cvOutputData)
                                                       ).toImageMsg();
  pub_debug_img_.publish(*debug_img);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "openpose_ros");
  ros::NodeHandle nh("~");
  // Initialize frame rate
  int frame_rate = 30;
  nh.param<int>("frame_rate", frame_rate, 30);
  ros::Rate loop_rate(frame_rate);
  ROS_INFO_STREAM("\033[1;32m Node started with rate " 
                   << frame_rate << " \033[0m\n");

  // Parse Openpose Args
  gflags::ParseCommandLineFlags(&argc, &argv, true);

  // Use Asynchronous to call empty constructor WrapperT, then apply user configs,
  // and call emplace and pop fuction to track skeleton given an image and 
  // retrive the result.
  op::Wrapper op_wrapper{op::ThreadManagerMode::Asynchronous};
  openpose::configureOPWrapper(&op_wrapper);
  op_wrapper.start();

  // Start ROS wrapper
  OpenPoseROS skeleton_tracking(&nh, &op_wrapper);

  while (ros::ok())
  {
    skeleton_tracking.update();
    ros::spinOnce();
    loop_rate.sleep();
  }
  op_wrapper.stop();
  return 0;
}
