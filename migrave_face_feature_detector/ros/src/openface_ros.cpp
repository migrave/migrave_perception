#include <vector>
#include <unordered_map>

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <geometry_msgs/TransformStamped.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>

#include <OpenFace/LandmarkCoreIncludes.h>
#include <OpenFace/Face_utils.h>
#include <OpenFace/FaceAnalyser.h>
#include <OpenFace/GazeEstimation.h>
#include <OpenFace/Visualizer.h>
#include <OpenFace/VisualizationUtils.h>

#include <tf2/LinearMath/Quaternion.h>

#include <migrave_face_feature_detector/openface_ros.h>
#include <migrave_ros_msgs/Face.h>
#include <migrave_ros_msgs/Faces.h>
#include <migrave_ros_msgs/FaceActionUnit.h>

OpenFaceROS::OpenFaceROS(ros::NodeHandle &nh):
  nh_(nh),
  max_faces_(2),
  image_msg_received_(false),
  camera_info_received_(false),
  continuous_tracking_(true)
{
  sub_event_in_ = nh_.subscribe("event_in", 1, &OpenFaceROS::eventCallback, this);
  pub_event_out_ = nh_.advertise<std_msgs::String>("event_out", 1);
  pub_faces_ = nh_.advertise<migrave_ros_msgs::Faces>("faces", 1);

  nh_.param<std::string>("cam_info_topic", cam_info_topic_, "/camera/rgb/camera_info");
  cam_info_sub_ = nh_.subscribe(cam_info_topic_, 1, &OpenFaceROS::camInfoCallback, this);

  nh_.param<std::string>("rgb_image_topic", rgb_image_topic_, "/camera/color/image_raw");

  pub_debug_img_ = nh_.advertise<sensor_msgs::Image>("debug_image", 1);

  initializeOpenFace();
}

OpenFaceROS::~OpenFaceROS(){}

void OpenFaceROS::eventCallback(const std_msgs::String::ConstPtr &msg)
{
  std_msgs::String event_out_msg;
  if (msg->data == "e_start")
  {
    //ROS_INFO("e_start received");
    event_out_msg.data = "e_started";

    // subscribe to image topic
    image_sub_ = nh_.subscribe(rgb_image_topic_, 1, &OpenFaceROS::imageCallback, this);

    pub_event_out_.publish(event_out_msg);
  }
  else if (msg->data == "e_stop")
  {
    image_sub_.shutdown();

    event_out_msg.data = "e_stopped";
    pub_event_out_.publish(event_out_msg);
  }
  else
  {
    return;
  }
}

void OpenFaceROS::camInfoCallback(const sensor_msgs::CameraInfoConstPtr &msg)
{
  // camera params
  if (!camera_info_received_)
  {
    fx_ = msg->K[0];
    fy_ = msg->K[4];
    cx_ = msg->K[2];
    cy_ = msg->K[5];
    camera_info_received_ = true;
  }
}

void OpenFaceROS::imageCallback(const sensor_msgs::ImageConstPtr &image)
{
  if (!image_msg_received_)
  {
    image_msg_ = image;
    image_msg_received_ = true;
  }
}

void OpenFaceROS::update()
{
  // Stop subscribing if continuous tracking is false
  if (!continuous_tracking_) image_sub_.shutdown();

  if (image_msg_received_)
  {
    trackFaces();
  }

  // Reset image msg flag
  image_msg_received_ = false;
}

void OpenFaceROS::initializeOpenFace()
{
  // Load facial feature extractor and AU analyser
  FaceAnalysis::FaceAnalyserParameters fa_params;
  fa_params.OptimizeForImages();
  face_analyser_ = FaceAnalyserPtr(new FaceAnalysis::FaceAnalyser(fa_params));

  std::vector<std::string> arguments(1,"");
  LandmarkDetector::FaceModelParameters det_params(arguments);

	// This is so that the model would not try re-initialising itself
	det_params.reinit_video_every = -1;
	det_params.curr_face_detector = LandmarkDetector::FaceModelParameters::MTCNN_DETECTOR;

	//std::vector<LandmarkDetector::FaceModelParameters> det_parameters;
	det_parameters_.push_back(det_params);

	LandmarkDetector::CLNF face_model(det_parameters_[0].model_location);

	if (!face_model.loaded_successfully)
	{
		ROS_ERROR("Could not load the landmark detector");
		return;
	}

	// Loading the face detectors
	face_model.face_detector_HAAR.load(det_parameters_[0].haar_face_detector_location);
	face_model.haar_face_detector_location = det_parameters_[0].haar_face_detector_location;
	face_model.face_detector_MTCNN.Read(det_parameters_[0].mtcnn_face_detector_location);
	face_model.mtcnn_face_detector_location = det_parameters_[0].mtcnn_face_detector_location;

	// If can't find MTCNN face detector, default to HOG one
	if (det_parameters_[0].curr_face_detector == LandmarkDetector::FaceModelParameters::MTCNN_DETECTOR 
      && face_model.face_detector_MTCNN.empty())
	{
		ROS_INFO("Defaulting to HOG-SVM face detector");
		det_parameters_[0].curr_face_detector = LandmarkDetector::FaceModelParameters::HOG_SVM_DETECTOR;
	}

	face_models_.reserve(max_faces_);

	face_models_.push_back(face_model);
	active_models_.push_back(false);

	for (int i = 1; i < max_faces_; ++i)
	{
		face_models_.push_back(face_model);
		active_models_.push_back(false);
		det_parameters_.push_back(det_params);
	}

	if (!face_model.eye_model)
	{
		ROS_WARN("No eye model found");
	}

	if (face_analyser_->GetAUClassNames().size() == 0)
	{
		ROS_WARN("No Action Unit models found");
	}

  // OpenFace visualizer
  of_visualizer_ = OFVisualizerPtr(new Utilities::Visualizer(true, false, false, true));

  ROS_INFO("OpenFace initialized!");
}

double OpenFaceROS::getIOU(cv::Rect_<float> rect1, cv::Rect_<float> rect2)
{
	double intersection_area = (rect1 & rect2).area();
	double union_area = rect1.area() + rect2.area() - intersection_area;
	return intersection_area / union_area;
}

void OpenFaceROS::nonOverlapingDetections(const std::vector<LandmarkDetector::CLNF>& clnf_models, 
                                       std::vector<cv::Rect_<float> >& face_detections)
{
	// Go over the model and eliminate detections that are not informative 
  // (there already is a tracker there)
	for (size_t model = 0; model < clnf_models.size(); ++model)
	{

		// See if the detections intersect
		cv::Rect_<float> model_rect = clnf_models[model].GetBoundingBox();

		for (int detection = face_detections.size() - 1; detection >= 0; --detection)
		{
			// If the model is already tracking what we're detecting ignore the detection, 
      // this is determined by amount of overlap
			if (OpenFaceROS::getIOU(model_rect, face_detections[detection]) > 0.5)
			{
				face_detections.erase(face_detections.begin() + detection);
			}
		}
	}
}

void OpenFaceROS::removeOverlapingModels(std::vector<LandmarkDetector::CLNF>& face_models, 
                                              std::vector<bool>& active_models)
{
	// Go over the model and eliminate detections that are not informative (there already is a tracker there)
	for (size_t model1 = 0; model1 < active_models.size(); ++model1)
	{
		if (active_models[model1])
		{
			// See if the detections intersect
			cv::Rect_<float> model1_rect = face_models[model1].GetBoundingBox();

			for (int model2 = model1 + 1; model2 < active_models.size(); ++model2)
			{
				if(active_models[model2])
				{
					cv::Rect_<float> model2_rect = face_models[model2].GetBoundingBox();

					// If the model is already tracking what we're detecting ignore 
          // the detection, this is determined by amount of overlap
					if (OpenFaceROS::getIOU(model1_rect, model2_rect) > 0.5)
					{
						active_models[model1] = false;
						face_models[model1].Reset();
					}
				}
			}
		}
	}
}

// Some codes are based on FaceLandmarkVidMulti example
// https://github.com/TadasBaltrusaitis/OpenFace/blob/OpenFace_2.2.0/exe/FaceLandmarkVidMulti/FaceLandmarkVidMulti.cpp
// Some of the ROS related functionalities were adopted from
// https://github.com/interaction-lab/openface_ros/blob/master/src/openface_ros.cpp
void OpenFaceROS::trackFaces()
{
  cv_bridge::CvImagePtr cv_image;
  cv_bridge::CvImagePtr cv_image_mono;
  try
  {
    cv_image = cv_bridge::toCvCopy(image_msg_, sensor_msgs::image_encodings::BGR8);
    cv_image_mono = cv_bridge::toCvCopy(image_msg_, sensor_msgs::image_encodings::MONO8);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  // Defaul intrinsic cam params if fx and fy is 0
  if(fx_ == 0 || fy_ == 0)
  {
    fx_ = 500.0 * cv_image->image.cols / 640.0;
    fy_ = 500.0 * cv_image->image.rows / 480.0;
    fx_ = (fx_ + fy_) / 2.0;
    fy_ = fx_;
  }
  if(cx_ == 0) cx_ = cv_image->image.cols / 2.0;
  if(cy_ == 0) cy_ = cv_image->image.rows / 2.0;

  std::vector<cv::Rect_<float> > face_detections;

  bool all_models_active = true;
  for (unsigned int model = 0; model < face_models_.size(); ++model)
  {
    if (!active_models_[model])
    {
      all_models_active = false;
    }
  }
  
  // example code run detection every 8th frame
  if(!all_models_active)
  {
    ROS_DEBUG_STREAM("Start tracking face, model:" << det_parameters_[0].curr_face_detector);
    if (det_parameters_[0].curr_face_detector == LandmarkDetector::FaceModelParameters::HOG_SVM_DETECTOR)
    {
      std::vector<float> confidences;
      LandmarkDetector::DetectFacesHOG(face_detections, 
                                        cv_image_mono->image, 
                                        face_models_[0].face_detector_HOG, 
                                        confidences);
    }
    else if (det_parameters_[0].curr_face_detector == LandmarkDetector::FaceModelParameters::HAAR_DETECTOR)
    {
      LandmarkDetector::DetectFaces(face_detections, 
                                    cv_image_mono->image, 
                                    face_models_[0].face_detector_HAAR);
    }
    else
    {
      std::vector<float> confidences;
      LandmarkDetector::DetectFacesMTCNN(face_detections, 
                                          cv_image->image, 
                                          face_models_[0].face_detector_MTCNN, 
                                          confidences);
    }
  }

  // Keep only non overlapping detections (also convert to a concurrent vector
  OpenFaceROS::nonOverlapingDetections(face_models_, face_detections);
  std::vector<bool> face_detections_used(face_detections.size(), false);

  // Go through every model and update the tracking
  for (unsigned int model = 0; model < face_models_.size(); ++model)
  {
    bool detection_success = false;
    // If the current model has failed more than 4 times in a row, remove it
    if (face_models_[model].failures_in_a_row > 4)
    {
      active_models_[model] = false;
      face_models_[model].Reset();
    }

    // If the model is inactive reactivate it with new detections
    if (!active_models_[model])
    {
      for (size_t detection_ind = 0; detection_ind < face_detections.size(); ++detection_ind)
      {
        // if it was not taken by another tracker take it
        if (!face_detections_used[detection_ind])
        {
          face_detections_used[detection_ind] = true;
          // Reinitialise the model
          face_models_[model].Reset();
          // This ensures that a wider window is used for the initial landmark localisation
          face_models_[model].detection_success = false;
          detection_success = LandmarkDetector::DetectLandmarksInVideo(cv_image->image, 
                                                face_detections[detection_ind], 
                                                face_models_[model], 
                                                det_parameters_[model], 
                                                cv_image_mono->image);

          // This activates the model
          active_models_[model] = true;

          // break out of the loop as the tracker has been reinitialised
          break;
        }

      }
    }
    else
    {
      // The actual facial landmark detection / tracking
      detection_success = LandmarkDetector::DetectLandmarksInVideo(cv_image->image, 
                                                          face_models_[model], 
                                                          det_parameters_[model], 
                                                          cv_image_mono->image);
    }
  }

  // Remove models that end up tracking overlapping faces
  // even if initial bounding boxes were not overlapping, 
  // they could have ended up converging to the same face
  OpenFaceROS::removeOverlapingModels(face_models_, active_models_);

  //decltype(cv_image->image) viz_img = cv_image->image.clone();
  if(pub_debug_img_) of_visualizer_->SetImage(cv_image->image, fx_, fy_, cx_, cy_);

  // Keeping track of FPS
  of_fps_tracker_.AddFrame();

  migrave_ros_msgs::Faces faces;
  faces.header.frame_id = image_msg_->header.frame_id;
  faces.header.stamp = ros::Time::now();

  // Go through every model and detect eye gaze, record results
  for (size_t model = 0; model < face_models_.size(); ++model)
  {
    // Visualising and recording the results
    if (active_models_[model])
    {
      // Estimate head pose and eye gaze
      migrave_ros_msgs::Face face;
      face.header.frame_id = image_msg_->header.frame_id;
      face.header.stamp = ros::Time::now();

      // Estimate head pose and eye gaze				
      cv::Vec6d head_pose = LandmarkDetector::GetPose(face_models_[model], fx_, fy_, cx_, cy_);
      face.head_pose.position.x = head_pose[0];
      face.head_pose.position.y = head_pose[1];
      face.head_pose.position.z = head_pose[2];
          
      const auto head_orientation = toQuaternion(head_pose[4], -head_pose[3], -head_pose[5]);
      //toQuaternion(M_PI / 2, 0, M_PI / 2);// toQuaternion(0, 0, 0);  toQuaternion(M_PI,  0,  0);
      face.head_pose.orientation = toQuaternion(M_PI / 2, 0, M_PI / 2);
      face.head_pose.orientation = face.head_pose.orientation * head_orientation;

      geometry_msgs::TransformStamped transform;
      transform.header = face.header;
      std::stringstream out;
      out << "head" << model;
      transform.child_frame_id = out.str();
      transform.transform.translation.x = face.head_pose.position.x / 1000.0;
      transform.transform.translation.y = face.head_pose.position.y / 1000.0;
      transform.transform.translation.z = face.head_pose.position.z / 1000.0;
      transform.transform.rotation = face.head_pose.orientation;
      tf2_br_.sendTransform(transform);
        
      const std::vector<cv::Point3f> eye_landmarks3d = LandmarkDetector::Calculate3DEyeLandmarks(face_models_[model], 
                                                          fx_, fy_, cx_, cy_);
      cv::Point3f gaze_direction0(0, 0, 0); 
      cv::Point3f gaze_direction1(0, 0, 0); 
      cv::Vec2d gaze_angle(0, 0);

      // Detect eye gazes
      if (face_models_[model].detection_success && face_models_[model].eye_model)
      {
        GazeAnalysis::EstimateGaze(face_models_[model], gaze_direction0, 
                                    fx_, fy_, cx_, cy_, true);
        GazeAnalysis::EstimateGaze(face_models_[model], gaze_direction1, 
                                    fx_, fy_, cx_, cy_, false);

        gaze_angle = GazeAnalysis::GetGazeAngle(gaze_direction0, gaze_direction1);	

        face.left_gaze.orientation = toQuaternion(M_PI , 0, 0) * 
                                      toQuaternion(gaze_direction0.y, 
                                                  -gaze_direction0.x, 
                                                  -gaze_direction0.z);
        face.right_gaze.orientation = toQuaternion(M_PI , 0, 0) * 
                                      toQuaternion(gaze_direction1.y, 
                                                    -gaze_direction1.x, 
                                                    -gaze_direction1.z);

        face.gaze_angle.x = gaze_angle[0];
        face.gaze_angle.y = gaze_angle[1];

        // Grabbing the pupil location, to determine eye gaze vector, 
        // we need to know where the pupil is
        cv::Point3f pupil_left(0, 0, 0);
        cv::Point3f pupil_right(0, 0, 0);
        for (size_t i = 0; i < 8; ++i)
        {
          pupil_left = pupil_left + eye_landmarks3d[i];
          pupil_right = pupil_right + eye_landmarks3d[i + eye_landmarks3d.size()/2];
        }

        pupil_left = pupil_left / 8;
        pupil_right = pupil_right / 8;

        face.left_gaze.position.x = pupil_left.x;
        face.left_gaze.position.y = pupil_left.y;
        face.left_gaze.position.z = pupil_left.z;

        face.right_gaze.position.x = pupil_right.x;
        face.right_gaze.position.y = pupil_right.y;
        face.right_gaze.position.z = pupil_right.z;

        std::stringstream out_left;
        out_left << "head" << model << "_left_eye";
        transform.child_frame_id = out_left.str();
        transform.transform.translation.x = face.left_gaze.position.x / 1000.0;
        transform.transform.translation.y = face.left_gaze.position.y / 1000.0;
        transform.transform.translation.z = face.left_gaze.position.z / 1000.0;
        transform.transform.rotation = face.left_gaze.orientation;
        tf2_br_.sendTransform(transform);

        std::stringstream out_right;
        out_right << "head" << model << "_right_eye";
        transform.child_frame_id = out_right.str();
        transform.transform.translation.x = face.right_gaze.position.x / 1000.0;
        transform.transform.translation.y = face.right_gaze.position.y / 1000.0;
        transform.transform.translation.z = face.right_gaze.position.z / 1000.0;
        transform.transform.rotation = face.right_gaze.orientation;
        tf2_br_.sendTransform(transform);
      }
      
      //extract facial landmarks
      const auto &landmarks = face_models_[model].detected_landmarks;
      for(unsigned i = 0; i < face_models_[model].pdm.NumberOfPoints(); ++i)
      {
        geometry_msgs::Point p;
        p.x = landmarks.at<float>(i);
        p.y = landmarks.at<float>(face_models_[model].pdm.NumberOfPoints() + i);
        face.landmarks_2d.push_back(p);
      }

      cv::Mat_<double> shape_3d = face_models_[model].GetShape(fx_, fy_, cx_, cy_);
      for(unsigned i = 0; i < face_models_[model].pdm.NumberOfPoints(); ++i)
      {
        geometry_msgs::Point p;
        p.x = shape_3d.at<double>(i);
        p.y = shape_3d.at<double>(face_models_[model].pdm.NumberOfPoints() + i);
        p.z = shape_3d.at<double>(face_models_[model].pdm.NumberOfPoints() * 2 + i);
        face.landmarks_3d.push_back(p);
      }

      // Face action units
      face_analyser_->PredictStaticAUsAndComputeFeatures(cv_image->image, 
                                      face_models_[model].detected_landmarks);

      auto aus_reg = face_analyser_->GetCurrentAUsReg();
      auto aus_class = face_analyser_->GetCurrentAUsClass();

      std::unordered_map<std::string, migrave_ros_msgs::FaceActionUnit> aus;
      for(const auto &au_reg : aus_reg)
      {
        auto it = aus.find(std::get<0>(au_reg));
        if(it == aus.end())
        {
          migrave_ros_msgs::FaceActionUnit fau;
          fau.name = std::get<0>(au_reg);
          fau.intensity = std::get<1>(au_reg);
          aus.insert({ std::get<0>(au_reg), fau});
          continue;
        }
        it->second.intensity = std::get<1>(au_reg);
      }

      for(const auto &au_class : aus_class)
      {
        auto it = aus.find(std::get<0>(au_class));
        if(it == aus.end())
        {
          migrave_ros_msgs::FaceActionUnit fau;
          fau.name = std::get<0>(au_class);
          fau.presence = std::get<1>(au_class);
          aus.insert({ std::get<0>(au_class), fau});
          continue;
        }
        it->second.presence = std::get<1>(au_class);
      }

      for(const auto &au : aus) face.action_units.push_back(std::get<1>(au));

      cv::Point min(100000, 100000);
      cv::Point max(0, 0);
      for(const auto &p : face.landmarks_2d)
      {
        if(p.x < min.x) min.x = p.x;
        if(p.y < min.y) min.y = p.y;
        if(p.x > max.x) max.x = p.x;
        if(p.y > max.y) max.y = p.y;
      }

      of_visualizer_->SetObservationLandmarks(face_models_[model].detected_landmarks, 
                                          face_models_[model].detection_certainty);
      of_visualizer_->SetObservationPose(LandmarkDetector::GetPose(face_models_[model], fx_, fy_, cx_, cy_), 
                                    face_models_[model].detection_certainty);
      of_visualizer_->SetObservationGaze(gaze_direction0, 
                                    gaze_direction1, 
                                    LandmarkDetector::CalculateAllEyeLandmarks(face_models_[model]),
                                    eye_landmarks3d, 
                                    face_models_[model].detection_certainty);
      of_visualizer_->SetObservationActionUnits(aus_reg, aus_class);
      faces.faces.push_back(face);
    }
  }
  pub_faces_.publish(faces);
  
  ROS_INFO("Detected faces: %d", faces.faces.size());
  of_visualizer_->SetFps(of_fps_tracker_.GetFPS());
  auto viz_msg = cv_bridge::CvImage(cv_image->header, 
                                    "bgr8", 
                                    of_visualizer_->GetVisImage()).toImageMsg();
  pub_debug_img_.publish(viz_msg);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "openface_ros");
  ros::NodeHandle nh("~");
  int frame_rate = 30;
  nh.param<int>("frame_rate", frame_rate, 30);
  ros::Rate loop_rate(frame_rate);

  OpenFaceROS of_ros(nh);
  while (ros::ok())
  {
    of_ros.update();
    ros::spinOnce();
    loop_rate.sleep();
  }
}
