import os
import rospy
from cv_bridge import CvBridge, CvBridgeError

from std_msgs.msg import Header
from sensor_msgs.msg import Image
from geometry_msgs.msg import Pose, Point, Vector3
from std_msgs.msg import String
from migrave_ros_msgs.msg import AffectiveState, AudioFeatures, Face, FaceActionUnit
from qt_nuitrack_app.msg import Faces

from migrave_face_feature_detector.face_feature_detector import FaceFeatures
from migrave_common_ros import vision_utils as ros_vision_utils
from migrave_common import file_utils


class FaceFeatureDetectorWrapper(object):

    def __init__(self):
        self._config_file = rospy.get_param("~config_path", None)

        if not self._config_file or os.path.isfile(self._config_file):
            rospy.logerr("Config file is not given or does not exist")

        self._config = file_utils.parse_yaml_config(self._config_file)
        self._debug = rospy.get_param("~debug", False) 
        self._demo = rospy.get_param("~demo", False) 

        self._rgb_image = rospy.get_param("~rgb_image_topic", "/camera/rgb/image_raw") 
        self._depth_image = rospy.get_param("~rgb_depth_topic", "/camera/depth/image_raw") 
        self._camera_frame_id = rospy.get_param("~camera_frame_id", "camera_rgb_optical_frame") 

        # publishers
        self._pub_face_feature = rospy.Publisher("~face_features", Face, queue_size=1)
        
        self._cvbridge = CvBridge()
        self._face_feature_detector = FaceFeatures(self._config)

    def rgb_image_cb(self, data: Image) -> None:
        rospy.logdebug("RGB image msg received")
        cv_image = ros_vision_utils.get_cv_image(data)
        self.get_face_features(cv_image)

    def depth_image_cb(self, data: Image) -> None:
        rospy.logdebug("Depth image msg received")

    def get_face_features(self, image) -> None:
        #estimate affective state
        rospy.logdebug("Computing face features...")

        #openface returns landmarks
        face_landmarks = self._face_feature_detector.get_face_features(image)

        face = Face()
        header = Header()
        header.stamp = rospy.Time.now()
        header.frame_id = self._camera_frame_id
        face.header = header

        face.detection_confidence = 0.01

        left_gaze = Vector3()
        right_gaze = Vector3()
        face.left_gaze = left_gaze
        face.right_gaze = right_gaze

        head_pose = Pose()
        face.head_pose = head_pose

        landmarks_3d = Point()
        landmarks_2d = Point()
        face.landmarks_3d = [landmarks_3d]
        face.landmarks_2d = [landmarks_2d]

        action_units = FaceActionUnit()
        face.action_units = [action_units]

        self._pub_face_feature.publish(face)

    def event_callback(self, data: String) -> None:
        event_out_data = String()
        if data.data == "e_start":
            self._sub_rgb_image = rospy.Subscriber(self._rgb_image, Image, self.rgb_image_cb)
            # self._sub_depth_image = rospy.Subscriber(self._depth_image, Image, self.depth_image_cb)

            event_out_data.data = "e_started"
            self._pub_event.publish(event_out_data)

        elif data.data == "e_stop":
            event_out_data.data = "e_stopped"
            self._pub_event.publish(event_out_data)

    def run(self) -> None:
        #start event in and out
        self._sub_event = rospy.Subscriber("~event_in", String, self.event_callback)
        self._pub_event = rospy.Publisher("~event_out", String, queue_size=1)
