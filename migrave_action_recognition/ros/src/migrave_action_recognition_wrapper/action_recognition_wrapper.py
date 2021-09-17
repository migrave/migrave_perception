import rospy

from std_msgs.msg import String

from qt_nuitrack_app.msg import Skeletons
from migrave_common import file_utils
from migrave_ros_msgs.msg import Activity, Person

from migrave_action_recognition.action_recognition import ActionRecognition

class ActionRecognitionWrapper(object):

    def __init__(self):
        self._config_file = rospy.get_param("~config_path", None)
        self._skeleton_topic = rospy.get_param("~skeleton_topic", "/qt_nuitrack_app/skeletons")
        self._debug = rospy.get_param("~debug", False) 

        self._config = file_utils.parse_yaml_config(self._config_file)

        # publisher
        self._pub_activity = rospy.Publisher("~activity", Activity, queue_size=1)

        self._action_recognizer = ActionRecognition(self._config)

    def skeleton_cb(self, data: Skeletons) -> None:
        # rospy.loginfo("Skeleton msg received")

        #ToDo: recognize action
        recognized_action = self._action_recognizer.recognize_action(data)
        print(recognized_action)

        activity = Activity()
        activity.stamp = rospy.Time.now()
        #ToDo: get person from therapist's tablet?
        person = Person()
        person.id = "1"
        person.name = "1"
        person.age = 4
        person.gender = "1"
        person.mother_tongue = "1"
        
        activity.person = person
        activity.activity = activity
        activity.confidence = 1.0

        self._pub_activity.publish(activity)


    def event_callback(self, data: String) -> None:
        event_out_data = String()
        if data.data == "e_start":
            self._sub_skeleton = rospy.Subscriber(self._skeleton_topic, Skeletons, self.skeleton_cb)

            event_out_data.data = "e_started"
            self._pub_event.publish(event_out_data)

        elif data.data == "e_stop":
            event_out_data.data = "e_stopped"
            self._pub_event.publish(event_out_data)

    def run(self) -> None:
        #start event in and out
        self._sub_event = rospy.Subscriber("~event_in", String, self.event_callback)
        self._pub_event = rospy.Publisher("~event_out", String, queue_size=1)
