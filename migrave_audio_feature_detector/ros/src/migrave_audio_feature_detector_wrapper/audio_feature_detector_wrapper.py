import numpy as np

import rospy

from std_msgs.msg import String
from audio_common_msgs.msg import AudioData

from migrave_audio_feature_detector.audio_feature_detector import AudioFeatures, AudioFeatureDetector
from migrave_ros_msgs.msg import AudioFeatures as AudioFeatureMsg

def feature_obj_to_msg(features: AudioFeatures) -> AudioFeatureMsg:
    msg = AudioFeatureMsg()
    msg.pitch_frequency = features.pitch_frequency
    msg.var_pitch_frequency = features.var_pitch_frequency
    msg.pitch_strength = features.pitch_strength
    msg.var_pitch_strength = features.var_pitch_strength
    msg.mean_harmonicity = features.mean_harmonicity
    msg.var_harmonicity = features.var_harmonicity
    msg.mean_intensity = features.mean_intensity
    msg.var_intensity = features.var_intensity
    msg.mfccs = features.mfccs.tolist()
    msg.var_mfccs = features.var_mfccs.tolist()
    return msg

class AudioFeatureDetectorWrapper(object):
    feature_detector = None
    audio_data_sub = None
    feature_pub = None
    event_pub = None
    event_sub = None
    event_out_data = String()
    debug = False

    def __init__(self):
        self.audio_data_topic = rospy.get_param("~audio_data_topic", None)
        self.feature_topic = rospy.get_param("~feature_topic", None)
        self.debug = rospy.get_param("~debug", False)

        detector_config_path = rospy.get_param("~config_file_path", None)
        self.feature_detector = AudioFeatureDetector(detector_config_path)

    def audio_data_cb(self, audio_msg: AudioData):
        sound_frame = np.frombuffer(audio_msg.data, np.int16)
        features = self.feature_detector.get_audio_features(sound_frame)
        feature_msg = feature_obj_to_msg(features)
        self.feature_pub.publish(feature_msg)

    def event_cb(self, data: String) -> None:
        if data.data == "e_start":
            self.feature_pub = rospy.Publisher(self.feature_topic,
                                               AudioFeatureMsg,
                                               queue_size=1)
            self.audio_data_sub = rospy.Subscriber(self.audio_data_topic,
                                                   AudioData,
                                                   self.audio_data_cb)
            self.event_out_data.data = "e_started"
            self.event_pub.publish(self.event_out_data)
        elif data.data == "e_stop":
            self.event_out_data.data = "e_stopped"
            self.event_pub.publish(self.event_out_data)

    def run(self) -> None:
        self.event_sub = rospy.Subscriber("~event_in", String, self.event_cb)
        self.event_pub = rospy.Publisher("~event_out", String, queue_size=1)
