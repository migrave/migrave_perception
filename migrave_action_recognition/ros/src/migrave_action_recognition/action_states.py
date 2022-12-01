#!/usr/bin/env python3
import rospy

from pyftsm.ftsm import FTSM, FTSMTransitions

from mas_tools.ros_utils import get_package_path
from action_recognition.action_learner import ActionLearner
from action_recognition.action_classifier import ActionClassifier
from action_recognition.action_model import ActionModel

from migrave_action_recognition.msg import ContinualActionLearningGoal, ContinualActionLearningResult, ContinualActionLearningFeedback

model_cfg_file = {'1': get_package_path("migrave_action_recognition", "config", "action_model_config.yaml"),
                  '2': get_package_path("migrave_action_recognition", "config", "exp_model1_config.yaml"),
                  '3': get_package_path("migrave_action_recognition", "config", "exp_model2_config.yaml"),
                  '4': get_package_path("migrave_action_recognition", "config", "exp_model3_config.yaml")}

action_list_file = {'1': get_package_path("migrave_action_recognition", "config", "action_list.txt"),
                    '2': get_package_path("migrave_action_recognition", "config", "exp_action_list1.txt"),
                    '3': get_package_path("migrave_action_recognition", "config", "exp_action_list2.txt"),
                    '4': get_package_path("migrave_action_recognition", "config", "exp_action_list3.txt")}

class ContinualActionLearningSM(FTSM):
    def __init__(self, args):
        super(ContinualActionLearningSM, self).__init__('ContinualActionLearning', [], args.max_recovery_attempts)

        self.execution_requested = False
        self.goal = None
        self.seq_size = args.seq_size
        self.num_seq = args.num_seq
        self.ske_topic = args.nuitrack_skeleton_topic
        self.mtype = args.model_type

        self.action_fb_pub = rospy.Publisher("/continual_action_learning/feedback", ContinualActionLearningFeedback, queue_size=1)

    def init(self):
        rospy.loginfo('Initialising Continual Action Learning Server')

        rosbag_file = get_package_path("migrave_action_recognition", "data", "bag_files")
        save_data_path = get_package_path("migrave_action_recognition", "data", "learned_data")
        model_path = get_package_path("migrave_action_recognition", "models")

        action_model = ActionModel(model_cfg_file[self.mtype], action_list_file[self.mtype], model_path)
        self.action_classifier = ActionClassifier(action_model, rosbag_file, self.mtype, self.ske_topic, self.seq_size)
        self.action_learner = ActionLearner(action_model, save_data_path, self.ske_topic, self.num_seq, self.seq_size)

        return FTSMTransitions.INITIALISED

    def configuring(self):
        rospy.loginfo('Continual Action Learning Server is Ready')
        return FTSMTransitions.DONE_CONFIGURING

    def ready(self):
        if self.execution_requested:
            self.execution_requested = False
            self.result = None
            return FTSMTransitions.RUN
        else:
            return FTSMTransitions.WAIT

    def running(self):
        if self.goal:
            if self.goal.request_type == ContinualActionLearningGoal.CLASSIFY:
                action = self.action_classifier.classify_action()

                if action is not None:
                    rospy.loginfo('Recognized action: %s, index %d', action[0], action[1])
                    self.publish_feedback(action[0])
                else:
                    self.publish_feedback("Action Not Recognized")

                return FTSMTransitions.CONTINUE
            elif self.goal.request_type == ContinualActionLearningGoal.LEARN:
                rospy.loginfo('Learning')
                self.action_classifier.record = False
                self.action_learner.learn(self.goal)
                rospy.sleep(2)

                self.action_classifier.reset()
                self.result = self.set_result(True)
                return FTSMTransitions.DONE
            elif self.goal.request_type == ContinualActionLearningGoal.STOP:
                rospy.loginfo('Stopping')
                rospy.sleep(2)
                self.result = self.set_result(False)
                return FTSMTransitions.DONE
            else:
                self.result = self.set_result(False)
                return FTSMTransitions.RECOVER
        else:
            return FTSMTransitions.DONE

    def recovering(self):
        rospy.loginfo('Continual Action Learning Server is Recovering')
        self.goal = None
        return FTSMTransitions.DONE_RECOVERING

    def set_result(self, success):
        result = ContinualActionLearningResult()
        result.success = success
        return result

    def publish_feedback(self, action):
        feedback = ContinualActionLearningFeedback()
        feedback.action_name = action
        self.action_fb_pub.publish(feedback)
