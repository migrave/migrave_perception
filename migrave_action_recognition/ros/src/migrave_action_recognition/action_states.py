#!/usr/bin/env python3
import rospy

from pyftsm.ftsm import FTSM, FTSMTransitions

from mas_tools.ros_utils import get_package_path
from action_recognition.action_learner import ActionLearner
from action_recognition.action_classifier import ActionClassifier
from action_recognition.action_model import ActionModel

from migrave_action_recognition.msg import ContinualActionLearningGoal, ContinualActionLearningResult, ContinualActionLearningFeedback

class ContinualActionLearningSM(FTSM):
    def __init__(self, max_recovery_attempts=1):
        super(ContinualActionLearningSM, self).__init__('ContinualActionLearning', [], max_recovery_attempts)

        self.execution_requested = False
        self.goal = None

        self.action_fb_pub = rospy.Publisher("/continual_action_learning/feedback", ContinualActionLearningFeedback, queue_size=1)

    def init(self):
        rospy.loginfo('Initialising Continual Action Learning Server')

        model_cfg_file = get_package_path("migrave_action_recognition", "config", "action_model_config.yaml")
        action_list_file = get_package_path("migrave_action_recognition", "config", "action_list.txt")
        rosbag_file = get_package_path("migrave_action_recognition", "data", "bag_files")
        save_data_path = get_package_path("migrave_action_recognition", "data", "learned_data")
        model_path = get_package_path("migrave_action_recognition", "models")

        action_model = ActionModel(model_cfg_file, action_list_file, model_path)
        self.action_classifier = ActionClassifier(action_model, rosbag_file, seq_size=50)
        self.action_learner = ActionLearner(action_model, save_data_path)

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
