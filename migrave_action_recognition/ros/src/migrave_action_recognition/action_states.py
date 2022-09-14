#!/usr/bin/env python3

import rospy

from pyftsm.ftsm import FTSM, FTSMTransitions

from mas_tools.ros_utils import get_package_path
from action_recognition.action_learner import ActionLearner
from action_recognition.action_classifier import ActionClassifier
from action_recognition.action_model import ActionModel

from migrave_action_recognition.msg import ContinualActionLearningResult

class ContinualActionLearningSM(FTSM):
    def __init__(self, max_recovery_attempts=1):
        super(ContinualActionLearningSM, self).__init__('ContinualActionLearning', [], max_recovery_attempts)
        
        self.execution_requested = False
        self.goal = None
        self.result = None
            
    def init(self):
        rospy.loginfo('Initialising Continual Action Learning Server')

        model_cfg_file = get_package_path("migrave_action_recognition", "config", "action_model_config.yaml")
        action_list_file = get_package_path("migrave_action_recognition", "config", "action_list.txt")

        action_model = ActionModel(model_cfg_file, action_list_file)
        self.action_classifier = ActionClassifier(action_model)
        self.action_learner = ActionLearner(action_model)
        
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
            if self.result:
                self.result = None
            return FTSMTransitions.WAIT
        
    def running(self):
        rospy.loginfo('Running Continual Action Learning Server')
        
        rospy.sleep(2)
        
        return FTSMTransitions.DONE
        
    def recovering(self):
        rospy.loginfo('Continual Action Learning Server is Recovering')
    
        return FTSMTransitions.DONE_RECOVERING
        
    def set_result(self, success):
        result = ContinualActionLearningResult()
        result.success = success
        return result
