#!/usr/bin/env python3

import rospy
import numpy as np
from queue import Queue

from qt_nuitrack_app.msg import Skeletons
from migrave_skeleton_tools_ros.skeleton_utils import JointUtils

ntu_joint_map = [3, 2, 1, 0, 20, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 16, 17, 18]

class ActionLearner(object):
    def __init__(self, action_model, nuitrack_skeleton_topic="/qt_nuitrack_app/skeletons"):

        self.action_data = np.zeros((25, 80, 19*6), dtype=np.float32)
        self.ske_seq = Queue(maxsize = 10)
        self.seq_cnt = 0

        self.ske_sub = rospy.Subscriber(nuitrack_skeleton_topic, Skeletons, self.process_nt_data)
        self.record = False

        self.model = action_model


    def learn(self, action):

        self.action_data = np.zeros((25*action.num_actions, 80, 19*6), dtype=np.float32)

        rospy.loginfo('Sleeping for 3 seconds; get into position!')
        rospy.sleep(3.)
        rospy.loginfo('Capturing data...')

        self.record = True
        while self.seq_cnt < 25:
            continue

        self.record = False

        rospy.loginfo('Training model...')
        N, T, J = self.action_data.shape
        x = self.action_data.reshape((N, T, 2, int(J/6), 3)).transpose(0, 4, 1, 3, 2)
        print(x[20:25].shape)
        trn_data = {'x': x[:20], 'y': [len(self.model.actions)]*20}
        val_data = {'x': x[20:25], 'y': [len(self.model.actions)]*5}
        
        self.model.train(action, trn_data, val_data)

        rospy.loginfo('Saving new model...')
        #self.model.save_model(action)

        return True


    def process_nt_data(self, ske_data_msg):
        if not self.record or not ske_data_msg.skeletons:
            return

#        rospy.loginfo('Recieved Skeleton Data...Processing Now')
        first_skeleton_msg = ske_data_msg.skeletons[0]

        joint_positions = []
        for joint in first_skeleton_msg.joints:
            joint_name = JointUtils.JOINTS[joint.type]
            if joint_name in JointUtils.JOINTS_TO_IGNORE:
                continue

            joint_positions.append(np.array(joint.real, dtype=np.float32) / 1000.)

        self.ske_seq.put(joint_positions)

        if self.ske_seq.full():
            rospy.loginfo("Sequence: {} captured".format(self.seq_cnt))
            self.record = False
            ske_data = np.array([self.ske_seq.get() for i in range(self.ske_seq.qsize())])
            self.prepare_data(ske_data)
            if self.seq_cnt < 25:
               self.seq_cnt += 1
               self.record = True

    def process_ntu_data(self, ske_data_msg):
        if not self.record or not ske_data_msg.skeletons:
            return

#        rospy.loginfo('Recieved Skeleton Data...Processing Now')
        first_skeleton_msg = ske_data_msg.skeletons[0]

        joint_positions = np.zeros((len(ntu_joint_map), 3), dtype=np.float32)
        for joint in first_skeleton_msg.joints:
            if joint.type in ntu_joint_map:
                joint_positions[ntu_joint_map.index(joint.type), :] = np.array(joint.real, dtype=np.float32) 

        self.ske_seq.put(joint_positions)

        if self.ske_seq.full():
            rospy.loginfo("Sequence: {} captured".format(self.seq_cnt))
            self.record = False
            ske_data = np.array([self.ske_seq.get() for i in range(self.ske_seq.qsize())])
            self.prepare_data(ske_data)
            if self.seq_cnt < 25:
               self.seq_cnt += 1
               self.record = True

    def prepare_data(self, ske_data):
        num_frames = ske_data.shape[0]
        ske_joints = np.zeros((num_frames, 19*3), dtype=np.float32)

        ske_joints = ske_data.reshape(-1, 19*3)

        origin = np.copy(ske_joints[0, 3:6])

        for f in range(num_frames):
            ske_joints[f] -= np.tile(origin, 19)

        self.action_data[self.seq_cnt, :num_frames] = np.hstack((ske_joints, np.zeros_like(ske_joints)))

        return
