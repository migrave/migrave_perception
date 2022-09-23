#!/usr/bin/env python3

import rospy
import numpy as np
from queue import Queue

from qt_nuitrack_app.msg import Skeletons
from std_msgs.msg import String
from migrave_skeleton_tools_ros.skeleton_utils import JointUtils

ntu_joint_map = [3, 2, 1, 0, 20, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 16, 17, 18]

class ActionLearner(object):
    def __init__(self, action_model, nuitrack_skeleton_topic="/qt_nuitrack_app/skeletons"):

        self.model = action_model

        self.ske_seq = Queue(maxsize = 80)
        self.seq_cnt = 0

        self.ske_sub = rospy.Subscriber(nuitrack_skeleton_topic, Skeletons, self.process_nt_data)
        self.record = False

        self.qt_speech = rospy.Publisher("/qt_robot/speech/say", String, queue_size = 5)

    def learn(self, action):
        skes_data = []

        rospy.loginfo('Sleeping for 3 seconds; get into position!')
        self.qt_speech.publish('Get into position!')
        rospy.sleep(3.)

        while self.seq_cnt < 25:
            rospy.loginfo('Capturing data...')
            #self.qt_speech.publish('Capturing action sequence!')
            self.record = True

            while not self.ske_seq.full():
                continue

            rospy.loginfo('Sequence: {} captured'.format(self.seq_cnt))
            #self.qt_speech.publish('Sequence: {} captured'.format(self.seq_cnt))
            self.record = False
            skes_data.append(np.array([self.ske_seq.get() for i in range(self.ske_seq.qsize())]))
            self.seq_cnt += 1
            input("Press Enter to continue....")
            rospy.sleep(1.)

        rospy.loginfo('Preparing Data...')
        trn_data, val_data = self.prepare_data(skes_data)

        self.model.train(action, trn_data, val_data)

        rospy.loginfo('Saving new model...')
        #self.model.save_model(action)

        rospy.loginfo('Learning Complete!...')
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

    def prepare_data(self, skes_data):
        action_data = np.zeros((len(skes_data), 80, 19*6), dtype=np.float32)

        for idx, ske_data in enumerate(skes_data):
            num_frames = ske_data.shape[0]
            ske_joints = np.zeros((num_frames, 19*3), dtype=np.float32)

            ske_joints = ske_data.reshape(-1, 19*3)

            origin = np.copy(ske_joints[0, 3:6])

            for f in range(num_frames):
                ske_joints[f] -= np.tile(origin, 19)

            action_data[idx, :num_frames] = np.hstack((ske_joints, np.zeros_like(ske_joints)))

        N, T, J = action_data.shape
        x = action_data.reshape((N, T, 2, int(J/6), 3)).transpose(0, 4, 1, 3, 2)

        trn_data = {'x': x[:20], 'y': [len(self.model.actions)]*20}
        val_data = {'x': x[20:25], 'y': [len(self.model.actions)]*5}

#        for idx, ske in enumerate(x):
#            if idx < 20:
#                trn_data['x'].append(ske)
#                trn_data['y'].append(len(self.model.actions))
#            else:
#                val_data['x'].append(ske)
#                val_data['y'].append(len(self.model.actions))

        return trn_data, val_data
