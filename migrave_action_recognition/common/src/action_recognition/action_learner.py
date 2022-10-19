#!/usr/bin/env python3

import rospy
import numpy as np
from queue import Queue

from qt_nuitrack_app.msg import Skeletons
from std_msgs.msg import String
from migrave_skeleton_tools_ros.skeleton_utils import JointUtils

ntu_joint_map = [3, 2, 1, 0, 20, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 16, 17, 18]

class ActionLearner(object):
    def __init__(self, action_model, save_path, nuitrack_skeleton_topic="/qt_nuitrack_app/skeletons", num_seq=25, seq_size=50):

        self.model = action_model
        self.save_path = save_path

        self.ske_seq = Queue(maxsize = seq_size)
        self.num_seq = num_seq
        self.seq_size = seq_size
        self.seq_cnt = 0

        self.ske_sub = rospy.Subscriber(nuitrack_skeleton_topic, Skeletons, self.process_nt_data)
        self.record = False

        self.qt_speech = rospy.Publisher("/qt_robot/speech/say", String, queue_size = 5)

    def learn(self, action):
        skes_data = []
        num_actions = action.num_actions

        rospy.loginfo('Sleeping for 3 seconds; get into position!')
        #self.qt_speech.publish('Get into position!')
        rospy.sleep(3.)

        while self.seq_cnt < self.num_seq*num_actions:
            rospy.loginfo('Capturing data...')
            #self.qt_speech.publish('Capturing action sequence!')
            self.record = True

            while not self.ske_seq.full():
                continue

            rospy.loginfo('Sequence: {} captured'.format(self.seq_cnt+1))
            #self.qt_speech.publish('Sequence: {} captured'.format(self.seq_cnt+1))
            self.record = False
            skes_data.append(np.array([self.ske_seq.get() for i in range(self.ske_seq.qsize())]))
            self.seq_cnt += 1
            #self.qt_speech.publish('Get into Position!')
            rospy.sleep(2.)

        rospy.loginfo('Preparing Data...')
        trn_data, val_data = self.prepare_data(skes_data, num_actions)
        np.savez(self.save_path + '/test.npz', x_train=trn_data['x'], y_train=trn_data['y'], x_test=val_data['x'], y_test=val_data['y'])
        return True
        self.model.train(action, trn_data, val_data)

        rospy.loginfo('Saving new model...')
        self.model.save_model(action)

        rospy.loginfo('Learning Complete!...')
        return True

    def process_nt_data(self, ske_data_msg):
        if not self.record or not ske_data_msg.skeletons:
            return

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

    def prepare_data(self, skes_data, num_actions):
        n_train = int(self.num_seq*.8)
        n_val = int(self.num_seq*.2)
        y_train = []
        y_val = []

        learned_data = np.zeros((self.num_seq*num_actions, self.seq_size, 19*6), dtype=np.float32)
        train_data = np.zeros((n_train*num_actions, self.seq_size, 19*6), dtype=np.float32)
        val_data = np.zeros((n_val*num_actions, self.seq_size, 19*6), dtype=np.float32)

        for idx, ske_data in enumerate(skes_data):
            num_frames = ske_data.shape[0]
            ske_joints = np.zeros((num_frames, 19*3), dtype=np.float32)

            ske_joints = ske_data.reshape(-1, 19*3)

            origin = np.copy(ske_joints[0, 6:9])

            for f in range(num_frames):
                ske_joints[f] -= np.tile(origin, 19)

            learned_data[idx, :num_frames] = np.hstack((ske_joints, np.zeros_like(ske_joints)))

        for i in range(num_actions):
            train_data[n_train*i:n_train*(i+1)] = learned_data[self.num_seq*i:(self.num_seq*i + n_train)]
            y_train.extend([len(self.model.actions)+i]*n_train)
            val_data[n_val*i:n_val*(i+1)] = learned_data[(self.num_seq*i + n_train):self.num_seq*(i+1)]
            y_val.extend([len(self.model.actions)+i]*n_val)

        N, T, J = train_data.shape
        x_train = train_data.reshape((N, T, 2, int(J/6), 3)).transpose(0, 4, 1, 3, 2)

        N, T, J = val_data.shape
        x_val = val_data.reshape((N, T, 2, int(J/6), 3)).transpose(0, 4, 1, 3, 2)

        trn_data = {'x': x_train, 'y': y_train}
        val_data = {'x': x_val, 'y': y_val}

        return trn_data, val_data
