#!/usr/bin/env python3

from queue import Queue
from collections import deque
import numpy as np

import rospy

from qt_nuitrack_app.msg import Skeletons
from migrave_skeleton_tools_ros.skeleton_utils import JointUtils

from CTRGCN.data.ntu.get_raw_denoised_data import get_one_actor_points, get_two_actors_points, remove_missing_frames
from CTRGCN.data.ntu.seq_transformation import seq_translation, align_frames

ntu_joint_map = [3, 2, 1, 0, 20, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 16, 17, 18]

class ActionClassifier(object):
    def __init__(self, action_model, nuitrack_skeleton_topic="/qt_nuitrack_app/skeletons", camera_frame="Camera_link"):

        self.cam_frame = camera_frame

        self.model = action_model
        self.ske_data = np.array([])

        self.classify = False
        self.record = True
        #self.ske_seq = Queue(maxsize = 80)
        self.ske_seq = deque(maxlen = 80)
        self.seq_cnt = 0

        self.ske_sub = rospy.Subscriber(nuitrack_skeleton_topic, Skeletons, self.process_nt_data)

    def classify_action(self):
        if self.ske_data.shape[0] == 80:
            rospy.loginfo('Recognizing action')
            self.ske_data = self.prepare_data(self.ske_data)
            action_type = self.model.classify(self.ske_data)
            self.ske_data = np.array([])

            return action_type
        else:
            return None

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

        self.ske_seq.append(joint_positions)
        self.seq_cnt += 1
        
        if self.seq_cnt == 80:
            rospy.loginfo("80 frames captured")
            #self.ske_data = np.array([self.ske_seq.get() for i in range(80)])
            self.ske_data = np.array(list(self.ske_seq))
            print(self.ske_data.shape)
            self.seq_cnt = 0

    def process_ntu_data(self, ske_data_msg):
        rospy.loginfo('Recieved Skeleton Data...Processing Now')

        if len(ske_data_msg.skeletons) == 0:
            num_frames = self.ske_data['num_frames']
            num_bodies = len(self.ske_data['data'])
            num_joints = 19

            if num_bodies == 1:  # only 1 actor
                body_data = list(self.ske_data['data'].values())[0]
                joints, colors = get_one_actor_points(body_data, num_frames, num_joints)
            else:  # more than 1 actor, select two main actors
                joints, colors = get_two_actors_points(self.ske_data, num_joints)
                # Remove missing frames
                joints, colors = remove_missing_frames(ske_name, joints, colors)
                num_frames = joints.shape[0]  # Update

            seq_joints = seq_translation([joints], 19)
            seq_joints = align_frames(seq_joints, np.array([num_frames]), 19)

            self.ske_seq = seq_joints
            self.classify = True

            return

        self.ske_data['num_frames'] += 1
        bodies_data = self.ske_data['data']
        joints = np.zeros((len(ske_data_msg.skeletons), len(joint_map), 3), dtype=np.float32)
        colors = np.zeros((len(ske_data_msg.skeletons), len(joint_map), 2), dtype=np.float32)

        for b, skeleton in enumerate(ske_data_msg.skeletons):
            body_id = str(skeleton.id)
            valid_frames = int(skeleton.joints[0].confidence)

            for joint in skeleton.joints:
                if joint.type in ntu_joint_map:
                    joints[b, joint_map.index(joint.type), :] = np.array(joint.real, dtype=np.float32)
                    colors[b, joint_map.index(joint.type), :] = np.array(joint.projection, dtype=np.float32)

            if body_id not in bodies_data:  # Add a new body's data
                body_data = dict()
                body_data['joints'] = joints[b]  # ndarray: (25, 3)
                body_data['colors'] = colors[b, np.newaxis]  # ndarray: (1, 25, 2)
                body_data['interval'] = [valid_frames]  # the index of the first frame
            else:  # Update an already existed body's data
                body_data = bodies_data[body_id]
                # Stack each body's data of each frame along the frame order
                body_data['joints'] = np.vstack((body_data['joints'], joints[b]))
                body_data['colors'] = np.vstack((body_data['colors'], colors[b, np.newaxis]))
                pre_frame_idx = body_data['interval'][-1]
                body_data['interval'].append(pre_frame_idx + 1)

            bodies_data[body_id] = body_data

        self.ske_data['data'] = bodies_data

    def prepare_data(self, ske_data):
        ske_joints = np.zeros((80, 19*3), dtype=np.float32)

        ske_joints = ske_data.reshape(-1, 19*3)

        origin = np.copy(ske_joints[0, 6:9])

        for f in range(80):
            ske_joints[f] -= np.tile(origin, 19)

        processed_ske_data = np.zeros((1, 80, 19*6), dtype=np.float32)
        processed_ske_data[0, :80] = np.hstack((ske_joints, np.zeros_like(ske_joints)))

        return processed_ske_data
