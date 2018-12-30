#!/usr/bin/env python
import os
import copy
import math
import xmlrpclib
import numpy as np

import rospy
import roslib
import tf
from geometry_msgs.msg import PoseStamped
from arl_hw_msgs.msg import MusculatureState
from arl_hand_tracker_msgs.msg import TrainingData


class Publisher:
    def __init__(self, name):
        rospy.init_node(name, anonymous=True)
        self._name = name
        self._ros_master = xmlrpclib.ServerProxy(os.environ['ROS_MASTER_URI'])

        rospy.Subscriber('/musculature/state', MusculatureState, self._state_cb)
        self._last_musculature_state = MusculatureState()

        self._hand_pose_publisher = rospy.Publisher('/hand_pose', PoseStamped, queue_size=10)
        self._training_data_publisher = rospy.Publisher('/training/state_data', TrainingData, queue_size=10)

        self._tf_listener = None
        self._tf_broadcaster = tf.TransformBroadcaster()

    def _state_cb(self, data):
        self._last_musculature_state = data

    def run(self):
        self._tf_listener = tf.TransformListener()

        rate = rospy.Rate(100)
        static_pose = None

        while not rospy.is_shutdown():

            # get reference transform once
            if static_pose is None:
                (static_pose, success) = self._get_transform('/camera_depth_frame', '/red_marker_frame')
                if not success:
                    static_pose = None
            else:
                self._tf_broadcaster.sendTransform((static_pose.pose.position.x, static_pose.pose.position.y,
                                                    static_pose.pose.position.z), (static_pose.pose.orientation.x,
                                                                                   static_pose.pose.orientation.y,
                                                                                   static_pose.pose.orientation.z,
                                                                                   static_pose.pose.orientation.w),
                                                   rospy.Time.now(), '/base_link', '/camera_depth_frame')

            # publish training data
            data = TrainingData()
            data.header.stamp = rospy.get_rostime()
            data.header.frame_id = '0'
            data.musculature_state = self._last_musculature_state

            (pose_msg, success) = self._get_transform('/base_link', '/yellow_marker_frame')
            if success:
                self._hand_pose_publisher.publish(pose_msg)
                data.hand_pose = pose_msg
                self._training_data_publisher.publish(data)

            (pose_msg, success) = self._get_transform('/base_link', '/blue_marker_frame')
            if success:
                data.elbow_pose = pose_msg
                self._training_data_publisher.publish(data)

            (pose_msg, success) = self._get_transform('/base_link', '/green_marker_frame')
            if success:
                data.shoulder_pose = pose_msg
                self._training_data_publisher.publish(data)

            rate.sleep()

    def _get_transform(self, source, destination):
        pose_msg = PoseStamped()
        success = False
        try:
            (trans, quat) = self._tf_listener.lookupTransform(source, destination, rospy.Time(0))
            pose_msg.header.stamp = rospy.get_rostime()
            pose_msg.header.frame_id = '0'
            pose_msg.pose.position.x = trans[0]
            pose_msg.pose.position.y = trans[1]
            pose_msg.pose.position.z = trans[2]
            pose_msg.pose.orientation.x = quat[0]
            pose_msg.pose.orientation.y = quat[1]
            pose_msg.pose.orientation.z = quat[2]
            pose_msg.pose.orientation.w = quat[3]
            success = True

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            pass

        return pose_msg, success


if __name__ == '__main__':
    try:
        publisher = Publisher('hand_position_publisher_node')
        publisher.run()
    except rospy.ROSInterruptException:
        pass
