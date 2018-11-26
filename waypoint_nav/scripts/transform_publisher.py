#!/usr/bin/env python
# -*- coding: utf-8 -*-


import rospy
import tf
from std_srvs.srv import Empty, EmptyResponse
from geometry_msgs.msg import Pose
import PyKDL


class TransformPublisher(object):
    def __init__(self, name):
        rospy.loginfo("Starting %s ..." % name)
        self.rate = rospy.get_param("~rate", 30)
        self.listener = tf.TransformListener()
        self.broadcaster = tf.TransformBroadcaster()
        self.trans = tuple()
        self.rot = tuple()
        self.calibrate()
        rospy.Service("~calibrate", Empty, self.calibrate)
        rospy.loginfo("... done")
        
    def calibrate(self, *args):
        rospy.loginfo("Calibrating transform publisher")
        while not rospy.is_shutdown():
            try:
                self.listener.waitForTransform('/camera_link', '/odom', rospy.Time(0), timeout=rospy.Duration(5))
                t = self.listener.getLatestCommonTime('/camera_link', '/odom')
                (trans_odom, rot_odom) = self.listener.lookupTransform('/odom', '/camera_link', t)
            except (tf.Exception, tf.LookupException, tf.ConnectivityException) as ex:
                rospy.logwarn(ex)
            else:
                break
        while not rospy.is_shutdown():
            try:
                self.listener.waitForTransform('/map', '/camera_pose', rospy.Time(0), timeout=rospy.Duration(5))
                t = self.listener.getLatestCommonTime('/map', '/camera_pose')
                (trans_map, rot_map) = self.listener.lookupTransform('/map', '/camera_pose', t)
            except (tf.Exception, tf.LookupException, tf.ConnectivityException) as ex:
                rospy.logwarn(ex)
            else:
                break
        print trans_odom
        print rot_odom
        print "---"
        print trans_map
        print rot_map

        quat_rot_odom = PyKDL.Rotation.Quaternion(rot_odom[0],rot_odom[1],rot_odom[2],rot_odom[3])
        quat_rot_map = PyKDL.Rotation.Quaternion(rot_map[0],rot_map[1],rot_map[2],rot_map[3])
        rot_diff = quat_rot_map * quat_rot_odom.Inverse()
        print rot_diff, type(rot_diff)

        trans_diff = rot_diff * PyKDL.Vector(trans_odom[0], trans_odom[1], trans_odom[2])
        print trans_diff

        rot_diff = rot_diff.GetQuaternion()
        print rot_diff

        self.pose = Pose()
        self.pose.position.x = trans_map[0] - trans_diff[0]
        self.pose.position.y = trans_map[1] - trans_diff[1]
        self.pose.position.z = trans_map[2] - trans_diff[2]
        self.pose.orientation.x = rot_diff[0]
        self.pose.orientation.y = rot_diff[1]
        self.pose.orientation.z = rot_diff[2]
        self.pose.orientation.w = rot_diff[3]
        rospy.loginfo("Calibrated.")
        return EmptyResponse()

    def spin(self):
        r = rospy.Rate(self.rate)
        while not rospy.is_shutdown():
            self.broadcaster.sendTransform(
                (self.pose.position.x, self.pose.position.y, 0.0),
                (self.pose.orientation.x, self.pose.orientation.y, self.pose.orientation.z, self.pose.orientation.w),
                rospy.Time.now(),
                "/odom",
                "/map"
            )
            r.sleep()
            

if __name__ == "__main__":
    rospy.init_node("cozmo_tf")
    t = TransformPublisher(rospy.get_name())
    t.spin()

