#!/usr/bin/env python
# -*- coding: utf-8 -*-


import rospy
import tf
from std_srvs.srv import Empty, EmptyResponse
from geometry_msgs.msg import Pose


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
                self.listener.waitForTransform('/base_link', '/odom', rospy.Time(0), timeout=rospy.Duration(5))
                t = self.listener.getLatestCommonTime('/base_link', '/odom')
                (trans, rot) = self.listener.lookupTransform('/odom', '/base_link', t)
            except (tf.Exception, tf.LookupException, tf.ConnectivityException) as ex:
                rospy.logwarn(ex)
            else:
                break
        self.pose = Pose()
        self.pose.position.x = trans[0]
        self.pose.position.y = trans[1]
        self.pose.position.z = trans[2]
        self.pose.orientation.x = rot[0]
        self.pose.orientation.y = rot[1]
        self.pose.orientation.z = rot[2]
        self.pose.orientation.w = rot[3]
        rospy.loginfo("Calibrated.")
        return EmptyResponse()

    def spin(self):
        r = rospy.Rate(self.rate)
        while not rospy.is_shutdown():
            self.broadcaster.sendTransform(
                (self.pose.position.x, self.pose.position.y, 0.0),
                (self.pose.orientation.x, self.pose.orientation.y, self.pose.orientation.z, self.pose.orientation.w),
                rospy.Time.now(),
                "/map",
                "/odom"
            )
            r.sleep()
            

if __name__ == "__main__":
    rospy.init_node("cozmo_tf")
    t = TransformPublisher(rospy.get_name())
    t.spin()

