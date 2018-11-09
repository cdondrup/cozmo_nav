#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from actionlib import SimpleActionServer
from waypoint_nav.msg import WayPointNavAction, WayPointNavResult
from geometry_msgs.msg import PoseStamped, Twist
import numpy as np
import tf


class Server(object):
    def __init__(self, name):
        rospy.loginfo("Starting '{test}'.".format(test=name))
        self._as = SimpleActionServer(name, WayPointNavAction, auto_start=False)
        self._as.register_goal_callback(self.execute_cb)
        self.listener = tf.TransformListener()
        self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        self.sub = None
        self._as.start()
        rospy.loginfo("Started '{test}'.".format(test=name))

    def execute_cb(self):
        self.goal = self._as.accept_new_goal()
        print self.goal
        self.sub = rospy.Subscriber("/orb_slam/pose", PoseStamped, self.pose_cb)

    def pose_cb(self, msg):
        dist = self.get_dist(self.goal.goal, msg)
        t = Twist()
        new_goal = self._transform(self.goal.goal, "camera_pose")
        print new_goal
        print "Dist", dist
        theta = self.get_theta(new_goal.pose)
        print "Theta", theta
        if dist < 0.05:
            self._as.set_succeeded(WayPointNavResult())
            self.sub.unregister()
            self.sub = None
            t.linear.x = 0.0
            t.angular.z = 0.0
        else:
            t.linear.x = 0.03
            t.angular.z = theta * .6
        
        self.pub.publish(t)

    def get_dist(self, pose1, pose2):
        return np.sqrt(np.power(pose1.pose.position.x-pose2.pose.position.x, 2)+np.power(pose1.pose.position.y-pose2.pose.position.y, 2))

    def _transform(self, msg, target_frame):
        if msg.header.frame_id != target_frame:
            try:
                t = self.listener.getLatestCommonTime(target_frame, msg.header.frame_id)
                msg.header.stamp = t
                return self.listener.transformPose(target_frame, msg)
            except (tf.Exception, tf.LookupException, tf.ConnectivityException) as ex:
                rospy.logwarn(ex)
                return None
        else:
            return msg

    def get_theta(self, pose):
        return np.arctan2(pose.position.y, pose.position.x)
        

if __name__ == "__main__":
    rospy.init_node("waypoint_nav_node")
    s = Server(rospy.get_name())
    rospy.spin()
