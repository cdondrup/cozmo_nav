#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import PoseWithCovariance, PoseStamped, PoseArray, Pose
from nav_msgs.msg import Odometry
from mcl_pf.particle_filter import ParticleFilter
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from std_srvs.srv import Empty, EmptyResponse
import numpy as np


class TestMCL(object):
    def __init__(self, name):
        rospy.loginfo("Starting '{}' ...".format(name))
        self.pf = ParticleFilter()
        self.robot_pose_pub = rospy.Publisher("/orb_slam/pose", PoseStamped, queue_size=1)
        self.odom_pub = rospy.Publisher("/odom", Odometry, queue_size=1)
        rospy.loginfo("Started '{}' ...".format(name))

    def spin(self):
        r = 1.
        t = 0
        ps = PoseStamped()
        ps.header.frame_id = "map"
        o = Odometry()
        o.header.frame_id = "map"
        while not rospy.is_shutdown():
            ps.header.stamp = rospy.Time.now()
            o.header.stamp = rospy.Time.now()
            x = r*np.sin(t)
            y = r*np.cos(t)
            t -= 0.01
            rospy.sleep(0.03)
            pc = PoseWithCovariance()
            ps.pose.position.x = x + np.random.uniform(-0.05, 0.05)
            ps.pose.position.y = y + np.random.uniform(-0.05, 0.05)
            pc.pose.position.x = x + np.random.uniform(-0.05, 0.05)
            pc.pose.position.y = y + np.random.uniform(-0.05, 0.05)
            q = quaternion_from_euler(0.0, 0.0, -t+np.pi + np.random.uniform(-0.07, 0.07))
            ps.pose.orientation.z = q[2]
            ps.pose.orientation.w = q[3]
            q = quaternion_from_euler(0.0, 0.0, -t+np.pi + np.random.uniform(-0.07, 0.07))
            pc.pose.orientation.z = q[2]
            pc.pose.orientation.w = q[3]
            self.robot_pose_pub.publish(ps)
            o.pose = pc
            self.odom_pub.publish(o)

if __name__ == "__main__":
    rospy.init_node("test_mcl_node")
    m = TestMCL(rospy.get_name())
    m.spin()

