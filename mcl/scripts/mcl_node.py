#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import PoseStamped, PoseArray, Pose
from nav_msgs.msg import Odometry
from mcl_pf.particle_filter import ParticleFilter
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from std_srvs.srv import Empty, EmptyResponse
import numpy as np
import message_filters


class MCL(object):
    def __init__(self, name):
        rospy.loginfo("Starting '{}' ...".format(name))
        self.pf = ParticleFilter(num_particles=500)
        self.subs = []
        self.prev_pose = None
        self.prev_odom = None
        self.particle_pub = rospy.Publisher("/particle_cloud", PoseArray, queue_size=1)
        self.pose_pub = rospy.Publisher("/robot_pose", PoseStamped, queue_size=1)
        self.srv = rospy.Service("~init_pose", Empty, self.init_srv_cb)
        rospy.loginfo("Started '{}' ...".format(name))

    def init_srv_cb(self, srv):
        msg = rospy.wait_for_message("/orb_slam/pose", PoseStamped)
        self.publish_pose_arry(*self.pf.initialise(msg))
        self.subs.append(message_filters.Subscriber("/orb_slam/pose", PoseStamped))
        self.subs.append(message_filters.Subscriber("/odom", Odometry))
        self.ts = message_filters.ApproximateTimeSynchronizer(self.subs, 1, 0.1)
        self.ts.registerCallback(self.pose_cb)
        return EmptyResponse()

    def test(self):
        while not rospy.is_shutdown():
            rospy.sleep(0.2)
            self.publish_pose_arry(*self.pf.update(0.1, 0.1, 0.3))


    def publish_pose_arry(self, px, py, pt):
        pa = PoseArray()
        pa.header.stamp = rospy.Time.now()
        pa.header.frame_id = "map"
        for x, y, t in zip(px, py, pt):
            p = Pose()
            p.position.x = x
            p.position.y = y
            q = quaternion_from_euler(0.0, 0.0, t)
            p.orientation.z = q[2]
            p.orientation.w = q[3]
            pa.poses.append(p)

        self.particle_pub.publish(pa)

    def pose_cb(self, pose, odom):
        if self.prev_odom is None:
            self.prev_odom = odom.pose
            return
        dx, dy = self.get_trans_diff(odom.pose, self.prev_odom)
        dt = self.get_rot_diff(odom.pose, self.prev_odom)
        self.pf.update(dx, dy, dt)
        self.prev_odom = odom.pose

        self.pf.observe(pose.pose.position.x, pose.pose.position.y, self.get_theta(self.get_quat_from_pose(pose)), pose_sigma=0.5, rot_sigma=0.6)
        x, y, t = self.pf.resample(pose)
        self.publish_pose_arry(x, y, t)
        p = PoseStamped()
        p.header.frame_id = "map"
        p.header.stamp = rospy.Time.now()
        p.pose.position.x = np.mean(x)
        p.pose.position.y = np.mean(y)
        print np.mean(t)
        q = quaternion_from_euler(0.0, 0.0, np.mean(t))
        p.pose.orientation.z = q[2]
        p.pose.orientation.w = q[3]
        self.pose_pub.publish(p)

    def get_trans_diff(self, pose1, pose2):
        dx = pose1.pose.position.x - pose2.pose.position.x
        dy = pose1.pose.position.y - pose2.pose.position.y
        return dx, dy

    def get_rot_diff(self, pose1, pose2):
        return self.get_theta(self.get_quat_from_pose(pose1)) - self.get_theta(self.get_quat_from_pose(pose2))

    def get_quat_from_pose(self, pose):
        return (pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w)

    def get_theta(self, quat):
        _, _, theta = euler_from_quaternion(quat)
        return theta


if __name__ == "__main__":
    rospy.init_node("cozmo_localisation")
    m = MCL(rospy.get_name())
    rospy.spin()

