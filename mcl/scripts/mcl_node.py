#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import PoseStamped, PoseArray, Pose
from nav_msgs.msg import Odometry
from mcl_pf.particle_filter import ParticleFilter
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from std_srvs.srv import Empty, EmptyResponse
import numpy as np
import scipy.stats
from threading import Thread, Lock


class MCL(object):
    def __init__(self, name):
        rospy.loginfo("Starting '{}' ...".format(name))
        # Creating a new instance of the particle filter setting num particles
        self.pf = ParticleFilter(num_particles=500)

        # Setting up member variables
        self.mutex = Lock()
        self.thread = None
        self.is_running = False
        self.subs = []
        self.prev_pose = None
        self.prev_odom = None

        # Creating ros publishers for pose_array and robot pose
        self.particle_pub = rospy.Publisher("/particle_cloud", PoseArray, queue_size=1)
        self.pose_pub = rospy.Publisher("/robot_pose", PoseStamped, queue_size=1)

        # Creating init service to start particle filter and calling it when starting the node
        self.srv = rospy.Service("~init_pose", Empty, self.init_srv_cb)
        self.init_srv_cb(None)
        rospy.loginfo("Started '{}' ...".format(name))

    def init_srv_cb(self, srv):
        """The init service call back triggered when the service is called."""

        # Waiting for a message from orb slam for the initial pose
        msg = rospy.wait_for_message("/orb_slam/pose", PoseStamped)

        # Initialising the pf with the received pose
        self.publish_pose_arry(*self.pf.initialise(msg))

        # Stopping and starting the prediction thread
        self.stop_pf()
        self.start_pf()
        return EmptyResponse()

    def start_pf(self):
        """Starts the prediction"""

        # Create subscribers to odometry and orb slam
        if not self.subs:
            self.subs.append(rospy.Subscriber("/orb_slam/pose", PoseStamped, self.pose_cb))
            self.subs.append(rospy.Subscriber("/odom", Odometry, self.odom_cb))

        # Start the prediction thread
        self.is_running = True
        self.thread = Thread(target=self.spin)
        self.thread.start()

    def stop_pf(self):
        """Stops the prediction"""

        # Unregister all subscribers if there are any
        if self.subs:
            for s in self.subs:
                s.unregister()
            self.subs = []

        # Resetting variables
        self.prev_odom = None
        self.prev_pose = None

        # Stop prediction thread if it is running
        self.is_running = False
        try:
            self.thread.join()
        except AttributeError:
            pass

    def publish_pose_arry(self, px, py, pt):
        """Publish a pose array for visualisation in rviz."""

        # Create new message
        pa = PoseArray()
        pa.header.stamp = rospy.Time.now()
        pa.header.frame_id = "map"

        # Create a pose for every particle and add it the array
        for x, y, t in zip(px, py, pt):
            p = Pose()
            p.position.x = x
            p.position.y = y
            q = quaternion_from_euler(0.0, 0.0, t)
            p.orientation.z = q[2]
            p.orientation.w = q[3]
            pa.poses.append(p)

        # Publish message
        self.particle_pub.publish(pa)

    def odom_cb(self, odom):
        """Function called whenever a new odometry message is received"""

        # If this is the first message ever received save it and return
        if self.prev_odom is None:
            self.prev_odom = odom
            return

        # get the translational difference between the current and last odomoetry message
        dx, dy = self.get_trans_diff(odom.pose, self.prev_odom.pose)
        # get the difference in rotation
        da = self.get_rot_diff(odom.pose, self.prev_odom.pose)
        # get the time increment since the last message
        dt = odom.header.stamp.to_sec() - self.prev_odom.header.stamp.to_sec()

        # Update the particle filter
        with self.mutex:
            self.pf.update(dx, dy, da, dt)

        # update last odom received
        self.prev_odom = odom

    def spin(self):
        """Prediction thread function"""

        # Set the publishing rate and the current time
        r = rospy.Rate(30)
        t = rospy.Time().now().to_sec()

        # Prediction loop
        while not rospy.is_shutdown() and self.is_running:
            with self.mutex:
                # Predict new postion of particles assuming contant velocity
                now = rospy.Time().now().to_sec()
                x, y, a = self.pf.predict(now - t)

            # Publish results
            self.publish_pose_arry(x, y, a)
            # Use the mean values of all particles to calculate current robot position
            self.publish_pose(np.mean(x), np.mean(y), np.mean(a))

            # Update last time and ensure publishing rate
            t = now
            r.sleep()

    def pose_cb(self, pose):
        """Function called whenever a new orb slam pose is received"""

        with self.mutex:
            # Make a new observation
            self.pf.observe(pose.pose.position.x, pose.pose.position.y, self.get_theta(self.get_quat_from_pose(pose)), pose_sigma=0.1, rot_sigma=0.2)
            # Resample particles based on new weights
            self.pf.resample(pose_sigma=0.03, rot_sigma=0.02, starvation_factor=0.2)

    def publish_pose(self, x, y, a):
        """Publish the postion of the robot"""

        p = PoseStamped()
        p.header.frame_id = "map"
        p.header.stamp = rospy.Time.now()
        p.pose.position.x = x
        p.pose.position.y = y
        q = quaternion_from_euler(0.0, 0.0, a)
        p.pose.orientation.z = q[2]
        p.pose.orientation.w = q[3]
        self.pose_pub.publish(p)

    def get_trans_diff(self, pose1, pose2):
        """Calculate the difference in location between two poses."""

        dx = pose1.pose.position.x - pose2.pose.position.x
        dy = pose1.pose.position.y - pose2.pose.position.y
        return dx, dy

    def get_rot_diff(self, pose1, pose2):
        """Calculate the difference in rotation between two poses and return them a single angle."""

        diff = self.get_theta(self.get_quat_from_pose(pose1)) - self.get_theta(self.get_quat_from_pose(pose2))
        diff = diff-(2*np.pi) if diff > np.pi else diff+(2*np.pi) if diff < -np.pi else diff
        return diff

    def get_quat_from_pose(self, pose):
        """Create a quaternion from a pose."""

        return (pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w)

    def get_theta(self, quat):
        """Get the yaw rotation (theta) from a quaternion"""

        _, _, theta = euler_from_quaternion(quat)
        return theta


if __name__ == "__main__":
    rospy.init_node("cozmo_localisation")
    m = MCL(rospy.get_name())
    rospy.spin()

