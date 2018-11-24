#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from actionlib import SimpleActionClient
from waypoint_nav.msg import TopoNavAction, TopoNavGoal
import yaml
from ar_pose.msg import ARMarkers

class Server(object):
    def __init__(self, name):
        rospy.loginfo("Starting '{test}'.".format(test=name))
        self.client = SimpleActionClient("/topo_nav_node", TopoNavAction)
        self.client.wait_for_server()
        with open(rospy.get_param("~goals_yaml"), "r") as f:
            self.goals = yaml.load(f)
        print self.goals
        self.cnt = {}
        self.active_id = -1
        self.sub = rospy.Subscriber("/ar_pose_marker", ARMarkers, self.pose_cb)
        rospy.loginfo("Started '{test}'.".format(test=name))

    def pose_cb(self, msg):
        ids = []
        for m in msg.markers:
            ids.append(m.id)
            if m.id in self.cnt:
                self.cnt[m.id] += 1
            else:
                self.cnt[m.id] = 1
        print self.cnt

        for k in self.cnt.keys():
            if k not in ids:
                self.cnt[k] = 0
            if self.cnt[k] > 5:
                if self.active_id != k:
                     print "Going to:", self.goals[k]
                     self.client.send_goal(TopoNavGoal(self.goals[k]))
                     self.active_id = k
                     self.cnt[k] = 0


if __name__ == "__main__":
    rospy.init_node("marker_nav_node")
    s = Server(rospy.get_name())
    rospy.spin()
