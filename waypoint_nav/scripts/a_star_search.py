#!/usr/bin/env python
# -*- coding: utf-8 -*-

import yaml
from collections import defaultdict, deque
import rospy
from actionlib import SimpleActionServer, SimpleActionClient
from waypoint_nav.msg import TopoNavAction, TopoNavResult
from geometry_msgs.msg import PoseStamped
from waypoint_nav.msg import WayPointNavAction, WayPointNavGoal
import numpy as np


class AStarSearch(object):

    def __init__(self, name):
        rospy.loginfo("Starting {}".format(name))
        self._as = SimpleActionServer(name, TopoNavAction, self.execute_cb, auto_start=False)
        self._as.register_preempt_callback(self.preempt_cb)
        self.client = SimpleActionClient("/waypoint_nav_node", WayPointNavAction)
        rospy.loginfo("Waiting for nav server")
        self.client.wait_for_server()
        self.nodes = set()
        self.edges = defaultdict(list)
        self.distances = {}

        with open(rospy.get_param("~waypoint_yaml"), 'r') as f:
            self.waypointInfo = yaml.load(f)

        for waypoint, info in self.waypointInfo.items():
            self.add_node(waypoint)
            for edge in info["edges"]:
                self.add_edge(waypoint, edge, 1.0)
            self.waypointInfo[waypoint]["position"]["x"] *= 0.555
            self.waypointInfo[waypoint]["position"]["y"] *= 0.555

        self.sub = rospy.Subscriber("/orb_slam/pose", PoseStamped, self.pose_cb)
        self._as.start()
        rospy.loginfo("Started {}".format(name))

    def preempt_cb(self, *args):
        self.client.cancel_all_goals()

    def add_node(self, value):
        self.nodes.add(value)

    def add_edge(self, from_node, to_node, distance):
        self.edges[from_node].append(to_node)
        # self.edges[to_node].append(from_node)
        self.distances[(from_node, to_node)] = distance

    def pose_cb(self, msg):
        closest = (None, None)
        for k, v in self.waypointInfo.items():
            dist = self.get_dist(msg, v["position"])
            if closest[0] is None or dist < closest[0]:
                closest = (dist, k)
        self.closest = closest[1]

    def get_dist(self, pose1, position):
        return np.sqrt(np.power(pose1.pose.position.x-position["x"], 2)+np.power(pose1.pose.position.y-position["y"], 2))

    def execute_cb(self, goal):
        path = self.shortest_path(self.closest, goal.waypoint)[1]
        print path
        for p in path:
            if not self._as.is_preempt_requested():
                rospy.loginfo("Going to waypoint: {}".format(p))
                target = PoseStamped()
                target.header.stamp = rospy.Time.now()
                target.header.frame_id = "map"
                target.pose.position.x = self.waypointInfo[p]["position"]["x"]
                target.pose.position.y = self.waypointInfo[p]["position"]["y"]
                target.pose.orientation.x = self.waypointInfo[p]["orientation"]["x"]
                target.pose.orientation.y = self.waypointInfo[p]["orientation"]["y"]
                target.pose.orientation.z = self.waypointInfo[p]["orientation"]["z"]
                target.pose.orientation.w = self.waypointInfo[p]["orientation"]["w"]
                self.client.send_goal_and_wait(
                    WayPointNavGoal(target)
                ) 
        if not self._as.is_preempt_requested():
            self._as.set_succeeded(TopoNavResult())
        else:
            self._as.set_preempted()

    def dijkstra(self, initial):
        visited = {initial: 0}
        path = {}

        nodes = set(self.nodes)

        while nodes:
            min_node = None
            for node in nodes:
                if node in visited:
                    if min_node is None:
                        min_node = node
                    elif visited[node] < visited[min_node]:
                        min_node = node
            if min_node is None:
                break

            nodes.remove(min_node)
            current_weight = visited[min_node]

            for edge in self.edges[min_node]:
                weight = current_weight + self.distances[(min_node, edge)]
                if edge not in visited or weight < visited[edge]:
                    visited[edge] = weight
                    path[edge] = min_node

        return visited, path

    def shortest_path(self, origin, destination):
        visited, paths = self.dijkstra(origin)
        full_path = deque()
        _destination = paths[destination]


        while _destination != origin:
            full_path.appendleft(_destination)
            _destination = paths[_destination]

        full_path.appendleft(origin)
        full_path.append(destination)

        return visited[destination], list(full_path)


if __name__ == "__main__":
    rospy.init_node("topo_nav_node")
    a = AStarSearch(rospy.get_name())
    rospy.spin()
