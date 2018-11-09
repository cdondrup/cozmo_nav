#!/usr/bin/env python
# -*- coding: utf-8 -*-

import yaml
from collections import defaultdict, deque
import rospy
from actionlib import SimpleActionServer
from waypoint_nav.msg import TopoNavAction, TopoNavResult


class AStarSearch(object):

    def __init__(self, name):
        rospy.loginfo("Starting {}".format(name))
        self._as = SimpleActionServer(name, TopoNavAction, self.execute_cb, auto_start=False)
        self.nodes = set()
        self.edges = defaultdict(list)
        self.distances = {}

        with open(rospy.get_param("~waypoint_yaml"), 'r') as f:
            self.waypointInfo = yaml.load(f)

        for waypoint, info in self.waypointInfo.items():
            self.add_node(waypoint)
            for edge in info["edges"]:
                self.add_edge(waypoint, edge, 1.0)

        self._as.start()
        rospy.loginfo("Started {}".format(name))

    def add_node(self, value):
        self.nodes.add(value)

    def add_edge(self, from_node, to_node, distance):
        self.edges[from_node].append(to_node)
        # self.edges[to_node].append(from_node)
        self.distances[(from_node, to_node)] = distance

    def execute_cb(self, goal):
        path = self.shortest_path("waypoint1", goal.waypoint)
        print path
        self._as.set_succeeded(TopoNavResult())

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
