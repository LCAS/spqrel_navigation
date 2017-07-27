#!/usr/bin/env python

import time
import signal
import sys
import argparse
import math
from random import randrange
from time import sleep

#import topological_map
from topological_map import TopologicalMap
from route_search import TopologicalRouteSearch, get_node, get_edge_from_id
from threading import Timer
from naoqi import *


def get_distance_node_pose(node, pose):
    """
    get_distance_node_pose

    Returns the straight line distance between a pose and a node.
    """
    return math.hypot(
        (pose[0] - node.pose.position.x),
        (pose[1] - node.pose.position.y))


# # create python module
# class MyModule(ALModule):
#     """python class MyModule test auto documentation : comment needed to create a new python module"""

#     def nav_goal_callback(self, str_var_name, value):
#         """callback when data change"""
#         print "New goal", str_var_name, " ", value, " "
#         global goal_check
#         goal_check = 1

#     def goalreached_callback(self, str_var_name, value):
#         print "GOAL REACHED: ", str_var_name, " ", value, " "

#     def get_plan_callback(self, str_var_name, value):
#         print "New plan", str_var_name, " ", value, " "
#         global get_plan
#         get_plan = 1

#     def status_callback(self, str_var_name, value):
#         """callback when data change"""
#         print "status", str_var_name, " ", value, " "
#         global goal_reached
#         goal_reached = 1


class TopologicalLocaliser(object):
    def __init__(self, pip, pport, topomap, fake=False):

        connection_url = "tcp://" + pip + ":" + str(pport)
        app = qi.Application(
            ["TopologicalNavigation", "--qi-url=" + connection_url]
        )
        app.start()
        session = app.session
        self.memProxy = session.service("ALMemory")
        self.goal_check = False
        self.goal_reached = False
        self._max_retries = 10

        self._sub_navgaol = self.memProxy.subscriber(
            "TopologicalNav/Goal"
        )
        self._sub_navgaol.signal.connect(self._on_goal)

        self._sub_plan = self.memProxy.subscriber(
            "TopologicalNav/GetPlan"
        )
        self._sub_plan.signal.connect(self._on_get_plan)

        self._sub_goal_reached = self.memProxy.subscriber(
            "NAOqiPlanner/GoalReached"
        )
        self._sub_goal_reached.signal.connect(self._on_qiplanner_goal_reached)

        self._sub_status = self.memProxy.subscriber(
            "NAOqiPlanner/Status"
        )
        self._sub_status.signal.connect(self._on_qiplanner_status)

        print 'subscribed'

        self.move_base_actions = ['NAOqiPlanner/Goal']
        self.__fake = fake
        self.__fake_node = 'start'
        self.closest_node = 'start'
        self.current_node = 'start'
        # If it fails reaching one goal this variable stores the node it was trying to reach when failed
        self.failed_to = 'none'
        self.fail_code = 0
        self.cancelled = False
        self.navigation_activated = False
        self.map = TopologicalMap(filename=topomap)
        self.loc_timer = Timer(0.5, self._localisation_timer)
        self.loc_timer.start()
        # self.nav_timer = Timer(1.0, self._nav_timer)
        # self.nav_timer.start()
        self._last_status = 'UNKNOWN'
        self.goal_active = False
        signal.signal(signal.SIGINT, self._on_shutdown)
        signal.pause()

    def _on_goal(self, goal):
        print('on_goal: %s' % goal)
        print "NEW GOAL " + goal
        self._last_status = 'UNKNOWN'

        print 'waiting for current goal to be cancelled'
        while (self.goal_active):
            self.goal_reached = False
            self.cancelled = True
            sleep(.5)
        print 'it is cancelled'
        self.memProxy.raiseEvent("NAOqiPlanner/Reset", True)

        if goal == '':
            # empty goal means abort! So nothing to be done as we cancelled above
            pass
        else:
            self.goal_active = True
            self.goal_reached = False
            self.cancelled = False
            self.navigate(goal)

            self.goal_active = False

    def _on_get_plan(self, data):
        print('on_get_plan: %s' % data)
        route = self.get_route(goal)
        plan = []
        for i in range(len(route.source)):
            step = {}
            cn = get_node(self.map, route.source[i])
            step['from'] = route.source[i]
            for j in cn.edges:
                if j.edge_id == route.edge_id[i]:
                    step['edge_id'] = j.edge_id
                    step['action'] = j.action
                    step['dest'] = {}
                    step['dest']['node'] = j.node
                    nn = get_node(self.map, j.node)
                    step['dest']['x'] = nn.pose.position.x
                    step['dest']['y'] = nn.pose.position.y
            plan.append(step)
        if route:
            self.memProxy.insertData("TopologicalNav/Route",
                                     plan.__repr__())
            self.memProxy.raiseEvent("TopologicalNav/PlanReady", "True")
        else:
            self.memProxy.raiseEvent("TopologicalNav/PlanReady", "False")
        self.get_plan = True

    def _on_qiplanner_goal_reached(self, data):
        print('on_qiplanner_goal_reached: %s' % data)
        self.goal_reached = True

    def _on_qiplanner_status(self, data):
        if data != self._last_status:
            print('on_qiplanner_status: %s' % data)
        self._last_status = data
        if data == 'GoalReached':
            if self.current_target == self.current_node:
                self.goal_reached = True
                self.failed = False
                self.failed_to = 'none'
                self.fail_code = 0
            else:
                self.goal_reached = False
                self.failed = True
                self.failed_to = self.current_target
                self.fail_code = 0
        elif data == 'PathNotFound':
            self.failed = True
            self.failed_to = self.current_target
            self.fail_code = 1

    def _insert_nodes(self):
        node_names = []
        for i in self.map:
            node_names.append(i.name)
        self.memProxy.insertData("TopologicalNav/Nodes", node_names)

    def _localisation_timer(self):
        pre_curnod = self.current_node
        pre_clonod = self.closest_node
        if self.__fake:
            self.closest_node = self.__fake_node
            self.current_node = self.__fake_node
        else:
            val = self.memProxy.getData("NAOqiLocalizer/RobotPose")

            dists = self.get_distances_to_pose(val)
            self.closest_node = dists[0]['node'].name
            if self.point_in_poly(dists[0]['node'], val):
                self.current_node = dists[0]['node'].name
            else:
                self.current_node = 'none'

        if pre_curnod != self.current_node:
            self.memProxy.raiseEvent("TopologicalNav/CurrentNode",
                                     self.current_node)
            self.memProxy.insertData("TopologicalNav/LastNode",
                                     self.current_node)
            print self.current_node
        if pre_clonod != self.closest_node:
            self.memProxy.raiseEvent("TopologicalNav/ClosestNode",
                                     self.closest_node)
            print self.closest_node

        self.loc_timer = Timer(0.5, self._localisation_timer)
        self.loc_timer.start()

    def get_route(self, target):
        g_node = get_node(self.map, target)
        print "get route", self.closest_node, target
        # Everything is Awesome!!!
        # Target and Origin are Different and none of them is None
        if (g_node is not None) and \
           (o_node is not None) and \
           (g_node.name != o_node.name):
            rsearch = TopologicalRouteSearch(self.map)
            route = rsearch.search_route(o_node.name, target)
            print route
        return route

    def navigate(self, target):
        if self.closest_node is None or self.closest_node == 'none':
            print ('was not localised, so assume we are at the target')
            o_node = get_node(self.map, target)
        else:
            o_node = get_node(self.map, self.closest_node)
        print self.closest_node, target

        g_node = get_node(self.map, target)

        # Everything is Awesome!!!
        # Target and Origin are Different and none of them is None
        if (g_node is not None) and\
           (o_node is not None) and\
           (g_node.name != o_node.name):
            rsearch = TopologicalRouteSearch(self.map)
            route = rsearch.search_route(o_node.name, target)
            print route
            if route:
                print "Navigating Case 1"
                self.follow_route(route)
            else:
                print "There is no route to this node check your edges ..."
        else:
            # Target and Origin are the same
            if(g_node.name == o_node.name):
                print "Target and Origin Nodes are the same"
                self.memProxy.raiseEvent("TopologicalNav/Status", "Success")
            else:
                print "Target or Origin Nodes were not found on Map"
                self.memProxy.raiseEvent("TopologicalNav/Status", "Fail")

    def follow_route(self, route):
        """
        Follow Route

        This function follows the chosen route to reach the goal
        """
        nnodes = len(route.source)
        self.navigation_activated = True
        orig = route.source[0]
        self.cancelled = False
        print str(nnodes) + " Nodes on route"

        rindex = 0
        nav_ok = True
        route_len = len(route.edge_id)

        o_node = get_node(self.map, orig)
        a = get_edge_from_id(self.map,
                             route.source[0],
                             route.edge_id[0]).action
        print "first action " + a

        # If the robot is not on a node or the first action is not move base type
        # navigate to closest node waypoint (only when first action is not move base)
        self.navigation_activated = True
        if self.current_node == 'none':
            print 'Do planner to %s' % (self.closest_node)
            self.current_target = orig
            nav_ok = self.monitored_navigation(o_node, 'NAOqiPlanner/Goal')

        while rindex < route_len and not self.cancelled:
            # current action
            cedg = get_edge_from_id(
                self.map, route.source[rindex], route.edge_id[rindex]
            )

            a = cedg.action
            # next action
            if rindex < (route_len - 1):
                a1 = get_edge_from_id(
                    self.map,
                    route.source[rindex + 1],
                    route.edge_id[rindex + 1]).action
            else:
                a1 = 'none'

            self.current_action = a
            self.next_action = a1

            cnode = get_node(self.map, cedg.node)

            print "From " + route.source[rindex] + \
                " do " + a + " to " + cedg.node
            self.current_target = cedg.node
            nav_ok = self.monitored_navigation(cnode, a)
            if nav_ok:
                rindex = rindex + 1
            elif self.fail_code == 0:  # wrong node, go the right one again
                pass
            else:
                print "qinav failed, but trying next node regardless"
                rindex = rindex + 1

        self.navigation_activated = False
        if nav_ok:
            self.memProxy.raiseEvent("TopologicalNav/Status", "Success")
        else:
            self.memProxy.raiseEvent("TopologicalNav/Status", "Fail")

    def monitored_navigation(self, gnode, command):

        if self.__fake:
            print('FAKE navigation, pretending to go to target')

            sleep(randrange(1, 10))
            print('FAKE navigation, pretending to have succeeded')
            self.memProxy.raiseEvent("TopologicalNav/Status",
                                     "PlannerSuccesful")
            self.__fake_node = gnode.name
            nav_ok = True
            self.goal_reached = True
        else:
            nav_ok = False
            gpose = gnode.pose
            self.goal_reached = False

            goal_pose = [gpose.position.x, gpose.position.y]

            print goal_pose, command

            # send goal to QiPlanner
            self.memProxy.raiseEvent(command, goal_pose)

            nTry = 0

            while nTry < self._max_retries and not self.goal_reached and not self.cancelled:
                print "monitored_nav attempt %d to %s" % (nTry, gnode.name)
                self.failure = False
                while not self.cancelled and not self.goal_reached and not self.failure:
                    if self.current_node == gnode.name:
                        print "we are in reach of the goal, so let's report success"
                        self.memProxy.raiseEvent("NAOqiPlanner/Reset", True)
                        return True

                    time.sleep(0.1)

                if self.goal_reached:
                    nav_ok = True
                    print "  succeeded going to %s" % gnode.name
                    self.memProxy.raiseEvent("TopologicalNav/Status",
                                             "PlannerSuccesful")
                    return nav_ok
                elif self.failure:
                    self.memProxy.raiseEvent("NAOqiPlanner/Reset", True)
                    print "  FAILED going to %s, reset naoqiplanner" % gnode.name
                    nav_ok = False
                    if self.fail_code == 0:
                        failmsg = "ReachedWrongNode " + self.failed_to
                        self.memProxy.raiseEvent("TopologicalNav/Status", failmsg)
                    if self.fail_code == 1:
                        failmsg = "PlannerFailedTo " + self.failed_to
                        self.memProxy.raiseEvent("TopologicalNav/Status", failmsg)
                # wait a bit before a retry.
                n = 0
                while n < nTry and not self.cancelled:
                    n += 1
                    sleep(1)
                nTry += 1
        return nav_ok

    def point_in_poly(self, node, pose):
        x = pose[0] - node.pose.position.x
        y = pose[1] - node.pose.position.y

        n = len(node.verts)
        inside = False

        p1x = node.verts[0].x
        p1y = node.verts[0].y
        for i in range(n + 1):
            p2x = node.verts[i % n].x
            p2y = node.verts[i % n].y
            if y > min(p1y, p2y):
                if y <= max(p1y, p2y):
                    if x <= max(p1x, p2x):
                        if p1y != p2y:
                            xints = (y - p1y) * (p2x - p1x) / (p2y - p1y) + p1x
                        if p1x == p2x or x <= xints:
                            inside = not inside
            p1x, p1y = p2x, p2y
        return inside

    def get_distances_to_pose(self, pose):
        """
         get_distances_to_pose

         This function returns the distance from each waypoint
         to a pose in an organised way
        """
        distances = []
        for i in self.map.nodes:
            a = dict()
            a['node'] = i
            a['dist'] = get_distance_node_pose(i, pose)
            distances.append(a)
        distances = sorted(distances, key=lambda k: k['dist'])
        return distances

    def _on_shutdown(self, signal, frame):
        print('You pressed Ctrl+C!')
        self.memProxy.raiseEvent("NAOqiPlanner/Reset", True)
        self.cancelled = True
        self.loc_timer.cancel()
        sys.exit(0)


if __name__ == '__main__':
    """
    Main entry point

    """
    parser = argparse.ArgumentParser()
    parser.add_argument("--pip", type=str, default=os.environ['PEPPER_IP'],
                        help="Robot IP address." +
                        "On robot or Local Naoqi: use '127.0.0.1'.")
    parser.add_argument("--pport", type=int, default=9559,
                        help="Naoqi port number")
    parser.add_argument("--tmap", type=str, default="INB3123.tpg",
                        help="path to topological map")
    parser.add_argument("--fake", action='store_true', default=False,
                        help="run fake nav")
    args = parser.parse_args()
    pip = args.pip
    pport = args.pport
    topomap = args.tmap

    # myBroker = ALBroker("pythonBroker", "0.0.0.0", 0, pip, pport)
    # try:
    #     pythonModule = MyModule("pythonModule")
    #     prox = ALProxy("ALMemory")
    #     prox.subscribeToEvent("TopologicalNav/Goal",
    #                           "pythonModule", "nav_goal_callback")
    #     prox.subscribeToEvent("TopologicalNav/GetPlan",
    #                           "pythonModule", "get_plan_callback")
    #     prox.subscribeToEvent("NAOqiPlanner/GoalReached",
    #                           "pythonModule", "goalreached_callback")
    #     prox.subscribeToEvent("NAOqiPlanner/Status", "pythonModule",
    #                           "status_callback")
    # except Exception, e:
    #     print "error"
    #     print e
    #     exit(1)

    server = TopologicalLocaliser(pip, pport, topomap, fake=args.fake)
