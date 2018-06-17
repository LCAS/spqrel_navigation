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

import coloredlogs
import logging

logger = logging.Logger(__file__)
coloredlogs.install(level='DEBUG', logger=logger)
#logger.debug("specific debug")


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
        self._max_retries = 3

        # self._sub_plan = self.memProxy.subscriber(
        #     "TopologicalNav/GetPlan"
        # )
        # self._sub_plan.signal.connect(self._on_get_plan)

        # self._sub_goal_reached = self.memProxy.subscriber(
        #     "NAOqiPlanner/GoalReached"
        # )
        # self._sub_goal_reached.signal.connect(self._on_qiplanner_goal_reached)

        logger.info('subscribed to NAOqiPlanner/Result')
        self.move_base_actions = ['NAOqiPlanner/Goal']
        self.__fake = fake
        self.__fake_node = 'WayPoint1'
        self.closest_node = 'start'
        self.current_node = 'start'
        # If it fails reaching one goal this variable stores the node it was trying to reach when failed
        self.failed_to = 'none'
        self.failure = False
        self.fail_code = 0
        self.cancelled = False
        self.navigation_activated = False
        self.map = TopologicalMap(filename=topomap)
        # self.nav_timer = Timer(1.0, self._nav_timer)
        # self.nav_timer.start()
        self._last_status = 'UNKNOWN'
        self.goal_active = False
        signal.signal(signal.SIGINT, self._on_shutdown)
        #signal.pause()

    def monitored_navigation(self, command="NAOqiPlanner/Goal"):
        goal_pose = [10, 2, 0]

        logger.info('goal_pose=%s, command=%s' % (goal_pose, command))

        while not self.cancelled:
            logger.info('CALL goal_pose=%s, command=%s' % (goal_pose, command))
            self.memProxy.raiseEvent("NAOqiPlanner/Reset", True)
            sleep(.1)
            self.memProxy.raiseEvent(command, goal_pose)
            sleep(.1)
            logger.info('goal_pose=%s, command=%s' % (goal_pose, command))

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
    server.monitored_navigation()
