#!/usr/bin/env python

import time
import signal
import sys
import argparse

#from topological_node import topological_node
from topological_map import topological_map
from route_search import *
from threading import Timer
from naoqi import *

myBroker=None
goal_check = 0
goal_reached = 0
get_plan = 0

# create python module
class myModule(ALModule):
    """python class myModule test auto documentation : comment needed to create a new python module"""

    def nav_goal_callback(self, strVarName, value):
        """callback when data change"""
        print "New goal", strVarName, " ", value, " "
        global goal_check
        goal_check = 1
    
    def goalreached_callback(self, strVarName, value):
        print "GOAL REACHED: ", strVarName, " ", value, " "
        #global goal_reached
        #goal_reached = 1

    def get_plan_callback(self, strVarName, value):
        print "New plan", strVarName, " ", value, " "
        global get_plan
        get_plan=1


    def status_callback(self, strVarName, value):
        """callback when data change"""
        print "status", strVarName, " ", value, " "
        global goal_reached
        goal_reached = 1

    def _pythonPrivateMethod(self, param1, param2, param3):
        global goal_check


def get_distance_node_pose(node, pose):
    """
    get_distance_node_pose
    
    Returns the straight line distance between a pose and a node
    """ 
    dist=math.hypot((pose[0]-node.pose.position.x),(pose[1]-node.pose.position.y))
    return dist


class topological_localiser(object):
    def __init__(self, pip,pport,topomap):
        self.move_base_actions =['NAOqiPlanner/Goal']
        self.closest_node = 'none'
        self.current_node = 'none'
        self.failed_to = 'none' #If it fails reaching one goal this variable stores the node it was trying to reach when failed
        self.fail_code=0
        self.cancelled = False
        self.navigation_activated = False
        self.memProxy = ALProxy("ALMemory",pip,pport)
        self.map = topological_map(filename=topomap)
        #print self.map.nodes
        self.loc_timer = Timer(0.5, self._localisation_timer)
        self.loc_timer.start()
        self.nav_timer = Timer(1.0, self._nav_timer)
        self.nav_timer.start()
        signal.signal(signal.SIGINT, self._on_shutdown)
        signal.pause()
        

    def _nav_timer(self):
        global goal_check
        global get_plan
        
        if goal_check:
            goal = self.memProxy.getData("TopologicalNav/Goal")
            print "NEW GOAL " + goal
            goal_check = 0
            self.navigate(goal)

        if get_plan:
            goal = self.memProxy.getData("TopologicalNav/GetPlan")
            route = self.get_route(goal)
            plan=[]
            for i in range(len(route.source)):
                step={}
                cn = get_node(self.map, route.source[i])
                step['from']=route.source[i]
                for j in cn.edges:
                    if j.edge_id == route.edge_id[i]:
                        step['edge_id']=j.edge_id
                        step['action']=j.action
                        step['dest']={}
                        step['dest']['node']=j.node
                        nn=get_node(self.map,j.node)
                        step['dest']['x']=nn.pose.position.x
                        step['dest']['y']=nn.pose.position.y                        
                plan.append(step)
            if route:
                self.memProxy.insertData("TopologicalNav/Route", plan.__repr__())
                self.memProxy.raiseEvent("TopologicalNav/PlanReady", "True")
            else:
                self.memProxy.raiseEvent("TopologicalNav/PlanReady", "False")
                
            get_plan=0
            

        self.nav_timer = Timer(0.5, self._nav_timer)
        self.nav_timer.start()


    def _localisation_timer(self):
        pre_curnod = self.current_node
        pre_clonod = self.closest_node
        val = self.memProxy.getData("NAOqiLocalizer/RobotPose")
        #print val
        dists=self.get_distances_to_pose(val)
        self.closest_node = dists[0]['node'].name
        if self.point_in_poly(dists[0]['node'], val):
            self.current_node = dists[0]['node'].name
        else:
            self.current_node = 'none'

        if (pre_curnod != self.current_node) or (pre_clonod != self.closest_node):
            print self.current_node, self.closest_node

        global goal_reached
        
        if goal_reached:
            if self.navigation_activated:
                val = self.memProxy.getData("NAOqiPlanner/Status")
                if val == 'GoalReached':
                    if self.current_target == self.current_node:
                        self.goal_reached = True
                        self.cancelled= False
                        self.failed_to = 'none'
                        self.fail_code=0
                    else:
                        self.goal_reached = False
                        self.cancelled=True
                        self.failed_to = self.current_target
                        self.fail_code=0
                elif val == 'PathNotFound':
                    self.cancelled=True
                    self.failed_to = self.current_target
                    self.fail_code=1
                goal_reached=False
        self.loc_timer = Timer(0.5, self._localisation_timer)
        self.loc_timer.start()



    def get_route(self, target):
        o_node = get_node(self.map, self.closest_node)
        g_node = get_node(self.map, target)
        print "get route", self.closest_node, target
        # Everything is Awesome!!!
        # Target and Origin are Different and none of them is None
        if (g_node is not None) and (o_node is not None) and (g_node.name != o_node.name) :
            rsearch = TopologicalRouteSearch(self.map)
            route = rsearch.search_route(o_node.name, target)
            print route
            
        return route


    def navigate(self, target):
        #tries=0
        #result = False
        
        print self.closest_node, target
        
        o_node = get_node(self.map, self.closest_node)
        g_node = get_node(self.map, target)
        
        # Everything is Awesome!!!
        # Target and Origin are Different and none of them is None
        if (g_node is not None) and (o_node is not None) and (g_node.name != o_node.name) :
            rsearch = TopologicalRouteSearch(self.map)
            route = rsearch.search_route(o_node.name, target)
            print route
            if route:
                print "Navigating Case 1"
                self.followRoute(route, target)
            else:
                print "There is no route to this node check your edges ..."
        else :
            # Target and Origin are the same
            if(g_node.name == o_node.name) :
                print "Target and Origin Nodes are the same"
            else:
                print "Target or Origin Nodes were not found on Map"


    """
     Follow Route
     
     This function follows the chosen route to reach the goal
    """
    def followRoute(self, route, target):
        nnodes=len(route.source)

        self.navigation_activated=True
        Orig = route.source[0]
        Targ = target
        self._target = Targ
        
        self.cancelled=False


        print str(nnodes) + " Nodes on route"

        inc=1
        rindex=0
        nav_ok=True
        route_len = len(route.edge_id)
        
        o_node = get_node(self.map, Orig)
        a = get_edge_from_id(self.map, route.source[0], route.edge_id[0]).action
        print "first action " + a

        # If the robot is not on a node or the first action is not move base type
        # navigate to closest node waypoint (only when first action is not move base)
        if self.current_node == 'none' and a not in self.move_base_actions :
            if a not in self.move_base_actions:
                print 'Do planner to %s' %(self.closest_node)
                inf = o_node.pose
                self.current_target = Orig
                nav_ok = self.monitored_navigation(inf,'NAOqiPlanner/Goal')
        else:
            if a not in self.move_base_actions :
                action_server = 'NAOqiPlanner/Goal'
                move_base_act= False
                for i in o_node.edges :
                    # Check if there is a move_base action in the edages of this node
                    # if not is dangerous to move
                    if i.action in self.move_base_actions :
                        move_base_act = True

                if not move_base_act :
                    print "Action not taken, it is dangerous to move. Outputing success."
                    nav_ok = True
                else:
                    print "Getting to exact pose"
                    self.current_target = Orig
                    nav_ok = self.monitored_navigation(o_node.pose, action_server)

        while rindex < route_len and not self.cancelled: #and nav_ok :
#            #current action
            cedg = get_edge_from_id(self.map, route.source[rindex], route.edge_id[rindex])

            a = cedg.action
            #next action
            if rindex < (route_len-1) :
                a1 = get_edge_from_id(self.map, route.source[rindex+1], route.edge_id[rindex+1]).action
            else :
                a1 = 'none'

            self.current_action = a
            self.next_action = a1
            
            cnode = get_node(self.map, cedg.node)

            print "From " + route.source[rindex] + " do " +  a + " to " + cedg.node
            self.current_target = cedg.node
            inf = cnode.pose
            nav_ok = self.monitored_navigation(inf, a)
            rindex=rindex+1

        self.navigation_activated=False
        if nav_ok:
            self.memProxy.raiseEvent("TopologicalNav/Status", "Success")
        else:
            self.memProxy.raiseEvent("TopologicalNav/Status", "Fail")


    def monitored_navigation(self, gpose, command):

        self.goal_reached=False

        goal_pose=[gpose.position.x, gpose.position.y]
        
        print goal_pose, command
        self.memProxy.raiseEvent(command, goal_pose)

        while not self.cancelled and not self.goal_reached :
            time.sleep(0.1)


        if self.goal_reached:
            nav_ok=True
            self.memProxy.raiseEvent("TopologicalNav/Status", "PlannerSuccesful")
        elif self.cancelled:
            nav_ok=False
            if self.fail_code==0:
                failmsg="ReachedWrongNode "+self.failed_to
                self.memProxy.raiseEvent("TopologicalNav/Status", failmsg)
            if self.fail_code==1:
                failmsg="PlannerFailedTo "+self.failed_to
                self.memProxy.raiseEvent("TopologicalNav/Status", failmsg)
        return nav_ok


    def point_in_poly(self,node,pose):
        x=pose[0]-node.pose.position.x
        y=pose[1]-node.pose.position.y

        n = len(node.verts)
        inside = False

        p1x = node.verts[0].x
        p1y = node.verts[0].y
        for i in range(n+1):
            p2x = node.verts[i % n].x
            p2y = node.verts[i % n].y
            if y > min(p1y,p2y):
                if y <= max(p1y,p2y):
                    if x <= max(p1x,p2x):
                        if p1y != p2y:
                            xints = (y-p1y)*(p2x-p1x)/(p2y-p1y)+p1x
                        if p1x == p2x or x <= xints:
                            inside = not inside
            p1x,p1y = p2x,p2y
        return inside


    def get_distances_to_pose(self, pose):
        """
         get_distances_to_pose
    
         This function returns the distance from each waypoint to a pose in an organised way
        """
        distances=[]
        for i in self.map.nodes:
            d= get_distance_node_pose(i, pose)#get_distance_to_node(i, msg)
            a={}
            a['node'] = i
            a['dist'] = d
            distances.append(a)
    
        distances = sorted(distances, key=lambda k: k['dist'])
        return distances


    def _on_shutdown(self, signal, frame):
        print('You pressed Ctrl+C!')
        global myBroker
        self.cancelled=True
        myBroker.shutdown()
        self.loc_timer.cancel()
        self.nav_timer.cancel()
        sys.exit(0)



if __name__ == '__main__':
    """ 
    Main entry point
    
    """
    parser = argparse.ArgumentParser()
    parser.add_argument("--pip", type=str, default=os.environ['PEPPER_IP'],
                        help="Robot IP address.  On robot or Local Naoqi: use '127.0.0.1'.")
    parser.add_argument("--pport", type=int, default=9559,
                        help="Naoqi port number")
    parser.add_argument("--tmap", type=str, default="INB3123.tpg",
                        help="path to topological map")
    args = parser.parse_args()
    pip = args.pip
    pport = args.pport
    topomap = args.tmap


    myBroker = ALBroker("pythonBroker","0.0.0.0",0,pip,pport)
    try:   
      pythonModule = myModule("pythonModule")
      prox = ALProxy("ALMemory")
      prox.subscribeToEvent("TopologicalNav/Goal","pythonModule", "nav_goal_callback")
      prox.subscribeToEvent("TopologicalNav/GetPlan","pythonModule", "get_plan_callback")      
      prox.subscribeToEvent("NAOqiPlanner/GoalReached","pythonModule", "goalreached_callback")
      prox.subscribeToEvent("NAOqiPlanner/Status","pythonModule", "status_callback")
    
    except Exception,e:
      print "error"
      print e
      exit(1)

    server=topological_localiser(pip,pport,topomap)
