#! /usr/bin/env python


import cv2
import yaml
import argparse
import math
import numpy as np
import signal
import sys


from topological_map import topological_map
from topological_node import topological_node
from topological_node import node_edges
from route_search import *


node_str ="""- node:
    edges:
    name: WayPoint
    pointset: NEW
    pose:
      orientation:
        w: 1.0
        x: 0.0
        y: 0.0
        z: 0.0
      position:
        x: 0.0
        y: 0.0
        z: 0.0
    verts:
    - x: 0.689999997616
      y: 0.287000000477
    - x: 0.287000000477
      y: 0.689999997616
    - x: -0.287000000477
      y: 0.689999997616
    - x: -0.689999997616
      y: 0.287000000477
    - x: -0.689999997616
      y: -0.287000000477
    - x: -0.287000000477
      y: -0.689999997616
    - x: 0.287000000477
      y: -0.689999997616
    - x: 0.689999997616
      y: -0.287000000477"""

def get_distance_node_pose(node, pose):
    """
    get_distance_node_pose
    
    Returns the straight line distance between a pose and a node
    """ 
    dist=math.hypot((pose[0]-node.pose.position.x),(pose[1]-node.pose.position.y))
    return dist



def draw_arrow(image, V1, V2, color, origin, arrow_magnitude=5, thickness=1, line_type=8, shift=0):
    xval = int((V1.x-origin[0])/origin[2])
    yval = int((origin[1]-V1.y)/origin[2])
        
#    xval = (int(V1.x/0.05))+origin[0]
#    yval = origin[1]+(int(V1.y/0.05))

    V3 = {}
    V3['x'] = (V1.x+V2.x)/2
    V3['y'] = (V1.y+V2.y)/2   
    p=(xval,yval)
    
    xval2 = int((V3['x']-origin[0])/origin[2])
    yval2 = int((origin[1]-V3['y'])/origin[2])

#    xval2 = (int(V3['x']/0.05))+origin[0]#map2d.info.resolution))+origin[0]
#    yval2 = origin[1]+(int(V3['y']/0.05))#map2d.info.resolution))    
    q=(xval2,yval2)
    
    # draw arrow tail
    cv2.line(image, p, q, color, thickness, line_type, shift)
    # calc angle of the arrow
    angle = np.arctan2(p[1]-q[1], p[0]-q[0])
    # starting point of first line of arrow head
    p = (int(q[0] + arrow_magnitude * np.cos(angle + np.pi/4)),
    int(q[1] + arrow_magnitude * np.sin(angle + np.pi/4)))
    # draw first half of arrow head
    cv2.line(image, p, q, color, thickness, line_type, shift)
    # starting point of second line of arrow head
    p = (int(q[0] + arrow_magnitude * np.cos(angle - np.pi/4)),
    int(q[1] + arrow_magnitude * np.sin(angle - np.pi/4)))
    # draw second half of arrow head
    cv2.line(image, p, q, color, thickness, line_type, shift)




def draw_top_map(points, info, map2dimg):
    topmap_image = map2dimg.copy()
    height, width, channels = topmap_image.shape
    font = cv2.FONT_HERSHEY_SIMPLEX


    origin=[]
    origin.append(info['origin'][0])
    origin.append(info['origin'][1] + (height*info['resolution']))
    origin.append(info['resolution'])
    
    #yoff= info['origin'][1] + (height*info['resolution'])
    #print yoff
    
    for i in points.nodes:
        V1=i.pose.position
        for j in i.edges:
            V2 = get_node(points, j.node).pose.position
            #print j.edge_id, est.probs[est.edge_ids.index(j.edge_id)], est.durations[est.edge_ids.index(j.edge_id)]
            draw_arrow(topmap_image, V1, V2, (255,0,0,255), origin, thickness=2, arrow_magnitude=5, line_type=1)
        xval = int((V1.x-origin[0])/info['resolution']) #(int(V1.x/info['resolution']))+origin[0]
        yval = int((origin[1]-V1.y)/info['resolution'])
        print xval, yval
        cv2.putText(topmap_image,i.name,(int(xval), int(yval-5)), font, 0.3,(20,20,20),1,cv2.LINE_AA)
        cv2.circle(topmap_image, (int(xval), int(yval)), 10, (0,0,255,255), -1)
    

    return topmap_image


class topological_editor(object):
    
    def __init__(self, mapprops,topomap, outfile):

        signal.signal(signal.SIGINT, self.signal_handler)
        self.props = self.read_map_properties(mapprops)
        print self.props
        self.function='none'
        self.running=True
        
        self.imgFile = cv2.imread(self.props['image'])
        
        self.outfile = outfile
        self.top_map = topological_map(filename=topomap)
        self.base_img = draw_top_map(self.top_map, self.props, self.imgFile)

        self.font = cv2.FONT_HERSHEY_SIMPLEX

        height, width, channels = self.base_img.shape
        self.origin=[]
        self.origin.append(self.props['origin'][0])
        self.origin.append(self.props['origin'][1] + (height*self.props['resolution']))
        self.origin.append(self.props['resolution'])        
        
        
        cv2.namedWindow('image')
        cv2.setMouseCallback('image',self.click_callback)
        
        self.img = self.base_img.copy()
        while(self.running):
            cv2.imshow('image',self.img)
            k = cv2.waitKey(20) & 0xFF
            self._change_function(k)


    def _change_function(self, k):
        if k == 27:
            self.running=False
        elif k == ord('a'):
            self._set_function('add_node', 'Add Node')
        elif k == ord('e'):
            self._set_function('add_edge', 'Add Edge')
            self.origin_node = 'none'
        elif k == ord('h'):
            self._set_function('none', 'Help Menu')
        elif k == ord('m'):
            self._set_function('move', 'Move Node')
        elif k == ord('w'):
            self._set_function('none', 'Save map')
            self.write_map()


    def write_map(self):
        text = "saving map to " + self.outfile
        cv2.putText(self.img,text,(10,30), self.font, 0.5,(20,20,20),1,cv2.LINE_AA)
        
#        self.top_map.sort(key=lambda x: x['node']['name'])
        yml = yaml.safe_dump(self.top_map.get_dict(), default_flow_style=False)
        print yml
        #print s_output
        
        fh = open(self.outfile, "w")
#        #s_output = str(bson.json_util.dumps(nodeinf, indent=1))
        s_output = str(yml)
#        #print s_output
        fh.write(s_output)
        fh.close     

    def _set_function(self, function, text):
            self.function = function
            self.img = self.base_img.copy()
            cv2.putText(self.img,text,(10,15), self.font, 0.5,(20,20,20),1,cv2.LINE_AA)

    def read_map_properties(self, mapprops):
        with open(mapprops, 'r') as f:
            props = yaml.load(f)
        return props


    def click_callback(self, event,x,y,flags,param):
        self.cmx=x
        self.cmy=y
        if event == cv2.EVENT_LBUTTONDOWN:
            xval = (self.cmx*self.origin[2])+self.origin[0]  
            yval = -((self.cmy*self.origin[2])-self.origin[1]) 
            dists = self.get_distances_to_pose([xval, yval])
            
            if self.function == 'move':
                text= "Moving " + dists[0]['node'].name
                cv2.putText(self.img,text,(10,30), self.font, 0.5,(20,20,20),1,cv2.LINE_AA)
                self.moving =True
                self.movingnode=dists[0]['node'].name
                
            elif self.function == 'add_node':
                xval = (self.cmx*self.origin[2])+self.origin[0]   
                yval = -((self.cmy*self.origin[2])-self.origin[1])
                self.add_new_node(xval, yval)

            elif self.function == 'add_edge':
                xval = (self.cmx*self.origin[2])+self.origin[0]   
                yval = -((self.cmy*self.origin[2])-self.origin[1])
                
                if self.origin_node == 'none':
                    text= "Connecting " + dists[0]['node'].name + " to "
                    cv2.putText(self.img,text,(10,30), self.font, 0.5,(20,20,20),1,cv2.LINE_AA)
                    self.origin_node=dists[0]['node'].name
                else:
                    self.connect_nodes(self.origin_node, dists[0]['node'].name)
                    
        if event == cv2.EVENT_LBUTTONUP:
            if self.function == 'move' and self.moving:
                xval = (self.cmx*self.origin[2])+self.origin[0]   #int((V1.x-)/)
                yval = -((self.cmy*self.origin[2])-self.origin[1]) #-V1.y)/origin[2])
                print x, y, xval, yval
                self.update_node_position(self.movingnode, xval, yval)

                #back to move function
                self._set_function('move', 'Move Node')
                self.moving = False

    def connect_nodes(self, origin, dest):
        Edge={}
        Edge['action'] = 'NAOqiPlanner/Goal'
        Edge['edge_id'] = origin+ "_" +dest
        Edge['node'] = dest
        new_edge= node_edges(Edge)
        for i in self.top_map.nodes:
            if i.name == origin:
                i.edges.append(new_edge)
        self.base_img = draw_top_map(self.top_map, self.props, self.imgFile)
        self._set_function('none', '')

    def add_new_node(self, xpos, ypos):
        name = self.get_new_name()
        print name
        global node_str
        #print node_str
        node = yaml.load(node_str)
        print node[0]
        node[0]['node']['edges']=[]
        node[0]['node']['name']=name
        node[0]['node']['pose']['position']['x']=xpos
        node[0]['node']['pose']['position']['y']=ypos
        new_node = topological_node(node[0]['node'])
        self.top_map.nodes.append(new_node)
        self.base_img = draw_top_map(self.top_map, self.props, self.imgFile)
        self._set_function('none', '')
        
    def get_new_name(self):
        namesnum=[]
        names=[]
        for i in self.top_map.nodes:
            names.append(i.name)
        for i in names :
            if i.startswith('WayPoint') :
                nam = i.strip('WayPoint')
                namesnum.append(int(nam))
        namesnum.sort()
        if namesnum:
            nodname = 'WayPoint%d'%(int(namesnum[-1])+1)
        else :
            nodname = 'WayPoint1'
        return nodname


    def update_node_position(self, node, xpos, ypos):
        for i in self.top_map.nodes:
            if i.name == node:
                i.pose.position.x=xpos
                i.pose.position.y=ypos
                print i.pose.position
        #redraw top map
        self.base_img = draw_top_map(self.top_map, self.props, self.imgFile)


    def get_distances_to_pose(self, pose):
        """
         get_distances_to_pose
    
         This function returns the distance from each waypoint to a pose in an organised way
        """
        distances=[]
        for i in self.top_map.nodes:
            d= get_distance_node_pose(i, pose)#get_distance_to_node(i, msg)
            a={}
            a['node'] = i
            a['dist'] = d
            distances.append(a)
    
        distances = sorted(distances, key=lambda k: k['dist'])
        return distances



    def signal_handler(self, signal, frame):
        cv2.destroyAllWindows()
        print('You pressed Ctrl+C!')
        sys.exit(0)



if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument("--map", type=str, default='lab.yaml',
                        help="Metric map")
    parser.add_argument("--tmap", type=str, default="INB1004.tpg",
                        help="path to topological map")
    parser.add_argument("--empty", type=bool, default=False,
                        help="set to true to start with empty map")
    parser.add_argument("--outmap", type=str, default="out.tpg",
                        help="path to topological map")

    args = parser.parse_args()
    mapprops = args.map

    outmap = args.outmap

    if not args.empty:
        topomap = args.tmap
    else:
        topomap = None

    server = topological_editor(mapprops,topomap,outmap)    


