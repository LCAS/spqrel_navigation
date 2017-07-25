#! /usr/bin/env python

import argparse
import cv2
import copy
import math
import numpy as np
import signal
import sys
import yaml

from os.path import dirname, realpath, join

from route_search import get_node
from topological_map import TopologicalMap
from topological_node import NodeEdges
from topological_node import TopologicalNode


script_path = realpath(dirname(__file__))

class TopologicalEditor(object):

    modes = {
        'help': 'press \'h\' for help',
        'add_node': 'Add Node',
        'add_edge': 'Add Edge',
        'remove_node': 'Delete Node',
        'move': 'Move Node',
        'none': ''
    }

    def __init__(self, mapprops, topomap, outfile):

        self.debug = False

        signal.signal(signal.SIGINT, self.signal_handler)
        self.props = self.read_map_properties(mapprops)

        mapprops_path = realpath(dirname(mapprops))

        if self.debug:
            print self.props
        self.current_mode = 'none'
        self.running = True
        self.msg_cache = dict()

        imgpath = join(mapprops_path, self.props['image'])

        self.imgFile = cv2.imread(imgpath)

        self.outfile = outfile
        self.top_map = TopologicalMap(filename=topomap)

        self.base_img = self.draw_top_map()
        # cv2.putText(self.base_img, self.modes['help'], (10, 30), self.font, 0.5, (20, 20, 20), 1)

        self.font = cv2.FONT_HERSHEY_SIMPLEX

        height, width, channels = self.base_img.shape
        self.origin = list()
        self.origin.append(self.props['origin'][0])
        self.origin.append(self.props['origin'][1] + (height * self.props['resolution']))
        self.origin.append(self.props['resolution'])

        cv2.namedWindow('image')
        cv2.setMouseCallback('image', self.click_callback)

        self.img = self.base_img.copy()
        while(self.running):
            cv2.imshow('image', self.img)
            k = cv2.waitKey(20) & 0xFF
            self._change_mode(k)

    def draw_top_map(self):
        topmap_image = self.imgFile.copy()
        height, width, channels = topmap_image.shape
        font = cv2.FONT_HERSHEY_SIMPLEX

        origin = list()
        origin.append(self.props['origin'][0])
        origin.append(self.props['origin'][1] + (height * self.props['resolution']))
        origin.append(self.props['resolution'])

        for i in self.top_map.nodes:
            v1 = i.pose.position
            for j in i.edges:
                v2 = get_node(self.top_map, j.node).pose.position
                self.draw_arrow(topmap_image, v1, v2, (255, 0, 0, 255), origin, thickness=2, arrow_magnitude=5, line_type=1)
            xval = int((v1.x - origin[0]) / self.props['resolution'])
            yval = int((origin[1] - v1.y) / self.props['resolution'])
            if self.debug:
                print xval, yval
            cv2.circle(topmap_image, (int(xval), int(yval)), 10, (0, 0, 255, 255), -1)
            cv2.putText(topmap_image, i.name, (int(xval), int(yval - 5)), font, 0.3, (20, 20, 20), 1)
        return topmap_image

    def draw_arrow(self, image, v1, v2, color, origin, arrow_magnitude=5, thickness=1, line_type=8, shift=0):
        v3 = dict()
        v3['x'] = (v1.x + v2.x) / 2
        v3['y'] = (v1.y + v2.y) / 2

        xval = int((v1.x - origin[0]) / origin[2])
        yval = int((origin[1] - v1.y) / origin[2])
        p = (xval, yval)

        xval2 = int((v3['x'] - origin[0]) / origin[2])
        yval2 = int((origin[1] - v3['y']) / origin[2])
        q = (xval2, yval2)

        # draw arrow tail
        cv2.line(image, p, q, color, thickness, line_type, shift)
        # calc angle of the arrow
        angle = np.arctan2(p[1] - q[1], p[0] - q[0])
        # starting point of first line of arrow head
        p = (int(q[0] + arrow_magnitude * np.cos(angle + np.pi / 4)),
        int(q[1] + arrow_magnitude * np.sin(angle + np.pi / 4)))
        # draw first half of arrow head
        cv2.line(image, p, q, color, thickness, line_type, shift)
        # starting point of second line of arrow head
        p = (int(q[0] + arrow_magnitude * np.cos(angle - np.pi / 4)),
        int(q[1] + arrow_magnitude * np.sin(angle - np.pi / 4)))
        # draw second half of arrow head
        cv2.line(image, p, q, color, thickness, line_type, shift)

    def _change_mode(self, k):
        if k == 27:
            self.running = False
        elif k == ord('a'):
            self._set_mode('add_node')
        elif k == ord('d'):
            self._set_mode('remove_node')
        elif k == ord('e'):
            self._set_mode('add_edge')
            self.origin_node = 'none'
        elif k == ord('h'):
            self._set_mode('none')
            print("Press (a) to add node\nPress (d) to remove node\nPress (e) to add edge between to nodes (click origin first and then destination)\nPress (h) for this Menu\nPress (m) to move a node\nPress (w) to write map to outmap\n")
        elif k == ord('m'):
            self._set_mode('move')
        elif k == ord('w'):
            self._set_mode('none')
            self.write_map()

    def write_map(self):
        text = "saving map to " + self.outfile
        cv2.putText(self.img, text, (10, 30), self.font, 0.5, (20, 20, 20), 1)
        yml = yaml.safe_dump(self.top_map.get_dict(), default_flow_style=False)
        if self.debug:
            print yml
        with open(self.outfile, 'w') as outfile:
            outfile.write(str(yml))
            print "map Successfully writen to", self.outfile

    def _set_mode(self, new_mode):
            self.current_mode = new_mode
            self.img = self.base_img.copy()
            cv2.putText(self.img, self.modes[self.current_mode], (10, 15), self.font, 0.5, (20, 20, 20), 1)

    def read_map_properties(self, mapprops):
        with open(mapprops, 'r') as f:
            props = yaml.load(f)
        return props

    def cb_move(self, dists, xpos, ypos):
        text = "Moving " + dists[0]['node'].name
        cv2.putText(self.img, text, (10, 30), self.font, 0.5, (20, 20, 20), 1)
        self.moving = True
        self.movingnode = dists[0]['node'].name

    def cb_add_node(self, dists, xpos, ypos):
        node = self.load_msgs('nodeMsg')
        if self.debug:
            print node[0]
        node[0]['node']['edges'] = list()
        node[0]['node']['name'] = self.get_new_name()
        node[0]['node']['pose']['position']['x'] = xpos
        node[0]['node']['pose']['position']['y'] = ypos
        self.top_map.nodes.append(TopologicalNode(node[0]['node']))
        self.base_img = self.draw_top_map()
        self._set_mode('none')

    def cb_remove_node(self, dists, xpos, ypos):
        for node in self.top_map.nodes:
            if node.name == dists[0]['node'].name:
                self.top_map.nodes.remove(node)
            else:
                for ne in node.edges:
                    if ne.node == dists[0]['node'].name:
                        node.edges.remove(ne)
        self.base_img = self.draw_top_map()
        self._set_mode('none')

    def cb_add_edge(self, dists, xpos, ypos):
        if self.origin_node == 'none':
            text = "Connecting " + dists[0]['node'].name + " to "
            cv2.putText(self.img, text, (10, 30), self.font, 0.5, (20, 20, 20), 1)
            self.origin_node = dists[0]['node'].name
        else:
            self.connect_nodes(self.origin_node, dists[0]['node'].name)
            self._set_mode('none')

    def click_callback(self, event, x, y, flags, param):
        xval = (x * self.origin[2]) + self.origin[0]
        yval = -((y * self.origin[2]) - self.origin[1])
        dists = self.get_distances_to_pose(xval, yval)

        if event == cv2.EVENT_LBUTTONDOWN:
            try:
                getattr(self, 'cb_' + self.current_mode)(dists, xval, yval)
            except AttributeError:
                pass

        if event == cv2.EVENT_LBUTTONUP:
            if self.current_mode == 'move' and self.moving:
                self.update_node_position(self.movingnode, xval, yval)
                # back to move mode
                self._set_mode('move')
                self.moving = False

    def connect_nodes(self, origin, dest):
        edge = dict()
        edge['action'] = 'NAOqiPlanner/Goal'
        edge['edge_id'] = origin + "_" + dest
        edge['node'] = dest
        new_edge = NodeEdges(edge)
        for i in self.top_map.nodes:
            if i.name == origin:
                i.edges.append(new_edge)
        self.base_img = self.draw_top_map()
        self._set_mode('none')

    def load_msgs(self, msg_name):
        if(msg_name not in self.msg_cache.keys()):
            with open(join(script_path, '../msgs/') +
                      msg_name + '.yaml', 'r') as raw_msg:
                self.msg_cache[msg_name] = yaml.load(raw_msg)
        return copy.deepcopy(self.msg_cache[msg_name])

    def get_new_name(self):
        namesnum = list()
        for i in [j.name for j in self.top_map.nodes]:
            if i.startswith('WayPoint'):
                namesnum.append(int(i.strip('WayPoint')))
        namesnum.sort()
        return 'WayPoint%d' % (int(namesnum[-1]) + 1) if namesnum else 'WayPoint1'

    def update_node_position(self, node, xpos, ypos):
        for i in self.top_map.nodes:
            if i.name == node:
                i.pose.position.x = xpos
                i.pose.position.y = ypos
                if self.debug:
                    print i.pose.position
        # redraw top map
        self.base_img = self.draw_top_map()

    def get_distances_to_pose(self, x, y):
        """
         get_distances_to_pose
         This function returns the distance from each waypoint to a pose in an organised way
        """

        distances = list()
        for node in self.top_map.nodes:
            distance = dict()
            distance['node'] = node
            distance['dist'] = math.hypot((x - node.pose.position.x), (y - node.pose.position.y))
            distances.append(distance)
        return sorted(distances, key=lambda k: k['dist'])

    def signal_handler(self, signal, frame):
        cv2.destroyAllWindows()
        print('You pressed Ctrl+C!')
        sys.exit(0)


def main():
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

    TopologicalEditor(args.map, args.tmap if not args.empty else None, args.outmap)

if __name__ == '__main__':
    main()
