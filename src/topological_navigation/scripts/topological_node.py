#! /usr/bin/env python


class Geometry(object):
    def __repr__(self):
        a = dir(self)
        b = []
        s = ''
        for i in a:
            if not i.startswith('_'):
                b.append(str(i))
        for i in b:
                s = s + str(i) + ': ' + str(self.__getattribute__(i)) + '\n'
        return s


class NodeEdges(Geometry):
    def __init__(self, edges):
        self.action = edges['action']
        self.edge_id = edges['edge_id']
        self.node = edges['node']


class NodeOrientation(Geometry):
    def __init__(self, orientation):
        self.w = orientation['w']
        self.x = orientation['x']
        self.y = orientation['y']
        self.z = orientation['z']


class NodePosition(Geometry):
    def __init__(self, position):
        self.x = position['x']
        self.y = position['y']
        self.z = position['z']


class NodePose(Geometry):
    def __init__(self, pose):
        self.position = NodePosition(pose['position'])
        self.orientation = NodeOrientation(pose['orientation'])


class TopologicalVertex(Geometry):
    def __init__(self, vertex):
        self.x = vertex['x']
        self.y = vertex['y']


class TopologicalNode(Geometry):

    def __init__(self, node):
        self.pointset = node['pointset']
        self.name = node['name']
        self.pose = NodePose(node['pose'])
        self.verts = self._get_verts(node['verts'])
        self.edges = self._get_edges(node['edges'])

    def _get_verts(self, verts):
        vertices = []
        for i in verts:
            v = TopologicalVertex(i)
            vertices.append(v)
        return vertices

    def _get_edges(self, ledges):
        edges = []
        for i in ledges:
            e = NodeEdges(i)
            edges.append(e)
        return edges
