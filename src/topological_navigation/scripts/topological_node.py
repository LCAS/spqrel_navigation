class geometry(object):
    def __repr__(self):
        a = dir(self)
        b =[]
        s = ''
        for i in a:
            if not i.startswith('_'):
                b.append(str(i))
        for i in b:
                s = s + str(i) +': ' + str(self.__getattribute__(i)) + '\n'
        return s  

class node_edges(geometry):
    def __init__(self, edges):
        self.action=edges['action']
        self.edge_id=edges['edge_id']
        self.node=edges['node']
        

class node_orientation(geometry):
    def __init__(self, orientation):
        self.w=orientation['w']
        self.x=orientation['x']
        self.y=orientation['y']
        self.z=orientation['z']
        

class node_position(geometry):
    def __init__(self, position):
        self.x=position['x']
        self.y=position['y']
        self.z=position['z']
        

class node_pose(geometry):
    def __init__(self, pose):
        self.position = node_position(pose['position'])
        self.orientation = node_orientation(pose['orientation'])
        

class topological_vertex(geometry):
    def __init__(self, vertex):
        self.x=vertex['x']
        self.y=vertex['y']        


class topological_node(geometry):
    
    def __init__(self, node):
        self.pointset = node['pointset']
        self.name = node['name']
        self.pose = node_pose(node['pose'])
        self.verts = self._get_verts(node['verts'])
        self.edges = self._get_edges(node['edges'])
        
    def _get_verts(self, verts):
        vertices = []
        for i in verts:
            v = topological_vertex(i)
            vertices.append(v)
        return vertices

    def _get_edges(self, ledges):
        edges = []
        for i in ledges:
            e = node_edges(i)
            edges.append(e)
        return edges