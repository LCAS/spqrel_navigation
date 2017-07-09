import yaml
from topological_node import topological_node


class topological_map(object):

    def __init__(self, filename):
        lnodes = self._loadMap(filename)
        self.nodes= self._get_nodes(lnodes)
        
    def _get_nodes(self, lnodes):
        nodes=[]
        for i in lnodes:
            node = topological_node(i['node'])
            nodes.append(node)
        return nodes
            

    def _loadMap(self, filename):
        print "loading " + filename
        with open(filename, 'r') as f:
            return yaml.load(f)
