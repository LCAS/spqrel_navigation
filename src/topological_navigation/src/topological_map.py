import yaml
from topological_node import topological_node


class topological_map(object):

    def __init__(self, filename):
        self.nodes=[]
        lnodes = self.loadMap(filename)
        print lnodes
        for i in lnodes:
            print "---------"
            #print i
            node = topological_node(i['node'])
            print node
            self.nodes.append(node)
            

        print self.nodes       

    def loadMap(self, filename):
        print "loading " + filename
        with open(filename, 'r') as f:
            return yaml.load(f)
