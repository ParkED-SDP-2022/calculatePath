

class Graph:
    
    class LongLat:
        
        def __init__(self, long, lat):
            self.Long = long
            self.Lat = lat
    
    
    class Node:
        
        def __init__(self, LongLat):
            self.LongLat = LongLat
            self.edges = []
            self.parent = None
            self.g = float('inf')
            self.f = float('inf')
        
        
        # add the edge to the node
        def addEdge(self, e):
            self.edges.append(e)
            
        # allow nodes to be compared with A* searchs
        def compare():
            ...
        
        
    class Edge:
        
        def __init__(self, nodeA, nodeB):
            self.start = nodeA
            self.end = nodeB
            #TODO: implement cost calculation? 
            self.cost = calculateCost(nodeA, nodeB)
       
    
    #initialise a graph object
    def __init__(self, boundaries):
        
        #boundaries of the park 
        self.Boundaries = boundaries
        
        # a dictionary of longlats to nodes in the graph
        self.nodes = self.initializeNodes()
        
        # a list of edges which cannot be traversed
        self.ignoreEdgeMatrix        
        
    # generates all nodes in the graph,
    # using self.boundaries object to 
    # generate the grid longlats
    def initializeNodes(self):
        return self.Boundaries.getNodes()
        
    # for all node, check if an edge exists between all other nodes
    # by calling bounary method to check intersection 
    # add the edges to a list in the node object 
    def initializeEdges(self):
        for nodeA in self.nodes:
            for nodeB in self.nodes:
                #TODO: not repeate calculations of existing edges
                #TODO: what format to validate? node, edge, longlats?
                if self.Boundaries.isValidPath(nodeA.LongLat, nodeB.LongLat):
                    nodeA.addEdge(Edge(nodeA, nodeB))
                    
    
    
    def GetPath(self, start, end, constraint):
        # use A* search
        ...