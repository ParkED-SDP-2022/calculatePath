

class Graph():
    
    class Node():
        
        def __init__(self, LongLat) -> None:
            self.LongLat = LongLat
            self.edges
            self.parent
            self.g
            self.f
        
        # add the edges to the node
        def addEdges():
            #edge knows its own cost :) + has a list of nodes?
            ...
            
        #allow nodes to be compared with A* searchs
        def compare():
            ...
        
    
    #initialise a graph object
    def __init__(self, boundaries) -> None:
        
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
        ...
        
    # for all node, check if an edge exists between all other nodes
    # by calling bounary method to check intersection 
    # add the edges to a list in the node object 
    def initializeEdges(self):
        ...
    
    
    def GetPath(self, start, end, constraint):
        # use A* search
        ...