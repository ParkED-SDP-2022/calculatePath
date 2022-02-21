from long_lat import LongLat
from long_lat import distance
import heapq

class Graph:    
    
    class Node:
        
        def __init__(self, LongLat : LongLat):
            self.longLat = LongLat
            self.edges = []
            self.parent = None
            self.g = float('inf')
            self.f = float('inf')
        
        
        # add the edge to the node
        def addEdge(self, e):
            self.edges.append(e)
            
        def __eq__(self, other):
            return self.longLat == other.longLat
        
        
    class Edge:
        
        def __init__(self, nodeA, nodeB):
            self.start = nodeA
            self.end = nodeB
            self.cost = nodeA.longLat.distance(nodeB)
       
    
    #initialise a graph object
    def __init__(self, boundaries):
        
        #boundaries of the park 
        self.Boundaries = boundaries
        
        # a dictionary of longlats to nodes in the graph
        self.nodes = self.initializeNodes()
        
        # a list of edges which cannot be traversed
        self.ignoreEdgeMatrix = [] #TODO: enable this  
        
    # generates all nodes in the graph,
    # using self.boundaries object to 
    # generate the grid longlats
    def initializeNodes(self):
        nodes = []
        for longlat in self.Boundaries.generatePoints():
            nodes.append(self.Node(longlat))
        return nodes
        
    # for all node, check if an edge exists between all other nodes
    # by calling bounary method to check intersection 
    # add the edges to a list in the node object 
    def initializeEdges(self):
        for nodeA in self.nodes:
            for nodeB in self.nodes:
                #TODO: not repeate calculations of existing edges
                #TODO: what format to validate? node, edge, longlats?
                if self.Boundaries.isValidEdge(nodeA.LongLat, nodeB.LongLat):
                    nodeA.addEdge(self.Edge(nodeA, nodeB))
                    
    
    
    def GetPath(self, start, end, constraint):
        #TODO: determine when to flush the ignore matrix
        self.ignoreEdgeMatrix.extend(self.constraintToEdges(constraint))
        
        #Add beginning and end as nodes to the graph
        startNode, goalNode = self.addStartEnd(start, end)
        
        toVisit = []        
        heapq.heappush(toVisit, startNode)
        result = None
        
        while len(toVisit) > 0:
            
            currentNode = heapq.heappop(toVisit)            
            for edge in currentNode.edges:
                child = edge.end #TODO: make sure to use the right end of the edge here
                if child == goalNode:
                    child.parent = currentNode
                    result = child
                    break 
                else:
                    temp_g = edge.cost
                    temp_f = temp_g + distance(child.longLat, goalNode.longLat)
                    if child.f > temp_f:
                        child.parent = currentNode
                        child.g = temp_g
                        child.f = temp_f
                        if child not in toVisit:
                            heapq.heappush(child)
            ...
        
        
        # remove temporary modifications to graph
        self.flush(start, end, constraint)
        return 
        
    # traverse the node parents to reconstruct the optimal route used
    def reconstructPath(self, goal : self.Node):
        path = [] 
        currentNode = goal
        while currentNode:
            path.insert(0, currentNode.longLat)
            currentNode = currentNode.parent
        return path
        
    # Add the start and goal locations to the graph, connecting them via valid edges
    def addStartEnd(self, start, end):
        
        startNode = self.Node(start)
        endNode = self.Node(end)
        endNode.g = 0
        endNode.f =0 
        
        for nodeA in [startNode, endNode]:
            for nodeB in self.nodes:
                if self.Boundaries.isValidEdge(nodeA.longLat, nodeB.longLat):
                    nodeA.addEdge(self.Edge(nodeA, nodeB))
                    nodeB.addEdge(self.Edge(nodeB, nodeA)) #TODO: ensure no repeated edges occur
                    
        return startNode, endNode
          
    # Once a search has been completed, reset all class variables           
    def flush(self, start, end, constraint):
        self.ignoreEdgeMatrix = []
        
        startNode = self.Node(start)
        endNode = self.Node(end)
        
        self.nodes.remove(startNode)
        self.nodes.remove(endNode)