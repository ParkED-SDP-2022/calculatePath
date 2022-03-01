from lib2to3.pytree import Node
from long_lat import LongLat
import heapq

class Graph:    
    
    # Node Type for use in A* Search 
    class SearchNode:
        
        # initialise a search node
        def __init__(self, ll : LongLat):
            self.longLat = ll
            self.edges = []
            self.parent = None
            self.g = float('inf')
            self.f = float('inf')
        
        # add a new edge to the node
        def addEdge(self, e):
            # Only include an edge once
            if e not in self.edges:
                self.edges.append(e)
            
        # Nodes are equivalent if they are in the same location
        def __eq__(self, other):
            return (self.longLat) == (other.longLat)
        
        # a node is less than than another if its f value is shorter (used for heap representation)
        def __lt__(self, other):
            return self.f<other.f
        
        def __str__(self):
            result = "{" + str(self.longLat) + "<-" 
            if not self.parent:
                result += "None"
            else:
                result += str(self.parent.longLat)
                
            return result + "}"
        
    # Edge class representing edges between graph nodes
    class Edge:
        
        # initialise an edge object
        def __init__(self, nodeA, nodeB):
            self.start = nodeA
            self.end = nodeB
            self.cost = nodeA.longLat.distance(nodeB.longLat)
         
        # Given one end of an edge, return the other end   
        def getOtherEnd(self, nodeA):
            if nodeA == self.start:
                return self.end
            if nodeA == self.end:
                return self.start
       
       # Edges are equal if they have ends at the same points
        def __eq__(self, other):
            # Edges should be equal regardles of their direction
            return (self.start == other.start and self.end == other.end) or (self.start == other.end and self.end == other.start)
        
        def __str__(self):
            return str(self.start) + "->" + str(self.end)
    
    
    #initialise a graph object
    def __init__(self, boundaries):
        
        #boundaries of the park 
        self.Boundaries = boundaries
        
        # a dictionary of longlats to nodes in the graph
        self.nodes = self.initializeNodes()
        self.initializeEdges()
        
        # a list of edges which cannot be traversed TODO: make this a set?
        self.ignoreEdgeMatrix = [] #TODO: enable this  

    # generates all nodes in the graph, using self.boundaries object to generate the grid longlats
    def initializeNodes(self):
        nodes = []
        for longlat in self.Boundaries.generatePoints():
            nodes.append(self.SearchNode(longlat))
        return nodes
        
    # for all node, check if an edge exists between all other nodes by calling bounary method to check intersection 
    # add the edges to a list in the node object 
    def initializeEdges(self):
        for nodeA in self.nodes:
            for nodeB in self.nodes:
                #TODO: not repeate calculations of existing edges
                #TODO: what format to validate? node, edge, longlats?
                #TODO: no overlapping edges?
                newEdge = self.Edge(nodeA, nodeB)
                if self.Boundaries.isValidEdge(newEdge) and nodeA != nodeB:
                    nodeA.addEdge(newEdge)
            
            #print("finding edges for node " + str(nodeA) + ":" + listToStr(nodeA.edges ))
    
    # given a start, end and a constraing, use A* search to find the optimum path through the graph
    # TODO: decide how to represent constraint
    def GetPath(self, start, end, constraint=[]):
        
        #TODO: determine when to flush the ignore matrix (e.g. stuck looping between two paths if remove immediately)
        self.applyConstraint(constraint)
        
        #Add beginning and end as nodes to the graph
        startNode, goalNode = self.addStartEnd(start, end)
        
        result = self.aStar(startNode, goalNode)
        
        # toVisit = []
        # heapq.heappush(toVisit, startNode)
        # result = None
        
        # while len(toVisit) > 0:
            
        #     #TODO: handle taking too long or no available path
        #     currentNode = heapq.heappop(toVisit) 
            
        #     for edge in currentNode.edges:
        #         if self.isIgnoredEdge(edge):
        #             continue
                
        #         child = edge.getOtherEnd(currentNode) #TODO: make sure to use the right end of the edge here
               
        #         if child == goalNode:
        #             child.parent = currentNode
        #             result = child
        #             break 
                
        #         else:
        #             temp_g = edge.cost
        #             temp_f = temp_g + child.longLat.distance(goalNode.longLat)
        #             if child.f > temp_f:
        #                 child.parent = currentNode
        #                 child.g = temp_g
        #                 child.f = temp_f
        #                 if child not in toVisit:
        #                     heapq.heappush(toVisit, child)
                                
        #     if result:
        #         break
        
        # remove temporary modifications to graph
        self.flush(startNode, goalNode, constraint) # TODO: can I flush before constructing the path? probably not...
        return(self.reconstructPath(result))
    
    def aStar(self, startNode, goalNode):
        # Initialise the priority queue of nodes to visit
        toVisit = []
        heapq.heappush(toVisit, startNode)
        
        while len(toVisit) > 0:
            #TODO: handle timeouts, start=end, etc
            
            # Look at all children of the node with the lowest f value and update them if necessary
            currentNode = heapq.heappop(toVisit) 
            for edge in currentNode.edges:
                if self.isIgnoredEdge(edge):
                    continue
                
                child = edge.getOtherEnd(currentNode) #TODO: make sure to use the right end of the edge here
                # If the child is a goal, we have found the shortest path
                if child == goalNode:
                    child.parent = currentNode
                    return child
                # Otherwise update the child if we have found a shorter route to it than previously
                else:
                    temp_g = edge.cost
                    temp_f = temp_g + child.longLat.distance(goalNode.longLat)
                    if child.f > temp_f:
                        child.parent = currentNode
                        child.g = temp_g
                        child.f = temp_f
                        if child not in toVisit:
                            heapq.heappush(toVisit, child)
        # If no route is found return none                      
        return None
    
    def applyConstraint(self, c):
        self.ignoreEdgeMatrix.extend(self.constraintToEdges(c))
        
    def constraintToEdges(self, constraint):
        return constraint
        
    def isIgnoredEdge(self, e):
        # should this be a matrix for fast lookup? rn its a list
        return e in self.ignoreEdgeMatrix
        
    # traverse the node parents to reconstruct the optimal route used
    def reconstructPath(self, goal):
        if not goal:
            return None
        
        path = [] 
        currentNode = goal
        while not( not currentNode):
            path.insert(0, currentNode)
            currentNode = currentNode.parent
        return path
        
    # Add the start and goal locations to the graph, connecting them via valid edges
    def addStartEnd(self, start, end):
        
        startNode = self.SearchNode(start)
        startNode.g = 0
        
        endNode = self.SearchNode(end)
        endNode.g = 0
        endNode.f =0 
        
        for nodeA in [startNode, endNode]:
            for nodeB in self.nodes:
                newEdge = self.Edge(nodeA, nodeB)
                if self.Boundaries.isValidEdge(newEdge) and nodeA != nodeB:
                    nodeA.addEdge(newEdge)
                    nodeB.addEdge(newEdge) #TODO: ensure no repeated edges occur
                    
        
        if startNode in self.nodes:
            # tODO: add this back at the end?
            self.nodes.remove(startNode) #TODO: work out what to do here
        if endNode not in self.nodes:
            self.nodes.append(endNode)
            
        return startNode, endNode
          
    # Once a search has been completed, reset all class variables           
    def flush(self, start, end, constraint):
        #self.ignoreEdgeMatrix = []        
        #self.nodes.remove(start)
        #self.nodes.remove(end)
        return
        
def listToStr(l):
    string = "["
    for elem in l:
        string += str(elem) + ","
    string += "]"
    return string