from long_lat import LongLat
import heapq
import math

class Graph:    
    
    # Node Type for use in A* Search 
    class SearchNode:
        
        # initialise a search node
        def __init__(self, ll : LongLat, i, j):
            self.i = i
            self.j = j
            self.longLat = ll
            self.edges = []
            self.parent = None
            self.g = float('inf')
            self.f = float('inf')
        
        # add a list of edges to the node
        def addEdges(self, es):
            # Only include an edge once
            for e in es:
                if e not in self.edges:
                    self.edges.append(e)
            
        # Nodes are equivalent if they are in the same location
        def __eq__(self, other):
            return (self.i) == (other.i) and self.j == other.j
        
        # a node is less than than another if its f value is shorter (used for heap representation)
        def __lt__(self, other):
            return self.f<other.f
        
        def __str__(self):
            result = "{(" + str(self.i) + "," + str(self.j) + ")<-" 
            if not self.parent:
                result += "None"
            else:
                result += "(" + str(self.parent.i) + "," + str(self.parent.j) + ")"
                
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
        nodes = {}
        pointMatrix = self.Boundaries.generate_grid()
        for i in range(len(pointMatrix)):
            for j in range(len(pointMatrix[0])):
                if pointMatrix[i][j] != -999:
                    nodes[(i,j)] = self.SearchNode(pointMatrix[i][j], i, j)
        return nodes
    
    def getNode(self, i, j):
        return self.nodes[(i,j)]
        
    # for all node, check if an edge exists between all other nodes by calling bounary method to check intersection 
    # add the edges to a list in the node object 
    def initializeEdges(self):
        for key in self.nodes:
            self.nodes[key].addEdges(self.generateEdges(self.nodes[key]))
    
    # validate edge
    def generateEdges(self, A : SearchNode):
        edges = []
        for i in range(A.i-1, A.i+2):
            for j in range(A.j-1, A.j+2):
                if A.i == i and A.j == j:
                    # never have an edge to yourself
                    continue
                if (i,j) in self.nodes:
                    if self.Boundaries.is_valid_edge(A.longLat, self.nodes[(i, j)].longLat):
                        edges.append(self.Edge(A, self.nodes[(i, j)]))
        return edges
                
        
    
    # given a start, end and a constraing, use A* search to find the optimum path through the graph
    # TODO: decide how to represent constraint
    def GetPath(self, start : LongLat, end : LongLat, constraint=[]):
        
        #TODO: determine when to flush the ignore matrix (e.g. stuck looping between two paths if remove immediately)
        self.applyConstraint(constraint)
        
        #Add beginning and end as nodes to the graph
        startNode, goalNode = self.addStartEnd(start, end)
        # self.nodes[(goalNode.i, goalNode.j)] = goalNode
        
        result = self.aStar(startNode, goalNode)
        
        # remove temporary modifications to graph
        self.flush(startNode, goalNode, constraint) # TODO: can I flush before constructing the path? probably not...
        return(self.reconstructPath(result))
    
    def aStar(self, startNode, goalNode):
        # Initialise the priority queue of nodes to visit
        toVisit = []
        visited = []
        heapq.heappush(toVisit, startNode)
        
        while len(toVisit) > 0:
            #TODO: handle timeouts, start=end, etc
            
            # Look at all children of the node with the lowest f value and update them if necessary
            currentNode = heapq.heappop(toVisit) 
            if currentNode == goalNode:
                return currentNode
            
            for edge in currentNode.edges:
                if self.isIgnoredEdge(edge) or (edge.getOtherEnd(currentNode).i, edge.getOtherEnd(currentNode).j) not in self.nodes:
                    continue
                
                child = edge.getOtherEnd(currentNode) #TODO: make sure to use the right end of the edge here
                # If the child is a goal, we have found the shortest path
                # if child == goalNode:
                #     print(child)
                #     child.parent = currentNode
                #     return child
                # # Otherwise update the child if we have found a shorter route to it than previously
                # else:
                if child not in toVisit and child not in visited:
                    heapq.heappush(toVisit, child)
                    child.parent = currentNode
                    child.g = currentNode.g + edge.cost
                    child.f = child.g + child.longLat.distance(goalNode.longLat)
                else:
                    if child.g > currentNode.g + edge.cost:
                        child.parent = currentNode
                        child.g = currentNode.g + edge.cost
                        child.f = child.g + child.longLat.distance(goalNode.longLat)
                        if child in visited:
                            visited.remove(child)
                            heapq.heappush(toVisit, child)
            heapq.heappush(visited, currentNode)
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
        # for k in self.nodes:
        #     print(str(self.nodes[k]))
        #return 0
        if not goal:
            return None
        
        path = [] 
        currentNode = goal
        i = 0 
        while not( not currentNode) and  i < 10:
            print(currentNode)
            path.insert(0, currentNode)
            currentNode = currentNode.parent
            i += 1
            
        #print(listToStr(path))
        return path
        
    # Add the start and goal locations to the graph, connecting them via valid edges
    def addStartEnd(self, start : LongLat, end : LongLat):
        max_dist_to_node = math.sqrt(2*(self.Boundaries.robot_size_in_coords**2))
                       
        endNode = self.SearchNode(end, -5, -5)
        endNode.g = 0
        endNode.f = 0 
        
        startNode = self.SearchNode(start, -10, -10)
        startNode.g = 0
        startNode.f = startNode.longLat.distance(endNode.longLat)                   

        self.removeNodeByLongLat(startNode)
        self.removeNodeByLongLat(endNode)
        self.nodes[(endNode.i, endNode.j)] = endNode
        
        for A in [startNode, endNode]:
            for key in self.nodes:
                B = self.nodes[key]
                if abs(B.longLat.distance(A.longLat)) <= max_dist_to_node:
                    if self.Boundaries.is_valid_edge(A.longLat, B.longLat):
                        e = self.Edge(A, B)
                        A.addEdges([e])
                        B.addEdges([e]) 
            
        return startNode, endNode
    
    def removeNodeByLongLat(self, A):
        removeKey = None
        for key in self.nodes:
            B = self.nodes[key]
            if B.longLat == A.longLat:
                removeKey = (B.i, B.j)
        if removeKey:
            self.nodes.pop(removeKey)
        
          
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