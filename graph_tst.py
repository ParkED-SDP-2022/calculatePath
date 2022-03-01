from long_lat import LongLat
from graph import *

import unittest

class DummyBoundary:
    
    def __init__(self):
        self.x = 0
    
    def generatePoints(self):
        return [LongLat(1,1), LongLat(0,0), LongLat(0,2)]
        
    def isValidEdge(self, e):
        return True
        #return not (e == Graph.Edge(Graph.SearchNode(LongLat(0,2)), Graph.SearchNode(LongLat(0,0))))
    
class TestGraph(unittest.TestCase):
    
    def test_SearchNode_eq(self):
        a = Graph.SearchNode(LongLat(0,3))
        b = Graph.SearchNode(LongLat(1,1))
        c = Graph.SearchNode(LongLat(0,3))
        self.assertEqual(a==b, False)
        self.assertEqual(a==c, True)
    
    def test_GetPath(self):
        graph = Graph(DummyBoundary())
        got = graph.GetPath(LongLat(0,0), LongLat(0,2), [])
        want = [Graph.SearchNode(LongLat(0,0)), Graph.SearchNode(LongLat(0,2))]
        self.assertEqual(got, want, "Should be direct path")
            
    def test_GetPath_constrained(self):
        graph = Graph(DummyBoundary())
        got = graph.GetPath(LongLat(0,0), LongLat(0,2), [Graph.Edge(Graph.SearchNode(LongLat(0,2)), Graph.SearchNode(LongLat(0,0)))])
        want = [Graph.SearchNode(LongLat(0,0)), Graph.SearchNode(LongLat(1,1)), Graph.SearchNode(LongLat(0,2))]
        self.assertEqual(got, want, "Should be longer path")
    
if __name__ == '__main__':
    unittest.main()