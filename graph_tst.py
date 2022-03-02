from long_lat import LongLat
from graph import *

import unittest

#TODO: test!
class DummyBoundary:
    
    robot_size_in_coords = 2.0
    
    def __init__(self):
        self.x = 0
    
    def generate_grid(self):
        return [[LongLat(0,0), -999, LongLat(0,2)], [-999, LongLat(1,1), -999], [-999, -999, -999]]
        
    def is_valid_edge(self, l1, l2):
        return True
    
class TestGraph(unittest.TestCase):
    
    def test_SearchNode_eq(self):
        a = Graph.SearchNode(LongLat(0,3), 0, 3)
        b = Graph.SearchNode(LongLat(1,1), 1, 1)
        c = Graph.SearchNode(LongLat(0,3), 0, 3)
        self.assertEqual(a==b, False)
        self.assertEqual(a==c, True)
    
    def test_GetPath(self):
        graph = Graph(DummyBoundary())
        got = graph.GetPath(LongLat(0,0), LongLat(0,2), [])
        print("got: " + listToStr(got))
        want = [Graph.SearchNode(LongLat(0,0), -10, -10), Graph.SearchNode(LongLat(0,2), -5, -5)]
        print("want: " + listToStr(want))
        self.assertEqual(got, want, "Should be direct path")
            
    def test_GetPath_constrained(self):
        graph = Graph(DummyBoundary())
        got = graph.GetPath(LongLat(0,0), LongLat(0,2), [Graph.Edge(Graph.SearchNode(LongLat(0,2), 0, 2), Graph.SearchNode(LongLat(0,0), 0, 0))])
        print("got: " + listToStr(got))
        want = [Graph.SearchNode(LongLat(0,0), 0, 0), Graph.SearchNode(LongLat(1,1), 1, 1), Graph.SearchNode(LongLat(0,2), 0, 2)]
        print("want: " + listToStr(want))
        self.assertEqual(got, want, "Should be longer path")
    
if __name__ == '__main__':
    unittest.main()