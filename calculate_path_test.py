import unittest
import timeit
from graph import Graph
from Boundaries import Boundaries
from long_lat import LongLat

class TestGetPath(unittest.TestCase):

    def setUp(self):
        self.boundaries = Boundaries('/afs/inf.ed.ac.uk/user/s18/s1829279/Desktop/sdp/catkin_ws/src/calculatePath/sdp_demo_space_from_camera_2.geojson')
        self.graph = Graph(self.boundaries)

        self.valid_point1 = LongLat(0.53009033203125,
          1.02167436848739)
        self.valid_point2 = LongLat(0.28701782226562494,
          0.4888856691231101)
        self.valid_point3 = LongLat(0.745697021484375,
          0.17990082744713973)
        self.valid_point4 = LongLat(0.185394287109375,
          0.9571393174703876)

        self.point_in_obstacle = LongLat(0.416107177734375,
          0.8060952321492764)
        self.point_out_of_bounds = LongLat(0.65093994140625,
          1.272936040103872)
        self.point_very_close_to_boundary = LongLat(0.65643310546875,
          0.832185094669236)

        self.constraints = [(LongLat(0.19978947368421052,0.9587368420692498), 
        LongLat(0.2917894736842105,1.0507368420692498))]

    def test_GetPath_duration(self):
        start_time = timeit.default_timer()
        self.graph.GetPath(self.valid_point1, self.valid_point2)
        stop_time = timeit.default_timer()  # finds the runtime of GetPath method
        duration = stop_time - start_time
        self.assertTrue(duration <= 0.01)

    def test_reject_start_out_of_bounds(self):
        path = self.graph.GetPath(self.point_out_of_bounds, self.valid_point1)
        self.assertTrue(path is None)

    def test_reject_end_in_obstacle(self):
        path = self.graph.GetPath(self.valid_point2, self.point_in_obstacle)
        self.assertTrue(path is None)

    def test_path_not_intersecting_obstacle(self):
        path = self.graph.GetPath(self.valid_point2, self.valid_point1)
        for i in range(len(path)):
            if i < len(path) - 2:
                self.boundaries.is_valid_edge(path[i].longLat, path[i+1].longLat)


    def test_nodes_in_path_in_boundary(self):
        path = self.graph.GetPath(self.valid_point3, self.valid_point4)
        for node in path:
            point = node.longLat
            self.assertTrue(self.boundaries.is_valid_point(point))

    def test_reject_node_on_boundary(self):
        path = self.graph.GetPath(self.point_very_close_to_boundary, self.valid_point1)
        self.assertTrue(path is None)

    def test_node_abides_by_constraint(self):
        print("test constraint +==========================")
        path = self.graph.GetPath(self.valid_point1, self.valid_point2, self.constraints)
        for i in range(len(path) - 1):
            pair = None
            if i < (len(path) - 2):
                pair = (path[i].longLat, path[i+1].longLat)
                self.assertFalse(pair in self.constraints)


if __name__ == '__main__':
    unittest.main()