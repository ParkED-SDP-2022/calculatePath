import unittest
import timeit
from graph import Graph
from Boundaries import Boundaries
from long_lat import LongLat

class TestGetPath(unittest.TestCase):

    def setUp(self):
        self.boundaries = Boundaries('sdp_space_map.geojson')
        self.graph = Graph(self.boundaries)

        self.valid_point1 = LongLat(-0.32958984375, 2.273573022378629)
        self.valid_point2 = LongLat(-2.548828125, 2.833317196855306)
        self.valid_point3 = LongLat(-0.59326171875, 3.392790869678844)
        self.valid_point4 = LongLat(-0.72509765625, 0.4614207935306211)

        self.point_in_obstacle = LongLat(-1.746826171875, 2.997898741103057)
        self.point_out_of_bounds = LongLat(0.9667968749999999, 2.4162756547063857)
        self.point_very_close_to_boundary = LongLat(-2.999922037124634, 3.4351588214774966)

        self.constraints = [(LongLat(-0.9000000000000004, 3.1000000000000005),
                             LongLat(-0.9000000000000004, 2.8000000000000007))]

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
        path = self.graph.GetPath(self.valid_point1, self.valid_point2, self.constraints)
        for i in range(len(path) - 1):
            edge = None
            if i < (len(path) - 2):
                pair = (path[i], path[i+1])
            self.assertFalse(edge in self.constraints)


if __name__ == '__main__':
    unittest.main()