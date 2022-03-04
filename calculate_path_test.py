import unittest
import timeit
from graph import Graph
from Boundaries import Boundaries
from long_lat import LongLat

class calculate_path_test(unittest.TestCase):

    def setUp(self):
        self.boundaries = Boundaries('Maps/sdp_space_map.geojson')
        self.graph = Graph(self.boundaries)

        self.valid_point1 = LongLat(-0.32958984375, 2.273573022378629)
        self.valid_point2 = LongLat(-2.548828125, 2.833317196855306)

        self.point_in_obstacle = LongLat(-1.746826171875, 2.997898741103057)
        self.point_out_of_bounds = LongLat(0.9667968749999999, 2.4162756547063857)

    def test_GetPath_duration(self):
        start_time = timeit.default_timer()
        self.graph.GetPath(self.valid_point1, self.valid_point2)
        stop_time = timeit.default_timer()  # finds the runtime of GetPath method
        duration = stop_time - start_time
        self.assertTrue(duration <= 0.01)

    def test_handles_start_out_of_bounds(self):
        path = self.graph.GetPath(self.point_out_of_bounds, self.valid_point1)
        self.assertTrue(path is None)

    def test_handles_end_in_obstacle(self):
        path


if __name__ == '__main__':
    unittest.main()