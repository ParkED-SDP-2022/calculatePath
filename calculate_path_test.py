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

    def test_path_time(self):
        start_time = timeit.default_timer()
        self.graph.GetPath(self.valid_point1, self.valid_point2)
        stop_time = timeit.default_timer()  # finds the runtime of GetPath method
        duration = stop_time - start_time
        print("duration: " + str(duration))
        self.assertTrue(duration <= 1)


if __name__ == '__main__':
    unittest.main()