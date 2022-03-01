from shapely.geometry import Point
from long_lat import LongLat

class BoundaryTest:

    def __init__(self, b):
        self.b = b
        self.p_inside_1 = LongLat(-3.197590112686157,55.941978287871514)
        self.p_inside_2 = LongLat(-3.196796178817749,55.941900177381)
        self.p_outside_1 = LongLat(-3.1946504116058345,55.941545673944226)
        self.p_in_obstacle_1 = LongLat(-3.196152448654175,55.94119717587758)

        print(str(self.test_is_valid_point_1()) + " is_valid_point_1" )
        print(str(self.test_is_valid_point_2()) + " is_valid_point_2" )
        print(str(self.test_is_valid_point_3()) + " is_valid_point_3" )
        print(str(self.test_is_intersecting_1()) + " is_intersecting_1")
        print(str(self.test_is_intersecting_2()) + " is_intersecting_2")
        print(str(self.test_is_intersecting_3()) + " is_intersecting_3")


    def test_is_valid_point_1(self):
        expect = True
        return self.b.is_valid_point(self.p_inside_1) == expect

    def test_is_valid_point_2(self):
        expect = False
        return self.b.is_valid_point(self.p_outside_1) == expect

    def test_is_valid_point_3(self):
        expect = False
        return self.b.is_valid_point(self.p_in_obstacle_1) == expect

    def test_is_intersecting_1(self):
        expect = False
        return self.b.is_intersecting(self.p_inside_1, self.p_inside_2) == expect

    def test_is_intersecting_2(self):
        expect = True
        return self.b.is_intersecting(self.p_inside_1, self.p_outside_1) == expect

    def test_is_intersecting_3(self):
        expect = True
        return self.b.is_intersecting(self.p_inside_1, self.p_in_obstacle_1) == expect





