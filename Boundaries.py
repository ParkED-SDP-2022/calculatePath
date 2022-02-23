import json
from shapely.geometry import Point, Polygon, LineString, shape, MultiPolygon
import copy
from long_lat import LongLat
from typing import List
import math

# class is representation of boundary and known obstacles in the park
class Boundaries:

    def __init__(self, fname):

        multi_polygon = self.read_file(fname)

        self.boundary_polygon = multi_polygon.geoms[0]
        self.obstacle_polygon = multi_polygon.geoms[1:]
        print(self.boundary_polygon)
        print("\n")
        print(self.obstacle_polygon)


    # method reads geojson file and returns a shapely.geometry.MultiPolygon object
    def read_file(self, fname):
        with open(fname) as file:
            features = (json.load(file)["features"])
            multi_polygon = MultiPolygon([shape(feature["geometry"]) for feature in features])
            return multi_polygon

#todo isIntersecting, geojson polygon, generate grid

    # method returns true iff point is in the boundary and in none of the obstacles
    def is_valid_point(self, longLat):
        point = longLat.to_point()
        return self.boundary_polygon.contains(point) and not(self.obstacle_polygon.contains(point))

    # iterates over multiple polygons, returns true if the given point is contained in any of them
    def point_in_polygons(self, polygons, point):
        for poly in polygons:
            if poly.contains(point):
                return True
        return False

    # checks if a line made from two longlat points crosses the border of the boundary or any obstacle
    def is_intersecting(self, longLat1, longLat2):
        linestring = longLat1.to_LineString(longLat2)
        return linestring.crosses(self.boundary_polygon) or linestring.crosses(self.obstacle_polygon)

    def is_valid_edge(self, longLat1, longLat2):
        ...


    def generate_points(self):
        #robot_size_in_coords = 0.0000027027
        robot_size_in_coords = 0.0001
        x1_bound = self.boundary_polygon.bounds[0]
        y1_bound = self.boundary_polygon.bounds[1]
        x2_bound = self.boundary_polygon.bounds[2]
        y2_bound = self.boundary_polygon.bounds[3]

        # init width and height of the array
        width = math.floor((x2_bound - x1_bound)/robot_size_in_coords)
        height = math.floor((y2_bound - y1_bound)/robot_size_in_coords)

        x = x1_bound
        y = y2_bound

        grid = [[-999 for j in range(width)] for i in range(height)] # init array

        # adds LongLat point to the grid if it is a valid point
        for i in range(height):
            if i != 0:
                y = y - robot_size_in_coords
            x = x1_bound
            for j in range(width):
                if j != 0:
                    x = x + robot_size_in_coords
                point = LongLat(x,y)
                if self.is_valid_point(point):
                    grid[i][j] = LongLat(x, y)

        print(grid)
        print(len(grid[0]))
        print(x1_bound)
        print(y2_bound)

