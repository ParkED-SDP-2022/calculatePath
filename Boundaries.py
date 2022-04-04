import json
from shapely.geometry import Point, Polygon, LineString, shape, MultiPolygon
import copy
from long_lat import LongLat
from typing import List
import math
import matplotlib
matplotlib.use('PDF')
import matplotlib.pyplot as plt


# class is representation of boundary and known obstacles in the park
class Boundaries:

    def __init__(self, fname):
        self.robot_size_in_coords = 0.08
        self.origin = None

        # read file as shapely.geometry.MultiPolygon object
        multi_polygon = self.read_file(fname)

        # 0th polygon is the boundary, all of the rest are obstacles
        self.boundary_polygon = multi_polygon.geoms[0]
        self.obstacle_polygon = multi_polygon.geoms[1:]

        # create buffer zones
        # a negative distance (first argument in buffer()) represents erosion which is used for the boundary
        self.boundary_buffer = self.boundary_polygon.buffer(-self.robot_size_in_coords*1, single_sided=False)

        # positive distance is dilation which is used for each obstacle
        self.obstacle_buffer = MultiPolygon([MultiPolygon([obstacle.buffer(self.robot_size_in_coords*(1.3), single_sided=False) for obstacle in self.obstacle_polygon.geoms]).buffer(0)])

        # plot boundaries and buffers
        self.dot_color = '#6ca85e'
        self.buffer_color = '#9a9ca1'
        self.boundary_color = '#373d3a'
        self.plot_polygon(self.boundary_buffer, self.buffer_color, 'dashed')
        self.plot_polygon(self.boundary_polygon, self.boundary_color, 'solid')
        self.plot_polygons(self.obstacle_polygon, self.boundary_color, 'solid')
        self.plot_polygons(self.obstacle_buffer, self.buffer_color, 'dashed')

        self.grid = self.generate_grid()


    def plot_polygons(self, polygons, color, linestyle):
        for poly in polygons.geoms:
            self.plot_polygon(poly, color, linestyle)

    def plot_polygon(self, polygon, color, linestyle):
        plt.plot(*polygon.exterior.xy, color=color, linestyle=linestyle)

    def show_plot(self):
        plt.savefig('global_path.png')

    # method reads geojson file and returns a shapely.geometry.MultiPolygon object
    def read_file(self, fname):
        with open(fname) as file:
            features = (json.load(file)["features"])
            multi_polygon = MultiPolygon([shape(feature["geometry"]) for feature in features])
            return multi_polygon

    # method returns true iff point is in the boundary and in none of the obstacles

    def is_valid_point(self, longLat):
        point = longLat.to_point()
        safely_in_boundary = self.boundary_buffer.contains(point)
        not_touching_obstacle = not(self.point_in_polygons(self.obstacle_buffer, point))
        return safely_in_boundary and not_touching_obstacle

    # iterates over multiple polygons, returns true if the given point is contained in any of them
    def point_in_polygons(self, polygons, point):
        for poly in polygons.geoms:
            if poly.contains(point):
                return True
        return False

    # checks if a line made from two longlat points crosses the border of the boundary or any obstacle
    def is_intersecting(self, longLat1, longLat2):
        linestring = longLat1.to_LineString(longLat2)
        return linestring.crosses(self.boundary_buffer) or linestring.crosses(self.obstacle_buffer)

    def is_valid_edge(self, longLat1, longLat2):
        return not(self.is_intersecting(longLat1, longLat2))

    # generates a 2d array of LongLats/int. The value -999 indicates that the point is not in a valid position
    def generate_grid(self):
        x1_bound = self.boundary_polygon.bounds[0]
        y1_bound = self.boundary_polygon.bounds[1]
        x2_bound = self.boundary_polygon.bounds[2]
        y2_bound = self.boundary_polygon.bounds[3]

        # init width and height of the array
        width = math.floor((x2_bound - x1_bound)/self.robot_size_in_coords)
        height = math.floor((y2_bound - y1_bound)/self.robot_size_in_coords)

        x = x1_bound
        y = y2_bound
        
        self.origin = (x,y)

        grid = [[-999 for j in range(width)] for i in range(height)]  # init array

        # adds LongLat point to the grid if it is a valid point
        for i in range(height):
            if i != 0:
                y = y - self.robot_size_in_coords
            x = x1_bound
            for j in range(width):
                if j != 0:
                    x = x + self.robot_size_in_coords
                point = LongLat(x,y)
                if self.is_valid_point(point):
                    grid[i][j] = LongLat(x, y)
                    plt.scatter(x,y, color=self.dot_color)
        #self.show_plot()
        return grid

