from geojson import FeatureCollection, Feature, LineString, Point
import geojson
from turfpy.measurement import boolean_point_in_polygon
import copy
import long_lat
from typing import List


# class is representation of boundary and known obstacles in the park
class Boundaries:

    def __init__(self, fname):

        gj = self.read_file(fname)
        self.boundary_polygon = gj['features'][0]   # polygon
        self.obstacle_polygons = gj['features'][1:]  # list of polygons
        self.boundary_linestring = self.polygon_to_linestring(self.boundary_polygon)
        self.obstacle_linestring = [self.polygon_to_linestring(polygon) for polygon in self.obstacle_polygons]

    def read_file(self, fname):
        with open(fname) as file:
            gj = geojson.load(file)
            return gj

    def polygon_to_linestring(self, polygon):
        linestring = copy.deepcopy(polygon)
        linestring['geometry']['type'] = 'LineString'
        linestring['geometry']['coordinates'] = linestring['geometry']['coordinates'][0]
        return linestring

#todo isIntersecting, geojson polygon, generate grid

    # method returns true iff point is in the boundary and in none of the obstacles
    def is_valid_point(self, longLat):
        point = longLat.to_point()
        return boolean_point_in_polygon(point, self.boundary_polygon) and\
            not(self.point_in_polygons(self.obstacle_polygons, point))

    def point_in_polygons(self, polygons, point):
        for poly in polygons:
            if boolean_point_in_polygon(point, poly):
                return True
        return False


    # method checks if line made from two longlat points intersects any boundary or obstacle
    def isIntersecting(self, longLat1, longLat2):
        lineString = longLat1.to_LineString(longLat2)


    def isValidEdge(self, longLat1, longLat2):
        ...


    def generatePoints() -> List[long_lat.LongLat]:
        #at least length of robot
        ...
        
    def crossesBoundaries():
        ...
