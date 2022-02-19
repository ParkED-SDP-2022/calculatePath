from geojson import FeatureCollection, Feature, LineString, Point
import geojson
import os


# class is representation of boundary and known obstacles in the park
class Boundaries:

    def __init__(self):
        self.boundaryCoordinates = 0  # list of points
        self.obstacles = 0
        self.readBoundaries()

    def readBoundaries(self, fname):
        with open(fname) as file:
            gj = geojson.load(file)
            print(gj)
            self.boundaryCoordinates = gj['features'][0]

#todo isIntersecting, geojson polygon, generate grid

    def generatePoints():
        #at least length of robot
        ...
        
    def crossesBoundaries():
        ...
