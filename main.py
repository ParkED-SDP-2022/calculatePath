from geojson import FeatureCollection, Feature, LineString, Point
import geojson
import os
#class is representation of boundary and known obstacles in the park
class Boundaries:

    def __init__(self):
        self.boundaryCoordinates = 0  # list of points
        self.obstacles = 0
        self.readBoundaries()

    def readBoundaries(self):
        with open('Maps/testMap.geojson') as file:
            gj = geojson.load(file)
            print(gj)
            self.boundaryCoordinates = gj['features'][0]


# Press the green button in the gutter to run the script.
if __name__ == '__main__':
    G = Boundaries()
    print(G.boundaryCoordinates)

# See PyCharm help at https://www.jetbrains.com/help/pycharm/
