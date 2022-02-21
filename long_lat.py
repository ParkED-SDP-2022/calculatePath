import math
from geojson import Point, LineString

class LongLat:

    def __init__(self, long, lat):
        self.long = long
        self.lat = lat

    def distance(self, longLat):
        x1 = self.long
        y1 = self.lat
        x2 = longLat.long
        y2 = longLat.lat

        distance = math.sqrt((x1 - x2)**2 + (y1 - y2)**2)

        return distance

    def to_point(self):
        return Point((self.long, self.lat))

    def to_LineString(self, longLat):
        x1 = self.long
        y1 = self.lat
        x2 = longLat.long
        y2 = longLat.lat

        return LineString([(x1, y1), (x2, y2)])
    
    def __eq__(self, other):
        return self.long == other.long and self.lat == other.lat

