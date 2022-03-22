import datetime
from geojson import Point, Feature, FeatureCollection
import math
time_resolution = 0.1  # fiddling with this value is encouraged

# hello! long_lat is just going to be a tuple(float, float) for simplicities sake.
# I hope this doesn't feel too complicated :^)
class HeatMapList:
    def __init__(self):
        # list of completed Occupy objects
        self.occupy_list = []
        # the Occupy object of a bench currently being sat on
        # it doesn't have an end_time or duration yet
        self.candidate = None
        # list of features

    # method called when the bench status changes.
    def change_state(self, long_lat, time: datetime, sit_down):
        if sit_down:
            self.candidate = Occupy(time, long_lat)  # creates new Occupy obj when sat on
        else:  # completes this obj with end_time and duration when sat off and adds to list
            if self.candidate.long_lat != long_lat:
                # not sure how to handle this state. I think the most graceful thing to
                # do in our situation is just keep going like this.
                print("This probably shouldn't happen but it's not an issue: problem with arguments for change_state" )
            self.candidate.add_end_time(time)
            self.occupy_list.append(self.candidate)

    # returns a feature collection of points from the occupy list between a specified window
    def to_geojson(self, start_time_window=datetime.datetime.min, end_time_window=datetime.datetime.max):
        features = []
        for occupy in self.occupy_list:
            if occupy.start_time >= start_time_window and occupy.end_time < end_time_window:
                point = Point((occupy.long_lat[0], occupy.long_lat[1]))
                feature = Feature(geometry=point)
                # time_resolution is a global constant and should be fiddled with
                # to find the value that gives the best looking results
                for i in range(math.ceil(occupy.duration * time_resolution)):
                    features.append(feature)
        feature_collection = FeatureCollection(features)
        return feature_collection

# Occupy data structure represents a (sit on, get off) bench interaction
class Occupy:
    def __init__(self, start_time: datetime, long_lat):
        self.start_time = start_time
        self.end_time = None
        self.long_lat = long_lat
        self.duration = None

    def add_end_time(self, end_time: datetime):
        self.end_time = end_time
        self.duration = (self.end_time - self.start_time).total_seconds()

