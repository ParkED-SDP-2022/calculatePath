import datetime
from geojson import Point, Feature, FeatureCollection
import math
time_resolution = 0.1  # fiddling with this value is encouraged

# hello! long_lat is just going to be a tuple(float, float) for simplicitiy's sake.
# I hope this doesn't feel too complicated :^)
class HeatMapList:


    def __init__(self, number_of_benches):

        poss = [(0.5,0.5,-999),(0.4641,1.1315,-999), (0.0933,0.3927,-999), (0.4943,0.0576,-999)]

        self.bench_dict = {}
        self.candidate_dict = {}
        self.occupy_list = []

        for i in range(number_of_benches):
            self.bench_dict[i+1] = Bench(i+1, (poss[i][0],poss[i][1]), poss[i][2], False)
            self.candidate_dict[i+1] = None

    # method called when the bench status changes.
    def change_state(self, bench_id, long_lat, time: datetime, sit_down, battery=100):
        if sit_down:
            self.candidate_dict[bench_id] = Occupy(time, long_lat)  # creates new Occupy obj when sat on
            self.bench_dict[bench_id].change_state(long_lat, battery, sit_down)
        else:  # completes this obj with end_time and duration when sat off and adds to list
            if self.candidate_dict[bench_id] is None:
                self.bench_dict[bench_id].change_state(long_lat, battery, sit_down)
                print('Error: invalid bench state - process can carry on but heatMap is now inaccurate')
                print('This might have been caused by an invalid initialisation - all benches should begin with sit_down == False')
            else:
                self.candidate_dict[bench_id].add_end_time(time)
                self.occupy_list.append(self.candidate_dict[bench_id])
                self.bench_dict[bench_id].change_state(long_lat, battery, sit_down)
                
    def change_loc(self, bench_id, long_lat):
        self.bench_dict[bench_id].change_loc((long_lat.long, long_lat.lat))

    # returns a feature collection of points from the occupy list between a specified window
    def to_geojson_heatmap(self, start_time_window=datetime.datetime.min, end_time_window=datetime.datetime.max):
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

    def to_geojson_benchstate(self):
        features = []
        for key in self.bench_dict.keys():
            features.append(self.bench_dict[key].to_geojson_feature())
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

class Bench:
    def __init__(self, bench_id: int, long_lat, battery=100, sit_down=False):
        self.bench_id = bench_id
        self.battery = battery
        self.sit_down = sit_down
        self.long_lat = long_lat

    def change_state(self, long_lat, battery, sit_down):
        self.long_lat = long_lat
        self.sit_down = sit_down
        self.battery = battery
        
    def change_loc(self, long_lat):
        self.long_lat = long_lat

    def to_geojson_feature(self):
        properties = {"benchName": self.bench_id,
                      "battery": self.battery,
                      "inUse": self.sit_down}

        point = Point((self.long_lat[0], self.long_lat[1]))

        return Feature(geometry=point, properties=properties)



