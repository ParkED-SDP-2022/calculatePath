from Boundaries import Boundaries
from long_lat import LongLat
from boundary_test import BoundaryTest


# Press the green button in the gutter to run the script.
if __name__ == '__main__':
    G = Boundaries('Maps/testMap.geojson')

    valid_point1 = LongLat(-3.19627046585083,55.941780007088)
    valid_point2 = LongLat(3.196860551834106,55.94188816036847)
    valid_point3 = LongLat(-3.1984376907348633,55.94196627088323)
    G.show_plot()