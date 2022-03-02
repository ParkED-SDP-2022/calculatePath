from Boundaries import Boundaries
from graph import Graph
from long_lat import LongLat
from boundary_test import BoundaryTest
import matplotlib.pyplot as plt


# Press the green button in the gutter to run the script.
if __name__ == '__main__':
    G = Boundaries('Maps/sdp_space_map.geojson')

    valid_point1 = LongLat(-3.19627046585083,55.941780007088)
    valid_point2 = LongLat(3.196860551834106,55.94188816036847)
    valid_point3 = LongLat(-3.1984376907348633,55.94196627088323)

    start = LongLat(-2.65869140625,
          1.1864386394452024)
    end = LongLat (-0.19775390625,
          2.4272521703917294)
    
    search = Graph(G)
    path = (search.GetPath(start =start, end=end))
    
    plot_line_strings = []
    for i in range(len(path) - 1):
        plot_line_strings.append((path[i].longLat).to_LineString(path[i+1].longLat))
        
    for ls in plot_line_strings:
        x,y = ls.xy
        plt.plot(x,y, color="#ff7040", linewidth=2, solid_capstyle='round')
        
    G.show_plot()
        