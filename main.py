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

    constraints = [(LongLat(-0.9000000000000004,3.1000000000000005),
                                         LongLat(-0.9000000000000004,2.8000000000000007))]
   # constraints = []
    
    search = Graph(G)
    path = (search.GetPath(start =start, end=end,
                           constraints=constraints))

    if constraints:
        for pair in constraints:
            midpoint = pair[0].midpoint(pair[1])
            plt.scatter(midpoint.long, midpoint.lat, marker='H', color='red', s=300)



    plot_line_strings = []
    for i in range(len(path) - 1):
        plot_line_strings.append((path[i].longLat).to_LineString(path[i+1].longLat))
        
    for ls in plot_line_strings:
        x,y = ls.xy
        plt.plot(x,y, color="#ff7040", linewidth=2, solid_capstyle='round')

    G.show_plot()
        