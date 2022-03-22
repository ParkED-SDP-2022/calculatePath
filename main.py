from Boundaries import Boundaries
from graph import Graph
from long_lat import LongLat
from boundary_test import BoundaryTest
import matplotlib.pyplot as plt
import timeit



# Press the green button in the gutter to run the script.
if __name__ == '__main__':
    # warning, if you change the map to the updated one
    # you will have to change the points that main.py and calculate_path_test.py run on
    # otherwise it will return None values for every path. I will change this soon, love from rory x
     G = Boundaries('/afs/inf.ed.ac.uk/user/s18/s1829279/Desktop/sdp/catkin_ws/src/calculatePath/sdp_demo_space_from_camera.geojson')

    # valid_point1 = LongLat(-3.19627046585083,55.941780007088)
    # valid_point2 = LongLat(3.196860551834106,55.94188816036847)
    # valid_point3 = LongLat(-3.1984376907348633,55.94196627088323)
    #
    # start = LongLat(-2.65869140625,
    #                 1.1864386394452024)
    #
    # end = LongLat (-0.19775390625,
    #                 2.4272521703917294)

    start = LongLat(0.406494140625, 1.0711045990129324)
    end = LongLat(0.7772827148437499, 0.1977535136255067)

    constraints = [(LongLat(0.5147368421052632,0.3210526315433332),
                   LongLat(0.5147368421052632,0.42105263154333317))]


    start = LongLat(0.2032470703125,
          1.0326589311777885)
    end = LongLat( 0.84869384765625,
          0.6838826866130326)
    #end = LongLat(0.19775390625,
            #0.4272521703917294)

    #end = LongLat(0.7772827148437499, 0.1977535136255067)



    # constraints = [(LongLat(-0.9000000000000004,3.1000000000000005),
    #                 LongLat(-0.9000000000000004,2.8000000000000007))]

    constraints = []

    search = Graph(G)


    path = (search.GetPath(start=start, end=end,
                           constraints=constraints))


    if constraints:
        for pair in constraints:
            midpoint = pair[0].midpoint(pair[1])
            plt.scatter(midpoint.long, midpoint.lat, marker='H', color='red', s=300)



    plot_line_strings = []
    for i in range(len(path) - 1):
        plot_line_strings.append(path[i].longLat.to_LineString(path[i + 1].longLat))
        
    for ls in plot_line_strings:
        x,y = ls.xy
        plt.plot(x,y, color="#ff7040", linewidth=2, solid_capstyle='round')

    G.show_plot()
        