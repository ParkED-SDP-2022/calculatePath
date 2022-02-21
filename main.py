from boundaries import Boundaries
from long_lat import LongLat


# Press the green button in the gutter to run the script.
if __name__ == '__main__':
    G = Boundaries('Maps/testMap.geojson')
    print(G.boundary_polygon)
    print(G.obstacle_polygons)
    print(G.obstacle_linestring)
    print(G.boundary_linestring)
    l = LongLat(1, 1) #false
    l2 = LongLat(-3.197085857391357,
          55.94205038972295) #true
    l3 = LongLat( -3.196130990982055,
          55.94126327057947) #false
    l4 = LongLat(-3.197418451309204,
          55.94164181079011) #false
    l5 = LongLat( -3.195412158966064,
          55.9416237851497) #outside boundary
    print(G.is_intersecting(l4, l5))

