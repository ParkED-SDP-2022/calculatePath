from heat_map import HeatMapList
import datetime
test_dt1 = datetime.datetime(2021, 3, 17, 13, 27, 0)
test_dt2 = datetime.datetime(2021, 3, 17, 13, 27, 20)
test_dt3 = datetime.datetime(2021, 3, 17, 13, 27, 30)
test_dt4 = datetime.datetime(2021, 3, 17, 13, 27, 50)

heat_map = HeatMapList(3)
heat_map.change_state(1, (9,9), test_dt1, True)
heat_map.change_state(2, (10, 19), test_dt2, True)
heat_map.change_state(1, (9, 9), test_dt3, False)
heat_map.change_state(2, (10, 19), test_dt4, False)

print(heat_map.to_geojson_heatmap())
print('')
print(heat_map.to_geojson_benchstate())


