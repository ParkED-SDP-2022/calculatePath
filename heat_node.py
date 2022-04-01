#!/usr/bin/env python
import rospy
from parked_custom_msgs.msg import Point
from std_msgs.msg import String, Bool
from heat_map import HeatMapList # todo we need to change message type to handle bench_id
import datetime

class HeatNode:
    def __init__(self):
        self.heat_map = HeatMapList(1)
        # subscribes to pressure sensor data. The payload of the the message will just be a boolean
        self.bench1_pressure_subscriber = rospy.Subscriber('/bench_1_pressure_data', Bool,
                                                    self.pressure_subscriber_cb, callback_args=[1])
        self.bench2_pressure_subscriber = rospy.Subscriber('/bench_2_pressure_data', Bool,
                                                    self.pressure_subscriber_cb, callback_args=[2])
        self.bench3_pressure_subscriber = rospy.Subscriber('/bench_3_pressure_data', Bool,
                                                    self.pressure_subscriber_cb, callback_args=[3])
        self.bench4_pressure_subscriber = rospy.Subscriber('/bench_4_pressure_data', Bool,
                                                    self.pressure_subscriber_cb, callback_args=[4])

        # subscribes to robot's current position. The payload of the message will be a parked_custom_msgs/Point object
        self.position_subscriber = rospy.Subscriber('/robot_position_longlat', Point, self.update_current_position, callback_args=1)
        # self.position_subscriber = rospy.Subscriber('/robot_positon_2', Point, self.update_current_position, callback_args=2)
        # self.position_subscriber = rospy.Subscriber('/robot_positon_3', Point, self.update_current_position, callback_args=3)
        # self.position_subscriber = rospy.Subscriber('/robot_positon_4', Point, self.update_current_position, callback_args=4)
        self.bench1_current_position = Point(0.6,0.6,-999)
        self.bench2_current_position = Point(1.6,1.7,-999)
        self.bench3_current_position = Point(-0.3,-0.2,-999)
        self.bench4_current_position = Point(0.5,0.5,-999)

        # publishes string notation of geojson
        self.heat_map_publisher = rospy.Publisher('/heat_map_data', String, queue_size=5)
        self.publisher_msg = String()

        rospy.init_node('heat_node', anonymous=True)
        rospy.spin()


    def pressure_subscriber_cb(self, pressure_data, args):
        # if self.current_position == None:
        #     return
        bench_id = args[0]
        longitude = 0.5
        latitude = 0.5
        if bench_id == 1:
            print("seeing bench 1")
            longitude = self.bench1_current_position.long
            latitude = self.bench1_current_position.lat
        elif bench_id == 2:
            print("seeing bench 2")
            longitude = self.bench2_current_position.long
            latitude = self.bench2_current_position.lat
        elif bench_id == 3:
            print("seeing bench 2")
            longitude = self.bench3_current_position.long
            latitude = self.bench3_current_position.lat
        elif bench_id == 4:
            print("seeing bench 2")
            longitude = self.bench4_current_position.long
            latitude = self.bench4_current_position.lat
        sit_down = pressure_data.data
        time = datetime.datetime.now()
        long_lat = (longitude, latitude)

        # todo we need to use this bench_id in the calling of these methods
        self.heat_map.change_state(bench_id, long_lat, time, sit_down)

        if not sit_down:
            geojson_heatmap = str(self.heat_map.to_geojson_heatmap())
            #geojson_benchstate = self.heat_map.to_geojson_benchstate() # todo let's not forget to publish this
            self.publisher_msg.data = geojson_heatmap
            self.heat_map_publisher.publish(self.publisher_msg)

        # todo we need to make use of the to_geojson_benchstate() method publishing its fruits to frontend
    
    def update_current_position(self, data, bench_id):
        if bench_id == 1:
            self.bench1_current_position = data
        elif bench_id == 2:
            self.bench2_current_position = data
        elif bench_id == 3:
            self.bench3_current_position = data
        elif bench_id == 4:
            self.bench4_current_position = data
    
if __name__ == '__main__':
    hn = HeatNode()





