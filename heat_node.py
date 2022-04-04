#!/usr/bin/env python
import rospy
from parked_custom_msgs.msg import Point
from std_msgs.msg import String, Bool
from heat_map import HeatMapList # todo we need to change message type to handle bench_id
import datetime

class HeatNode:
    def __init__(self):
        self.heat_map = HeatMapList(4)
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

        # Publish location data
        self.bench_state_publisher = rospy.Publisher('/bench_state_data', String, queue_size=5)
        self.bench_state_publisher_msg = String()

        rospy.init_node('heat_node', anonymous=True)
        rospy.spin()

    # Updates the 'seated' status of benches when sat on or un-sat on
    def pressure_subscriber_cb(self, pressure_data, args):
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
            longitude = self.bench3_current_position.long
            latitude = self.bench3_current_position.lat
        elif bench_id == 4:
            longitude = self.bench4_current_position.long
            latitude = self.bench4_current_position.lat
        sit_down = pressure_data.data
        time = datetime.datetime.now()
        long_lat = (longitude, latitude)

        # todo we need to use this bench_id in the calling of these methods
        self.heat_map.change_state(bench_id, long_lat, time, sit_down)

        if not sit_down:    #TODO: does this only publish when people stand up? If so, the bench state will never be 'in use'
            geojson_heatmap = str(self.heat_map.to_geojson_heatmap())
            geojson_benchstate = str(self.heat_map.to_geojson_benchstate())
            self.publisher_msg.data = geojson_heatmap
            self.heat_map_publisher.publish(self.publisher_msg)
            self.bench_state_publisher_msg.data = geojson_benchstate
            self.bench_state_publisher.publish(self.bench_state_publisher_msg)
    
    # Updates stored bench positions when they change and publishes new feature collection of benches to frontend
    # TODO: ensure this isn't publishing too often to be feasible
    def update_current_position(self, data, bench_id):
        if bench_id == 1:
            self.bench1_current_position = data
            self.heat_map.change_loc(1, data)
        elif bench_id == 2:
            self.bench2_current_position = data
            self.heat_map.change_loc(2, data)
        elif bench_id == 3:
            self.bench3_current_position = data
            self.heat_map.change_loc(3, data)
        elif bench_id == 4:
            self.bench4_current_position = data
            self.heat_map.change_loc(4, data)
            
        geojson_benchstate = str(self.heat_map.to_geojson_benchstate())
        self.bench_state_publisher_msg.data = geojson_benchstate
        self.bench_state_publisher.publish(self.bench_state_publisher_msg)
    
if __name__ == '__main__':
    hn = HeatNode()





