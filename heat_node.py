#!/usr/bin/env python
import rospy
from parked_custom_msgs.msg import Point
from std_msgs.msg import String, Bool
from heat_map import HeatMapList # todo we need to change message type to handle bench_id
import datetime

class HeatNode:
    def __init__(self):
        self.heat_map = HeatMapList(3)
        rospy.init_node('heat_node', anonymous=True)

        self.bench1_pressure_data_pub = rospy.Publisher('/bench1_pressure_data', Bool)
        self.bench1_pressure_data = Bool()

        # subscribes to robot's current position. The payload of the message will be a parked_custom_msgs/Point object
        self.position_subscriber = rospy.Subscriber('/robot_position_longlat', Point, self.update_current_position, callback_args=1)
        self.position_subscriber = rospy.Subscriber('/bench2_position_longlat', Point, self.update_current_position, callback_args=2)
        self.position_subscriber = rospy.Subscriber('/bench3_position_longlat', Point, self.update_current_position, callback_args=3)
        # self.position_subscriber = rospy.Subscriber('/robot_positon_4', Point, self.update_current_position, callback_args=4)
        self.bench1_current_position = Point(0.6,0.6,-999)
        self.bench2_current_position = Point(1.6,1.7,-999)
        self.bench3_current_position = Point(-0.3,-0.2,-999)

        self.bench1_occupied = False
        self.bench2_occupied = False
        self.bench3_occupied = False

        # publishes string notation of geojson
        self.heat_map_publisher = rospy.Publisher('/heat_map_data', String, queue_size=5)
        self.publisher_msg = String()

        # Publish location data
        self.bench_state_publisher = rospy.Publisher('/bench_state_data', String, queue_size=1)
        self.bench_state_publisher_msg = String()
        rospy.spin()

    # Updates the 'seated' status of benches when sat on or un-sat on
    def update_heatmap(self, pressure_data, bench_id):
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
        sit_down = pressure_data
        time = datetime.datetime.now()
        long_lat = (longitude, latitude)

        # todo we need to use this bench_id in the calling of these methods
        self.heat_map.change_state(bench_id, long_lat, time, sit_down)
        if not sit_down:    #TODO: does this only publish when people stand up? If so, the bench state will never be 'in use'
            geojson_heatmap = str(self.heat_map.to_geojson_heatmap())
            self.publisher_msg.data = geojson_heatmap
            self.heat_map_publisher.publish(self.publisher_msg)
            
    
    # Updates stored bench positions when they change and publishes new feature collection of benches to frontend
    # TODO: ensure this isn't publishing too often to be feasible
    def update_current_position(self, data, bench_id):
        # print("updating cp: ", bench_id, data)
        
        position = data
        if bench_id == 1:
            if position.long != self.bench1_current_position.long or position.lat != self.bench1_current_position.lat:
                self.bench1_current_position = position
                self.heat_map.change_loc(1, position)
            if position.angle == -999 and not self.bench1_occupied:
                self.bench1_occupied = True
                self.update_heatmap(self.bench1_occupied, 1)
                self.bench1_pressure_data.data = self.bench1_occupied
                self.bench1_pressure_data_pub.publish(self.bench1_pressure_data)
            elif position.angle != -999 and self.bench1_occupied:
                self.bench1_occupied = False
                self.update_heatmap(self.bench1_occupied, 1)
        elif bench_id == 2:
            if position.long == 0 and position.lat == 0 and not self.bench2_occupied:
                self.bench2_occupied = True
                self.update_heatmap(self.bench2_occupied, 2)
            elif (position.long != 0 or position.lat != 0) and self.bench2_occupied:
                self.bench2_occupied = False
                self.update_heatmap(self.bench2_occupied, 2)
            elif (position.long != 0 or position.lat != 0) and (position.long != self.bench2_current_position.long or position.lat != self.bench2_current_position.lat):
                print("updating pos")
                self.bench2_current_position = position
                self.heat_map.change_loc(2, position)
        elif bench_id == 3:
            if position.long == 0 and position.lat == 0 and not self.bench3_occupied:
                self.bench3_occupied = True
                self.update_heatmap(self.bench3_occupied, 3)
            elif (position.long != 0 or position.lat != 0) and self.bench3_occupied:
                self.bench3_occupied = False
                self.update_heatmap(self.bench3_occupied, 3)
            elif (position.long != 0 or position.lat != 0) and (position.long != self.bench3_current_position.long or position.lat != self.bench3_current_position.lat):
                print("updating pos")
                self.bench3_current_position = position
                self.heat_map.change_loc(3, position)
            
        geojson_benchstate = str(self.heat_map.to_geojson_benchstate())
        self.bench_state_publisher_msg.data = geojson_benchstate
        self.bench_state_publisher.publish(self.bench_state_publisher_msg)
    
if __name__ == '__main__':
    hn = HeatNode()





