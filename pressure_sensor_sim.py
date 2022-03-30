#!/usr/bin/env python
import rospy
from std_msgs.msg import Bool

class Pressure_Sensor_Sim:

    def __init__(self):
        self.publisher = rospy.Publisher('/pressure_data', Bool, queue_size=2)
        rospy.init_node('pressure_sensor_sim')
        print('sim node has been started')
        data = Bool()
        self.rate = rospy.Rate(5)

        while (not rospy.is_shutdown()):
            x = input('press T to toggle')            
            data.data = not data.data
            self.publisher.publish(data)

if __name__ =='__main__':
    pss = Pressure_Sensor_Sim()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")


        
