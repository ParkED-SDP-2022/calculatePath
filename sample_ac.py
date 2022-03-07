#! /usr/bin/env python

import rospy


# Brings in the SimpleActionClient
import actionlib
from parked_custom_msgs.msg import MoveToPointAction, MoveToPointGoal, Point

# Brings in the messages used by the fibonacci action, including the
# goal message and the result message.
import actionlib_tutorials.msg

def fibonacci_client():
    # Creates the SimpleActionClient, passing the type of the action
    # (FibonacciAction) to the constructor.
    client = actionlib.SimpleActionClient('move_to_point', MoveToPointAction)

    # Waits until the action server has started up and started
    # listening for goals.
    client.wait_for_server()
    print('server available')

    # Creates a goal to send to the action server.
    # goal = MoveToPointGoal(Point(-2.65869140625,
    #                 1.1864386394452024, -999), Point(-0.19775390625,
    #                 2.4272521703917294, -999), [])

    goal = MoveToPointGoal(Point(-0.19775390625,
                    2.4272521703917294, -999))

    # Sends the goal to the action server.
    client.send_goal(goal, feedback_cb=handle_feedback)
    print('goal sent')

    # Waits for the server to finish performing the action.
    client.wait_for_result()

    # Prints out the result of executing the action
    return client.get_result()  # A FibonacciResult

def handle_feedback(data):
    print(data)

if __name__ == '__main__':
    try:
        # Initializes a rospy node so that the SimpleActionClient can
        # publish and subscribe over ROS.
        rospy.init_node('fibonacci_client_py')
        result = fibonacci_client()
        print(result)
    except rospy.ROSInterruptException:
        print("program interrupted before completion", file=sys.stderr)
