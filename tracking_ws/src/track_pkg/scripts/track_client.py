#! /usr/bin/env python

import rospy
import actionlib
from track_pkg.msg import trackAction, trackActionGoal, trackActionResult, trackActionFeedback


def track_client():
    client = actionlib.SimpleActionClient('move_to_location', trackAction)
    client.wait_for_server()
    goal = trackActionGoal()
    client.send_goal(goal.goal)
    client.wait_for_result()
    return client.get_result()  # A FibonacciResult

if __name__ == '__main__':
    try:
        rospy.init_node('track_client')
        result = track_client()
        print(result.x)
        print(result.y)
    except rospy.ROSInterruptException:
        print("Error")