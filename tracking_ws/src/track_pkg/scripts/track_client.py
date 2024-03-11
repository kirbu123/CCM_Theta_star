#! /usr/bin/env python

import rospy
import actionlib
from track_pkg.msg import TrackCoordinateAction, TrackCoordinateActionGoal, TrackCoordinateActionResult, TrackCoordinateActionFeedback


def track_client():
    client = actionlib.SimpleActionClient('move_to_location', TrackCoordinateAction)
    client.wait_for_server()
    goal = TrackCoordinateActionGoal()
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