#! /usr/bin/env python

import rospy
import actionlib
from track_pkg.msg import TrackCoordinateAction, TrackCoordinateActionGoal, TrackCoordinateActionResult, TrackCoordinateActionFeedback
from track_pkg.msg import TrackTrajectoryAction, TrackTrajectoryActionGoal, TrackTrajectoryActionResult, TrackTrajectoryActionFeedback

TRACK_PIPELINE = 'TRAJECTORY' # can be switched on COORDINATE

def track_client_coordinates():
    client = actionlib.SimpleActionClient('move_to_location', TrackCoordinateAction)
    client.wait_for_server()
    goal = TrackCoordinateActionGoal()
    client.send_goal(goal.goal)
    client.wait_for_result()
    return client.get_result()

def track_client_trajectory():
    client = actionlib.SimpleActionClient('move_to_location', TrackTrajectoryAction)
    client.wait_for_server()
    goal = TrackTrajectoryActionGoal()
    client.send_goal(goal.goal)
    client.wait_for_result()
    return client.get_result()


if __name__ == '__main__':
    try:
        rospy.init_node('track_client')
        if TRACK_PIPELINE == 'COORDINATE':
            result = track_client_coordinates()
            print(result.x)
            print(result.y)
        elif TRACK_PIPELINE == 'TRAJECTORY':
            result = track_client_trajectory()
            print(result.result.points)
    except rospy.ROSInterruptException:
        print("Error")