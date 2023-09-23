#!/usr/bin/env python

import rospy
import actionlib
from my_action_package.msg import SquareNumberAction, SquareNumberGoal

def feedback_callback(feedback):
    rospy.loginfo('Received feedback: Current Step %d', feedback.current_step)

def send_goal():
    client = actionlib.SimpleActionClient('square_number', SquareNumberAction)
    client.wait_for_server()

    goal = SquareNumberGoal()
    goal.number_to_square = 4
    client.send_goal(goal, feedback_cb=feedback_callback)

    client.wait_for_result()
    return client.get_result()

if __name__ == '__main__':
    rospy.init_node('square_number_client')
    result = send_goal()
    if result:
        rospy.loginfo('The result is %d', result.squared_number)
    else:
        rospy.loginfo('Received no result from the server')