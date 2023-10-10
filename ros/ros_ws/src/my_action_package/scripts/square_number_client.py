#!/usr/bin/env python3

import time

import actionlib
import rospy
from my_action_package.msg import SquareNumberAction, SquareNumberGoal


def feedback_callback(feedback, server_id):
    rospy.loginfo(f"[{server_id}] Received feedback: Current Step %d", feedback.current_step)


def send_goal(server_id: int):
    time.sleep(2 * server_id)
    client = actionlib.SimpleActionClient(f"square_number_{server_id}", SquareNumberAction)
    client.wait_for_server()

    goal = SquareNumberGoal()
    goal.number_to_square = 4 * server_id
    client.send_goal(goal, feedback_cb=lambda feedback: feedback_callback(feedback, server_id))

    client.wait_for_result()
    return client.get_result()


if __name__ == "__main__":
    rospy.init_node("square_number_client")
    server_id = int(rospy.get_param("/square_number_client/id", "0"))
    result = send_goal(server_id)
    if result:
        rospy.loginfo(f"[{server_id}] The result is %d", result.squared_number)
    else:
        rospy.loginfo(f"[{server_id}] Received no result from the server")
