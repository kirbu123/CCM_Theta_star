#!/usr/bin/env python

import actionlib
import rospy
from my_action_package.msg import (
    SquareNumberAction,
    SquareNumberFeedback,
    SquareNumberResult,
)


class SquareNumberServer:
    def __init__(self):
        self.server = actionlib.SimpleActionServer("square_number", SquareNumberAction, self.execute, False)
        self.server.start()

    def execute(self, goal):
        feedback = SquareNumberFeedback()
        result = SquareNumberResult()
        r = rospy.Rate(1)
        for i in range(goal.number_to_square + 1):
            if self.server.is_preempt_requested():
                rospy.loginfo("The goal has been preempted")
                self.server.set_preempted()
                return

            feedback.current_step = i
            self.server.publish_feedback(feedback)
            r.sleep()

        result.squared_number = goal.number_to_square * goal.number_to_square
        rospy.loginfo("Sending result %d", result.squared_number)
        self.server.set_succeeded(result)


if __name__ == "__main__":
    rospy.init_node("square_number_server")
    server = SquareNumberServer()
    rospy.spin()
