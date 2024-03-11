#! /usr/bin/env python3
import rospy
import tf
import math
import time
import os
import actionlib
import numpy as np
from geometry_msgs.msg import PoseStamped
from track_pkg.msg import trackAction, trackActionGoal, trackActionResult, trackActionFeedback

class MoveRobot:
    def __init__(self):
        # Setup feedback and result
        rospy.init_node('track_action_server')
        self.rate = rospy.Rate(2)
        self.server = actionlib.SimpleActionServer('move_to_location', trackAction, self.server_callback, False)
        self.server.start()
        self._feedback = trackActionFeedback()
        self._result = trackActionResult()
        self._goal = trackActionGoal()
        
        rospy.Subscriber("/move_base_simple/goal", PoseStamped, self.subscriber_callback)

        rospy.loginfo('Server started')

    def server_callback(self, msg):
        r = rospy.Rate(2)
        print(self._result)
        self._result.result.x = self._goal.goal.x
        self._result.result.y = self._goal.goal.y
        print('OK')
        self.server.set_succeeded(self._result.result)

    def subscriber_callback(self, data):
        self._goal.goal.x = data.pose.position.x
        self._goal.goal.y = data.pose.position.y
        print(data)
    

if __name__ == '__main__':
    server = MoveRobot()
    rospy.spin()