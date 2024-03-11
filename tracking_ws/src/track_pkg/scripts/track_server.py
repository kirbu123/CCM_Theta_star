#! /usr/bin/env python3
import rospy
import tf
import math
import time
import os
import actionlib
import numpy as np
from geometry_msgs.msg import PoseStamped
from track_pkg.msg import TrackCoordinateAction, TrackCoordinateActionGoal, TrackCoordinateActionResult, TrackCoordinateActionFeedback


class MoveRobot:
    def __init__(self):
        # Setup feedback and result
        rospy.init_node('track_action_server')
        self.rate = rospy.Rate(2)
        self.server = actionlib.SimpleActionServer('move_to_location', TrackCoordinateAction, self.server_callback, False)
        self.server.start()
        self._feedback = TrackCoordinateActionFeedback()
        self._result = TrackCoordinateActionResult()
        self._goal = TrackCoordinateActionGoal()
        self.goal_determined = False
        
        rospy.Subscriber("/move_base_simple/goal", PoseStamped, self.subscriber_callback)

        rospy.loginfo('Server started')

    def server_callback(self, msg):
        r = rospy.Rate(2)
        if self.goal_determined:
            self._result.result.x = self._goal.goal.x
            self._result.result.y = self._goal.goal.y
            rospy.loginfo(f'Result determined by ROS: x: {self._result.result.x}  y: {self._result.result.y}')
        else:
            self._result.result.x, self._result.result.y = (msg.x, msg.y)
            rospy.loginfo(f'Result determined by MESSAGE: x: {self._result.result.x}  y: {self._result.result.y}')
        self.server.set_succeeded(self._result.result)

    def subscriber_callback(self, data):
        self.goal_determined = True
        self._goal.goal.x = data.pose.position.x
        self._goal.goal.y = data.pose.position.y
        rospy.loginfo(f'Data recieved: {data}')
    

if __name__ == '__main__':
    server = MoveRobot()
    rospy.spin()