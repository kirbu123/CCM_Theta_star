#! /usr/bin/env python3
import rospy
import tf
import math
import time
import os
import actionlib
import numpy as np
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker
from track_pkg.msg import TrackCoordinateAction, TrackCoordinateActionGoal, TrackCoordinateActionResult, TrackCoordinateActionFeedback
from track_pkg.msg import TrackTrajectoryAction, TrackTrajectoryActionGoal, TrackTrajectoryActionResult, TrackTrajectoryActionFeedback


class MoveRobot:
    def __init__(self):
        # Setup feedback and result

        self.ready_to_send = True
        rospy.init_node('track_action_server')
        self.rate = rospy.Rate(2)

        self.TRACK_PIPELINE = 'TRAJECTORY' # can be switched on COORDINATE
        if self.TRACK_PIPELINE == 'COORDINATE':
            self.server = actionlib.SimpleActionServer('move_to_location', TrackCoordinateAction, self.server_callback, False)
        elif self.TRACK_PIPELINE == 'TRAJECTORY':
            self.server = actionlib.SimpleActionServer('move_to_location', TrackTrajectoryAction, self.server_callback, False)
        self.server.start()

        self.goal_determined = False

        # coordinate msgs
        self._feedback_coordinate = TrackCoordinateActionFeedback()
        self._result_coordinate = TrackCoordinateActionResult()
        self._goal_coordinate = TrackCoordinateActionGoal()

        # trajectory msgs
        self._feedback_trajectory = TrackTrajectoryActionFeedback()
        self._result_trajectory = TrackTrajectoryActionResult()
        self._goal_trajectory = TrackTrajectoryActionGoal()
        
        rospy.Subscriber("/move_base_simple/goal", PoseStamped, self.subscriber_callback_cordinates)
        rospy.Subscriber("/path_marker", Marker, self.subscriber_callback_trajectory)

        rospy.loginfo('Server started')

    def server_callback(self, msg):
        r = rospy.Rate(2)
        if self.TRACK_PIPELINE == 'COORDINATE':
            if self.goal_determined:
                self._result_coordinate.result.x = self._goal_coordinate.goal.x
                self._result_coordinate.result.y = self._goal_coordinate.goal.y
                rospy.loginfo(f'Result determined by ROS: x: {self._result_coordinate.result.x}  y: {self._result_coordinate.result.y}')
            else:
                self._result_coordinate.result.x, self._result_coordinate.result.y = (msg.x, msg.y)
                rospy.loginfo(f'Result determined by MESSAGE: x: {self._result_coordinate.result.x}  y: {self._result_coordinate.result.y}')
            self.server.set_succeeded(self._result_coordinate.result)
        elif self.TRACK_PIPELINE == 'TRAJECTORY':
            if self.goal_determined:
                rospy.loginfo(f'Result determined by ROS: {self._result_trajectory.result.result.points}')
            else:
                rospy.loginfo(f'No trajectory recieved')
            if self.ready_to_send:
                self.server.set_succeeded(self._result_trajectory.result)
                self.ready_to_send = False

    


    def subscriber_callback_cordinates(self, data):
        self.goal_determined = True
        self._goal_coordinate.goal.x = data.pose.position.x
        self._goal_coordinate.goal.y = data.pose.position.y
        self.ready_to_send = True
        rospy.loginfo(f'Data recieved: {data}')

    def subscriber_callback_trajectory(self, data):
        self.goal_determined = True
        self._result_trajectory.result.result = data
        rospy.loginfo(f'Data recieved: {data.points}')


if __name__ == '__main__':
    server = MoveRobot()
    rospy.spin()