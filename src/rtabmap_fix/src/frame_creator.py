#!/usr/bin/env python

import rospy
import tf
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import Vector3, Quaternion
import traceback

rospy.init_node('create_new_frame')

tf_broadcaster = tf.TransformBroadcaster()

while not rospy.is_shutdown():
    try:
        # Create object Vector3 for position
        position = Vector3()
        position.x = 15.0
        position.y = 2.5
        position.z = 0.0

        # Create object Quaternion for orientation
        orientation = Quaternion()
        orientation.x = 0.0
        orientation.y = 0.0
        orientation.z = 0.0
        orientation.w = 1.0

        # Publish transform
        tf_broadcaster.sendTransform((position.x, position.y, position.z), (orientation.x, orientation.y, orientation.z, orientation.w), rospy.Time.now(), "map_frame", "world")

        rospy.sleep(1.0)  # Pause 1 second
    except Exception:
        traceback.print_exc()