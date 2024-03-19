#!/usr/bin/env python

import rospy
import tf
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import Vector3, Quaternion
import traceback
import tf.transformations as tft

rospy.init_node('create_new_frame')

tf_broadcaster = tf.TransformBroadcaster()

while not rospy.is_shutdown():
    try:
        # Create object Vector3 for position
        position = Vector3()
        position.x = 0.0
        position.y = 0.0
        position.z = 0.0

        # Create object Quaternion for orientation
        roll = 0.0
        pitch = 0.0
        yaw = 0.0

        quaternion = tft.quaternion_from_euler(roll, pitch, yaw)

        # Publish transform
        tf_broadcaster.sendTransform((position.x, position.y, position.z), quaternion, rospy.Time.now(), "local_map_lidar", "world")

        rospy.sleep(1.0)  # Pause 1 second
    except Exception:
        traceback.print_exc()