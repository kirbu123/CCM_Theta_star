#!/usr/bin/env python
# license removed for brevity
import rospy
import tf
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from geometry_msgs.msg import Vector3, Quaternion
# from slam_communication_msgs.msg import OptimizationResult
import traceback

def talker():
    global_odometry_topic = rospy.Publisher('cartographer/tracked_global_odometry', Odometry, queue_size=10)
    local_odometry_topic = rospy.Publisher('cartographer/tracked_local_odometry', Odometry, queue_size=10)
    rospy.init_node('rtabmap_fixer', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    listener = tf.TransformListener()

    while not rospy.is_shutdown():
        try:
            (trans, rot) = listener.lookupTransform('/world', '/base_link', rospy.Time(0))
            odometry_msg = Odometry()

            # Create object Vector3 for position
            position = Vector3()
            position.x = trans[0]
            position.y = trans[1]
            position.z = trans[2]
            odometry_msg.pose.pose.position = position

            # Create object Quaternion for orientation
            orientation = Quaternion()
            orientation.x = rot[0]
            orientation.y = rot[1]
            orientation.z = rot[2]
            orientation.w = rot[3]
            odometry_msg.pose.pose.orientation = orientation

            odometry_msg.header.stamp = rospy.Time.now()
            odometry_msg.header.frame_id = 'world'
            odometry_msg.child_frame_id= 'base_link'

            global_odometry_topic.publish(odometry_msg)
            local_odometry_topic.publish(odometry_msg)
            rate.sleep()

            rospy.loginfo('Translation: %s, Rotation: %s' % (trans, rot))

        except Exception:
            traceback.print_exc()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        traceback.print_exc()