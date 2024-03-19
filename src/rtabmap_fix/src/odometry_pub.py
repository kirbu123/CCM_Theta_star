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
            (trans_local, rot_local) = listener.lookupTransform('/world', '/base_link', rospy.Time(0))
            (trans_global, rot_global) = listener.lookupTransform('/world', '/local_map_lidar', rospy.Time(0))
            for i in range(len(trans_global)):
                trans_global[i] = trans_local[i] - trans_global[i]
            odometry_local_msg = Odometry()
            odometry_global_msg = Odometry()

            # Create object Vector3 for position
            position_local = Vector3()
            position_local.x = trans_local[0]
            position_local.y = trans_local[1]
            position_local.z = trans_local[2]
            odometry_local_msg.pose.pose.position = position_local

            position_global = Vector3()
            position_global.x = trans_global[0]
            position_global.y = trans_global[1]
            position_global.z = trans_global[2]
            odometry_global_msg.pose.pose.position = position_global

            # Create object Quaternion for orientation
            orientation_local = Quaternion()
            orientation_local.x = rot_local[0]
            orientation_local.y = rot_local[1]
            orientation_local.z = rot_local[2]
            orientation_local.w = rot_local[3]
            odometry_local_msg.pose.pose.orientation = orientation_local

            orientation_gobal = orientation_local
            odometry_global_msg.pose.pose.orientation = orientation_gobal

            odometry_local_msg.header.stamp = rospy.Time.now()
            odometry_local_msg.header.frame_id = 'world'
            odometry_local_msg.child_frame_id= 'base_link'

            odometry_global_msg.header.stamp = rospy.Time.now()
            odometry_global_msg.header.frame_id = 'local_map_lidar'
            odometry_global_msg.child_frame_id= 'base_link'

            global_odometry_topic.publish(odometry_global_msg)
            local_odometry_topic.publish(odometry_local_msg)
            rate.sleep()

            # rospy.loginfo('Translation: %s' % trans_global)
            rospy.loginfo("ROS TOPICS RUN")

        except Exception:
            traceback.print_exc()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        traceback.print_exc()