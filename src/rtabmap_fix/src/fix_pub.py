#!/usr/bin/env python

import rospy
import tf
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped
import traceback
import tf.transformations as tft

def subscriber_callback_map_wrapper(publisher):
    def subscriber_callback(data):
        try:
            trans = [33.6, -10.2, 0]

            data.info.origin.position.x = trans[0]
            data.info.origin.position.y = trans[1]
            data.info.origin.position.z = trans[2]

            roll = 0.0
            pitch = 0.0
            yaw = -2.0

            quaternion = tft.quaternion_from_euler(-roll, -pitch, -yaw)

            data.info.origin.orientation.x = quaternion[0]
            data.info.origin.orientation.y = quaternion[1]
            data.info.origin.orientation.z = quaternion[2]
            data.info.origin.orientation.w = quaternion[3]

            data.header.frame_id = 'local_map_lidar'
            publisher.publish(data)
        except Exception:
            traceback.print_exc()
    return subscriber_callback

def subscriber_callback_goal_wrapper(publisher):
    def subscriber_callback(data):
        try:
            data.header.frame_id = 'local_map_lidar'
            publisher.publish(data)
        except Exception:
            traceback.print_exc()
    return subscriber_callback

def occupancy_grid_subscriber(publisher_map, publisher_goal):
    rospy.init_node('occupancy_grid_pub_sub_node', anonymous=True)
    callback_map = subscriber_callback_map_wrapper(publisher_map)
    callback_goal = subscriber_callback_goal_wrapper(publisher_goal)

    rospy.Subscriber("/occupancy_grid_map/grid_map", OccupancyGrid, callback_map)
    rospy.Subscriber("/move_base_simple/goal", PoseStamped, callback_goal)
    rospy.spin()

def occupancy_grid_publisher():
    pub = rospy.Publisher('/occupancy_grid_map/grid_map_helper', OccupancyGrid, queue_size=10)
    pub.frame_id = "local_map_lidar"

    return pub

def goal_publisher():
    pub = rospy.Publisher('/move_base_simple/goal_helper', PoseStamped, queue_size=10)
    pub.frame_id = "local_map_lidar"

    return pub

if __name__ == '__main__':
    try:
        publisher_map = occupancy_grid_publisher()
        publisher_goal = goal_publisher()
        occupancy_grid_subscriber(publisher_map, publisher_goal)
    except rospy.ROSInterruptException:
        traceback.print_exc()
