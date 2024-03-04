#!/usr/bin/env python

import rospy
import tf
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped
import traceback

def subscriber_callback_map_wrapper(publisher):
    def subscriber_callback(data):
        try:
            data.header.frame_id = 'local_map_lidar'
            publisher.publish(data)
        except Exception:
            traceback.print_exc()
    return subscriber_callback

def subscriber_callback_goal_wrapper(publisher):
    def subscriber_callback(data):
        try:
            trans = [15.0, 2.5, 0.0] # have to be fixed (values from frame_creator)
            data.pose.position.x -= trans[0]
            data.pose.position.y -= trans[1]
            data.pose.position.z -= trans[2]
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
