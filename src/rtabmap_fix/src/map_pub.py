#!/usr/bin/env python

import rospy
from nav_msgs.msg import OccupancyGrid
import traceback

def subscriber_callback_wrapper(publisher):
    def subscriber_callback(data):
        # rospy.loginfo("I heard: %s", data.data)
        try:
            data.header.frame_id = 'map_frame'
            publisher.publish(data)
        except Exception:
            traceback.print_exc()
    return subscriber_callback

def occupancy_grid_subscriber(publisher):
    rospy.init_node('occupancy_grid_pub_sub_node', anonymous=True)
    callback = subscriber_callback_wrapper(publisher)
    rospy.Subscriber("/occupancy_grid_map/grid_map", OccupancyGrid, callback)
    rospy.spin()

def occupancy_grid_publisher():    
    pub = rospy.Publisher('/occupancy_grid_map/grid_map2', OccupancyGrid, queue_size=10)
    pub.frame_id = "map_frame"

    return pub

if __name__ == '__main__':
    try:
        publisher = occupancy_grid_publisher()
        occupancy_grid_subscriber(publisher)
    except rospy.ROSInterruptException:
        traceback.print_exc()
