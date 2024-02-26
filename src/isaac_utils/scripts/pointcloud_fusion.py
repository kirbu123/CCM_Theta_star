#!/usr/bin/env python3
import rospy
import tf2_ros

from sensor_msgs.msg import PointCloud2 
import numpy as np
import ros_numpy 
class PointcloudIsaacNode:

    def __init__(self) -> None:
        rospy.init_node('pcd_node')

        self.sub_pcd = rospy.Subscriber(
            '/velodyne_points',
            PointCloud2, 
            self.on_pcds
        )
        self.pcd_pub = rospy.Publisher(
            '/points_fusion',
            PointCloud2,
             queue_size=10
        )
        self.timer = rospy.Timer(rospy.Duration(2.0), self.on_publish)
        self.frame = "rtx_lidar"
        self.pcd2 = np.array([(0., 0., 0.)], dtype=[
        ('x', np.float32),
        ('y', np.float32),
        ('z', np.float32)
        ])
        self.time =rospy.Time()

    def on_pcds(self, msg):
        self.pcd2 = np.concatenate((self.pcd2, ros_numpy.numpify(msg)))
        self.frame = msg.header.frame_id
        self.time = msg.header.stamp
        #self.on_publish()

    def on_publish(self, event=None):
        msg_pub = ros_numpy.msgify(PointCloud2, self.pcd2)
        msg_pub.header.frame_id = self.frame
        msg_pub.header.stamp = self.time
        self.pcd_pub.publish(msg_pub)
        
        self.pcd2 = np.array([(0., 0., 0.)], dtype=[
        ('x', np.float32),
        ('y', np.float32),
        ('z', np.float32)
        ])

def main(args=None):
    node = PointcloudIsaacNode()
    rospy.spin()


if __name__ == '__main__':
    main()