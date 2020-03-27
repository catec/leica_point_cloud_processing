#! /usr/bin/env python

import sensor_msgs.point_cloud2 as pc2
import rospy
from sensor_msgs.msg import PointCloud2, LaserScan
import laser_geometry.laser_geometry as lg
import math

pc_pub = rospy.Publisher("/point_cloud", PointCloud2, queue_size=1)

def scan_cb(msg):
    lp = lg.LaserProjection()

    # convert the message of type LaserScan to a PointCloud2
    pc2_msg = lp.projectLaser(msg)

    # publish it
    pc_pub.publish(pc2_msg)

if __name__ == '__main__':
    rospy.init_node("laserscan_to_pointcloud")


    rospy.Subscriber("/c5/laser/scan", LaserScan, scan_cb, queue_size=1)
    rospy.spin()


