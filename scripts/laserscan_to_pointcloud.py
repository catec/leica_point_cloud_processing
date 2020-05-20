#! /usr/bin/env python

import rospy
import math
import laser_geometry.laser_geometry as lg
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2, LaserScan
import pcl_helper


pc_pub = rospy.Publisher("/point_cloud", PointCloud2, queue_size=1)

def scan_cb(msg):
    lp = lg.LaserProjection()

    # convert the message of type LaserScan to a PointCloud2
    pc2_msg = lp.projectLaser(msg)
    
    # publish 
    pc_pub.publish(pc2_msg)

if __name__ == '__main__':
    rospy.init_node("laserscan_to_pointcloud")


    rospy.Subscriber("/laser/scan", LaserScan, scan_cb, queue_size=1)
    rospy.spin()