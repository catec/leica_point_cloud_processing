#!/usr/bin/env python2

import rospy
import rospkg
import os
import pcl
from pcl import pcl_visualization
from pcl_helper import *

if __name__ == '__main__':
    rospy.init_node('load_pointcloud')

    rospack = rospkg.RosPack()
    pc_path = rospack.get_path("leica_scanstation") + "/pointclouds/"
    print(pc_path)

    pc_sub = rospy.Publisher("/point_cloud", PointCloud2, queue_size=1)

    # Load point cloud
    pc = pcl.load_XYZRGB(pc_path + "linux0.ptx.pcd")

    # print "viewer"
    os.system("pcl_viewer " + pc_path + "linux0.ptx.pcd")

    # Convert to ros
    ros_pc = pcl_to_ros(pc)
    
    while not rospy.is_shutdown():
        pc_sub.publish(ros_pc)