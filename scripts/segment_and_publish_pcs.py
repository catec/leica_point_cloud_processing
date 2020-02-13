#!/usr/bin/env python2

import rospy
import rospkg
import os
import pcl
from pcl import pcl_visualization
from pcl_helper import *

stringers_publisher = rospy.Publisher("/pc_stringers", PointCloud2, queue_size=1)

def recolorize_pointcloud(point_cloud, is_rgb, color):
    if is_rgb:
        # Remove color
        point_cloud = XYZRGB_to_XYZ(point_cloud)
    # apply color
    colorized_point_cloud = XYZ_to_XYZRGB(point_cloud, color)
    return colorized_point_cloud

def pointcloudCb(msg):
    rospy.loginfo("cb")
    pc = ros_to_pcl(msg)

    # Use cylinder segmentation to separate skin from stringers
    segmenter = pc.make_segmenter_normals(ksearch=50)
    segmenter.set_optimize_coefficients(True)
    segmenter.set_model_type(pcl.SACMODEL_CYLINDER)
    # segmenter.set_normal_distance_weight(0.1)
    segmenter.set_method_type(pcl.SAC_RANSAC)
    segmenter.set_max_iterations(10000)
    segmenter.set_distance_threshold(0.05)
    segmenter.set_radius_limits(1.5, 4)

    # obtain inlier indices and model coefficients
    inlier_indices, coefficients = segmenter.segment()

    # extract separated objects from pointcloud
    skin_pc = pc.extract(inlier_indices, False)
    stringers_pc = pc.extract(inlier_indices, True)

    # Colorize stringers
    coloured_stringers_pc = recolorize_pointcloud(stringers_pc, True, [0, 0, 255])

    # publish point clouds
    # ros_skin_pc = pcl_to_ros(skin_pc)
    ros_stringer_pc = pcl_to_ros(coloured_stringers_pc)

    # skin_publisher.publish(ros_skin_pc)
    stringers_publisher.publish(ros_stringer_pc)


if __name__ == '__main__':
    rospy.init_node('publish_segmented_pointcloud')

    rospack = rospkg.RosPack()
    pc_path = rospack.get_path("leica_scanstation") + "/pointclouds/"
    print(pc_path)
    rospy.loginfo("main 2")

    pc_sub = rospy.Subscriber("/point_cloud",PointCloud2,pointcloudCb,queue_size = 1)

    rospy.spin()