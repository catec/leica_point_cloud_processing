// utils.h
#pragma once
#ifndef _UTILS_H
#define _UTILS_H

#include "ros/ros.h"
#include "ros/package.h"
#include "pcl_conversions/pcl_conversions.h"
#include <pcl_ros/point_cloud.h> 
#include <pcl/features/normal_3d.h>

#endif 

typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudRGB;


class Utils {
    public:
        Utils();
        ~Utils() {};

        std::string _pc_path;
        
        static std::string _frame_id;

        std::string getPCpath();
        static bool getNormals(PointCloudRGB::Ptr &cloud,
                               double normal_radius,
                               pcl::PointCloud<pcl::Normal>::Ptr &normals);
        static void cloudToXYZRGB(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                                  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_rgb,
                                  int R, int G, int B);
        static void cloudToROSMsg(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
                                  sensor_msgs::PointCloud2 &cloud_msg);
        static double computeCloudResolution(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
        static double computeCloudResolution(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);
        static void printTransform(Eigen::Matrix4f transform);
        static void filterNanValues(PointCloudRGB::Ptr &cloud);
        static void filterNanValues(PointCloudRGB::Ptr &cloud, pcl::PointCloud<pcl::Normal>::Ptr &normals);
};