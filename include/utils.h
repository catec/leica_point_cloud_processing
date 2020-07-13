// utils.h
#pragma once
#ifndef _UTILS_H
#define _UTILS_H

#include "ros/ros.h"
#include "ros/package.h"
#include "pcl_conversions/pcl_conversions.h"
#include <pcl_ros/point_cloud.h> 
#include <pcl/features/normal_3d.h>
#include <pcl/filters/extract_indices.h>

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
        static bool isValidCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
        static bool isValidCloud(PointCloudRGB::Ptr cloud);
        static bool isValidCloudMsg(sensor_msgs::PointCloud2 cloud_msg);
        static void colorizeCloud(PointCloudRGB::Ptr cloud_rgb,
                                  int R, int G, int B);
        static void cloudToXYZRGB(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                                  PointCloudRGB::Ptr cloud_rgb,
                                  int R, int G, int B);
        static void cloudToROSMsg(PointCloudRGB::Ptr cloud,
                                  sensor_msgs::PointCloud2 &cloud_msg);
        static double computeCloudResolution(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
        static double computeCloudResolution(PointCloudRGB::Ptr cloud);
        static void printTransform(Eigen::Matrix4f transform);
        static void filterNanValues(PointCloudRGB::Ptr &cloud);
        static void filterNanValues(PointCloudRGB::Ptr &cloud, pcl::PointCloud<pcl::Normal>::Ptr &normals);
        static void indicesFilter(PointCloudRGB::Ptr cloud_in,
                                  PointCloudRGB::Ptr cloud_out, 
                                  boost::shared_ptr<std::vector<int> > indices);
};