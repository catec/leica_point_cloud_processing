// filter.h
#pragma once
#ifndef _FILTER_H
#define _FILTER_H

#include "ros/ros.h"
#include "ros/package.h"
#include <pcl_ros/point_cloud.h> 
#include "pcl_conversions/pcl_conversions.h"
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/crop_box.h>
#include "pcl/common/angles.h"
#include "pcl/segmentation/sac_segmentation.h"
#include <utils.h>

#endif

typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudRGB;

class Filter {
    public:
        Filter();
        Filter(double leaf_size);
        Filter(double leaf_size, double noise_threshold);
        Filter(double leaf_size, double noise_threshold, double floor_threshold);
        ~Filter() {};

        double _leaf_size, _noise_filter_threshold, _floor_filter_threshold;
        ros::Publisher pub;

        void downsampleCloud(PointCloudRGB::Ptr cloud, PointCloudRGB::Ptr cloud_downsampled);
        void filter_noise(double threshold, PointCloudRGB::Ptr cloud, PointCloudRGB::Ptr cloud_filtered);
        void filter_floor(double threshold, PointCloudRGB::Ptr cloud, PointCloudRGB::Ptr cloud_filtered);
        void setLeafSize(double new_leaf_size);
        void run(PointCloudRGB::Ptr cloud, PointCloudRGB::Ptr cloud_filtered);
};