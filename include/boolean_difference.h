// BooleanDifference.h
#pragma once
#ifndef _BooleanDifference_H
#define _BooleanDifference_H

#include "ros/ros.h"
#include "ros/package.h"
#include "pcl_conversions/pcl_conversions.h"
#include <pcl_ros/point_cloud.h> 
#include <pcl/octree/octree_pointcloud_changedetector.h>
#include <pcl/filters/extract_indices.h>

#include <utils.h>

#endif 

typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudRGB;
typedef std::vector<int> IndicesVector;
typedef boost::shared_ptr<IndicesVector> IndicesVectorPtr;

class BooleanDifference {
    public:
        BooleanDifference(PointCloudRGB::Ptr cloud);
        ~BooleanDifference() {};

        bool substract_error;

        void substract(PointCloudRGB::Ptr cloud_to_substract);
        void getResultCloud(PointCloudRGB::Ptr result_cloud);
        double getVoxelResolution();

    private:
        PointCloudRGB::Ptr _cloud, _result_cloud;
        double _voxel_resolution;
        IndicesVectorPtr _diff_indices;

        void computeResolution(PointCloudRGB::Ptr cloud);
        void setOctreeAndGetIndices(PointCloudRGB::Ptr cloud_to_substract);
        int computeResultCloud();

};