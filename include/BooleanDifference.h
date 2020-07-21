// boolean_difference.h
#pragma once
#ifndef _BOOLEAN_DIFFERENCE_H
#define _BOOLEAN_DIFFERENCE_H

#include <utils.h>

#include <pcl/octree/octree_pointcloud_changedetector.h>

#endif 


class BooleanDifference {

typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudRGB;
typedef std::vector<int> IndicesVector;
typedef boost::shared_ptr<IndicesVector> IndicesVectorPtr;

public:

    /**
     * @brief Construct a new Boolean Difference object
     * 
     * @param[in] cloud 
     */
    BooleanDifference(PointCloudRGB::Ptr cloud);
    
    /**
     * @brief Destroy the Boolean Difference object
     * 
     */
    ~BooleanDifference() {};


    /** @brief If true, substracting process failed. */
    bool substract_error;

    /**
     * @brief Implement substraction between cloud and cloud_to_substract. Store result cloud.
     * 
     * @param[in] cloud_to_substract 
     */
    void substract(PointCloudRGB::Ptr cloud_to_substract);

    /**
     * @brief Get the Result Cloud object obtained from substraction.
     * 
     * @param[out] result_cloud 
     */
    void getResultCloud(PointCloudRGB::Ptr result_cloud);

    /**
     * @brief Get the Voxel Resolution object.
     * 
     * @return double 
     */
    double getVoxelResolution();


private:

    /** @brief Base cloud */
    PointCloudRGB::Ptr _cloud;

    /** @brief Result cloud from differenciating _cloud and other */
    PointCloudRGB::Ptr _result_cloud;

    /** @brief The Voxel Resolution to set OcTree */
    double _voxel_resolution;
    
    /** @brief Indices of differences found */
    IndicesVectorPtr _diff_indices;

    /**
     * @brief Compute cloud resolution and set voxel resolution.
     * 
     * @param[in] cloud 
     */
    void computeResolution(PointCloudRGB::Ptr cloud);

    /**
     * @brief Apply boolean difference from Octrees and store indices of differences found.
     * 
     * @param cloud_to_substract 
     */
    void setOctreeAndGetIndices(PointCloudRGB::Ptr cloud_to_substract);

    /**
     * @brief Filter cloud with indices obtained. Store result cloud. Returns -1 if error.
     * 
     * @return int 
     */
    int computeResultCloud();

};