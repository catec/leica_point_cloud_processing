// initial_alignment.h
#pragma once
#ifndef _INITIAL_ALIGNMENT_H
#define _INITIAL_ALIGNMENT_H

#include <Utils.h>
#include <Viewer.h>

#include <pcl/features/fpfh.h>
#include <pcl/features/multiscale_feature_persistence.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/registration/correspondence_estimation.h>
#include <pcl/registration/correspondence_rejection_sample_consensus.h>
#include <pcl/registration/transformation_estimation_svd.h>

#endif


/**
 * @brief Perform a rigid alignment from source to target cloud.
 * 
 * POINTCLOUDS:
 * - \b target pointcloud: should be obtained from converting a part's CAD. Use CADToPointCloud.
 * - \b source pointcloud: result from scanning the same CAD part with Leica scanstation C5 (either in reality or in gazebo simulator).
 * 
 * WORKFLOW:
 * 1. Before using this algorithm prepare clouds: Filter pointclouds to enhance results.
 * 2. Extract features and keypoints from pointcloud and it's normals (FPFH descriptor and pcl::MultiscaleFeaturePersistence).
 * 3. Use features and keypoints to determine correspondences between clouds (pcl::CorrespondenceEstimation).
 * 4. Estimate rigid transformation as initial alignment (pcl::TransformationEstimationSVD).
 * 5. Then, apply GICP to refine transformation (pcl::GeneralizedIterativeClosestPoint)
 *  
 */
class InitialAlignment 
{

typedef pcl::PointCloud<pcl::PointXYZ> PointCloudXYZ;
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudRGB;


public:

    /**
     * @brief Construct a new Initial Alignment object.
     * 
     * @param[in] target_cloud 
     * @param[in] source_cloud 
     */
    InitialAlignment(PointCloudRGB::Ptr target_cloud, 
                     PointCloudRGB::Ptr source_cloud);
    
    /**
     * @brief Destroy the Initial Alignment object.
     * 
     */
    ~InitialAlignment() {};

    /** @brief If true, rigid transformation is complete. */
    bool transform_exists;

    /**
     * @brief Perform initial alignment.
     * 
     */
    void run();

    /**
     * @brief Get the rigid transform object
     * 
     * @return Eigen::Matrix4f 
     */
    Eigen::Matrix4f getRigidTransform();

    /**
     * @brief Get the aligned cloud object
     * 
     * @param[out] aligned_cloud 
     */
    void getAlignedCloud(PointCloudRGB::Ptr aligned_cloud);

    /**
     * @brief Get the aligned cloud msg in PointCloud2 format
     * 
     * @param[out] aligned_cloud_msg 
     */
    void getAlignedCloudROSMsg(sensor_msgs::PointCloud2 &aligned_cloud_msg);


private:

    /** @brief Transformation matrix as result of initial alignment. */
    Eigen::Matrix4f _rigid_tf;

    /** @brief Radius to compute normals. */
    double _normal_radius;
    
    /** @brief Radius to compute feature. */
    double _feature_radius;
    
    /** @brief Threshold to compute correspondences. */
    double _inlier_threshold;
    
    /** @brief Target pointcloud. */
    PointCloudRGB::Ptr _target_cloud;

    /** @brief Source pointcloud. */
    PointCloudRGB::Ptr _source_cloud;
    
    /** @brief Source pointcloud aligned with target cloud. */
    PointCloudRGB::Ptr _aligned_cloud;


    /**
     * @brief Will set parameters to compute initial alignment based on clouds data.
     * 
     */
    void configParameters();

    /**
     * @brief Get the scale values for given cloud to obtain features.
     * 
     * @param[in] cloud 
     * @return std::vector<float> 
     */
    std::vector<float> getScaleValues(PointCloudRGB::Ptr cloud);

    /**
     * @brief Print on console the scale values in vector format.
     * 
     * @param scale_values 
     */
    void printScaleValues(const std::vector<float> &scale_values);

    /**
     * @brief Compute the cloud normals with given radius. Return false if error
     * 
     * @param[in] cloud 
     * @param[in] normal_radius 
     * @param[out] normals 
     * @return true 
     * @return false 
     */
    bool getNormals(PointCloudRGB::Ptr &cloud,
                    double normal_radius,
                    pcl::PointCloud<pcl::Normal>::Ptr &normals);

    /**
     * @brief Get keypoints and features for cloud.
     * 
     * @param[in] cloud 
     * @param[in] normals 
     * @param[out] keypoints_cloud 
     * @param[out] features 
     */
    void getKeypointsAndFeatures(PointCloudRGB::Ptr cloud, 
                                 pcl::PointCloud<pcl::Normal>::Ptr normals,
                                 PointCloudRGB::Ptr keypoints_cloud,
                                 pcl::PointCloud<pcl::FPFHSignature33>::Ptr features);

    /**
     * @brief Estimate correspondences to perform initial alignment. 
     * 
     * @param[in] source_features 
     * @param[in] target_features 
     * @param[in] source_keypoints 
     * @param[in] target_keypoints 
     * @param[out] correspondences 
     */
    void performInitialAlingment(pcl::PointCloud<pcl::FPFHSignature33>::Ptr source_features,
                                 pcl::PointCloud<pcl::FPFHSignature33>::Ptr target_features,
                                 PointCloudRGB::Ptr source_keypoints,
                                 PointCloudRGB::Ptr target_keypoints,
                                 pcl::CorrespondencesPtr correspondences);

    /**
     * @brief Once initial alignment is finished, apply rigid transformation to source cloud.
     * 
     */
    void applyTFtoCloud();
};