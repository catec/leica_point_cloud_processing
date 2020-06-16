#include "ros/ros.h"
#include "ros/package.h"
#include "pcl_conversions/pcl_conversions.h"
#include <pcl_ros/point_cloud.h> 
#include <pcl/features/fpfh.h>
#include <pcl/features/multiscale_feature_persistence.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/registration/correspondence_estimation.h>
#include <pcl/registration/correspondence_rejection_sample_consensus.h>
#include <pcl/registration/transformation_estimation_svd.h>
#include <utils.h>

/**
 * POINTCLOUDS:
    - target pointcloud: directly obtain from downsampling a part's cad
    - source pointcloud: result from scanning the same cad part in Gazebo with leica c5 simulator
 * WORKFLOW:
    1. Filter pointclouds to reduce number of points (VoxelGrid)
    2. Get normal for every point in both clouds (pcl::NormalEstimation)
    3. Extract features and keypoints from pointcloud and it's normals (FPFH descriptor and pcl::MultiscaleFeaturePersistence)
    4. Perform initial aligment as rigid transformation (pcl::CorrespondenceEstimation)
    5. Applied GICP to refine transformation (pcl::GeneralizedIterativeClosestPoint)
**/

typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudRGB;

class InitialAlignment 
{
    public:
        InitialAlignment(PointCloudRGB::Ptr target_cloud, PointCloudRGB::Ptr source_cloud);
        ~InitialAlignment() {};

        bool transform_exists;
        void run();
        Eigen::Matrix4f getRigidTransform();

    private:
        Eigen::Matrix4f _rigid_tf;
        double _normal_radius, _feature_radius, _inlier_threshold;
        PointCloudRGB::Ptr _target_cloud, _source_cloud;

        void configParameters();
        std::vector<float> getScaleValues(PointCloudRGB::Ptr cloud);
        void printScaleValues(std::vector<float> scale_values);
        bool getNormals(PointCloudRGB::Ptr &cloud,
                        double normal_radius,
                        pcl::PointCloud<pcl::Normal>::Ptr &normals);
        void getKeypointsAndFeatures(PointCloudRGB::Ptr cloud, 
                                     pcl::PointCloud<pcl::Normal>::Ptr normals,
                                     PointCloudRGB::Ptr keypoints_cloud,
                                     pcl::PointCloud<pcl::FPFHSignature33>::Ptr features);
        void initialAlingment(pcl::PointCloud<pcl::FPFHSignature33>::Ptr source_features,
                              pcl::PointCloud<pcl::FPFHSignature33>::Ptr target_features,
                              PointCloudRGB::Ptr source_keypoints,
                              PointCloudRGB::Ptr target_keypoints,
                              pcl::CorrespondencesPtr correspondences);

};