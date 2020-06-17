#include "ros/ros.h"
#include "ros/package.h"
#include "pcl_conversions/pcl_conversions.h"
#include <pcl_ros/point_cloud.h> 
// #include <pcl/features/fpfh.h>
// #include <pcl/features/multiscale_feature_persistence.h>
// #include <pcl/features/normal_3d.h>
#include <pcl/features/from_meshes.h>
#include <pcl/filters/extract_indices.h>
// #include <pcl/filters/voxel_grid.h>
// #include <pcl/registration/correspondence_estimation.h>
// #include <pcl/registration/correspondence_rejection_sample_consensus.h>
// #include <pcl/registration/transformation_estimation_svd.h>
#include <pcl/registration/gicp.h>
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
typedef std::vector<Eigen::Matrix3d, Eigen::aligned_allocator<Eigen::Matrix3d>> CovariancesVector;

class GICPAlignment 
{
    public:
        GICPAlignment(PointCloudRGB::Ptr target_cloud, PointCloudRGB::Ptr source_cloud);
        ~GICPAlignment() {};

        bool transform_exists;
        pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB> _gicp;

        void run();
        void iterate();
        void undo();
        Eigen::Matrix4f getFineTransform();
        void getAlignedCloud(PointCloudRGB::Ptr aligned_cloud);
        void getAlignedCloudROSMsg(sensor_msgs::PointCloud2 &aligned_cloud_msg);

    private:
        Eigen::Matrix4f _fine_tf;
        double _normal_radius;
        PointCloudRGB::Ptr _target_cloud, _source_cloud, _aligned_cloud, _backup_cloud;

        void configParameters();
        void fineAlignment(PointCloudRGB::Ptr source_cloud,
                           PointCloudRGB::Ptr target_cloud);
        
        void getCovariances(PointCloudRGB::Ptr cloud,
                            boost::shared_ptr<CovariancesVector> covs);

        void iterateFineAlignment(PointCloudRGB::Ptr cloud);
        void applyTFtoCloud(PointCloudRGB::Ptr cloud);
        void backUp(PointCloudRGB::Ptr cloud);
};