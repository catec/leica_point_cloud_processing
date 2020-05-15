#include "ros/ros.h"
#include "ros/package.h"
#include "pcl_conversions/pcl_conversions.h"
#include <pcl_ros/point_cloud.h> 
#include <pcl/io/vtk_lib_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/features/fpfh.h>
#include <pcl/features/multiscale_feature_persistence.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/registration/correspondence_estimation.h>
#include <pcl/registration/correspondence_rejection_sample_consensus.h>
#include <pcl/registration/transformation_estimation_svd.h>
#include <cad_to_pointcloud.h>

/**
* Large scans key issue:

* To find and appropiate descriptor but also an appropiate scale, for computing proccess

* One option is to look for descriptors that proves to be distintive at the scale 
* and persistent over multiple scales 
**/

const Eigen::Vector4f downsampling_leaf_size(0.01f, 0.01f, 0.01f, 0.0f);

class PointCloudAlignment 
{
    public:
        PointCloudAlignment() {};
        ~PointCloudAlignment() {};

        pcl::PointCloud<pcl::PointXYZ> pc;

        void printKeypoints(std::vector<int> const &input);
        bool getNormals(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
                        pcl::PointCloud<pcl::Normal>::Ptr &normals);
        void getKeypointsAndFeatures(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, 
                                     pcl::PointCloud<pcl::Normal>::Ptr normals,
                                     pcl::PointCloud<pcl::PointXYZ>::Ptr keypoints_cloud,
                                     pcl::PointCloud<pcl::FPFHSignature33>::Ptr features);

        void downsampleCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                             pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_downsampled);

        void initialAlingment(pcl::PointCloud<pcl::FPFHSignature33>::Ptr source_features,
                              pcl::PointCloud<pcl::FPFHSignature33>::Ptr target_features,
                              pcl::PointCloud<pcl::PointXYZ>::Ptr source_keypoints,
                              pcl::PointCloud<pcl::PointXYZ>::Ptr target_keypoints);

};

void PointCloudAlignment::downsampleCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                                          pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_downsampled)
{
    ROS_INFO("pc size before: %zu",cloud->points.size());
    pcl::VoxelGrid<pcl::PointXYZ> downsampling_filter;
    downsampling_filter.setInputCloud(cloud);
    downsampling_filter.setLeafSize(downsampling_leaf_size);
    downsampling_filter.filter(*cloud_downsampled);
    ROS_INFO("pc size after: %zu",cloud_downsampled->points.size());
}

void PointCloudAlignment::getKeypointsAndFeatures(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, 
                                                  pcl::PointCloud<pcl::Normal>::Ptr normals,
                                                  pcl::PointCloud<pcl::PointXYZ>::Ptr keypoints_cloud,
                                                  pcl::PointCloud<pcl::FPFHSignature33>::Ptr features)
{
    // SELECTED DESCRIPTOR: FPFH
    ROS_INFO("1. Set descriptor FPFH");
    pcl::FPFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33>::Ptr  
    fest(new pcl::FPFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33>);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
    fest->setInputCloud(cloud);
    fest->setInputNormals(normals);
    // Use all neighbors in a sphere of radius 5cm
    // IMPORTANT: the radius has to be larger than the radius used to estimate the surface normals!!!
    fest->setRadiusSearch (0.5);
    fest->setSearchMethod(tree);

    // ROS_INFO("2. Computing features");
    // pcl::PointCloud<pcl::FPFHSignature33>::Ptr features (new pcl::PointCloud<pcl::FPFHSignature33> ());
    // // fest->compute(*features);

    ROS_INFO("3. Define feature persistence");
    pcl::MultiscaleFeaturePersistence<pcl::PointXYZ, pcl::FPFHSignature33> fper;
    boost::shared_ptr<std::vector<int> > keypoints(new std::vector<int>); // Interest points
    std::vector<float> scale_values = { 0.5f, 1.0f, 1.5f }; //pre-selected 
    fper.setScalesVector(scale_values);
    fper.setAlpha(1.3f);
    fper.setFeatureEstimator(fest);
    fper.setDistanceMetric(pcl::CS);

    ROS_INFO("4. Extracting keypoints");
    fper.determinePersistentFeatures(*features, keypoints);

    ROS_INFO("keypoints: %zu",keypoints->size());
    ROS_INFO("features: %zu",features->size());

    pcl::ExtractIndices<pcl::PointXYZ> extract_indices_filter;
    extract_indices_filter.setInputCloud(cloud);
    extract_indices_filter.setIndices(keypoints);
    // pcl::PointCloud<pcl::PointXYZ>::Ptr persistent_features_locations(new pcl::PointCloud<pcl::PointXYZ>());
    extract_indices_filter.filter(*keypoints_cloud);

    pcl::visualization::CloudViewer viewer("Viewer for keypoints");
    viewer.showCloud(keypoints_cloud, "persistent features");
    ROS_INFO("Persistent features have been computed. Waiting for the user to quit the "
           "visualization window.\n");

    while (!viewer.wasStopped(50)) {
    }
}

void PointCloudAlignment::printKeypoints(std::vector<int> const &input)
{
	for (int i = 0; i < input.size(); i++) {
		std::cout << input.at(i) << ' ';
	}
}

bool PointCloudAlignment::getNormals(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
                                     pcl::PointCloud<pcl::Normal>::Ptr &normals)
{
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    ne.setInputCloud(cloud);

    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ> ());
    
    ne.setSearchMethod(tree);

    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
    // Use all neighbors in a sphere of radius 3cm
    ne.setRadiusSearch(0.03);
    ne.compute(*normals);

    // As we compute normal for each pointcloud, both should have same number of points
    bool success = normals->points.size()==cloud->points.size() ? true : false;
    ROS_INFO("normal size: %zu",normals->points.size());
    ROS_INFO("pc size: %zu",cloud->points.size());

    return success;
}

void PointCloudAlignment::initialAlingment(pcl::PointCloud<pcl::FPFHSignature33>::Ptr source_features,
                                           pcl::PointCloud<pcl::FPFHSignature33>::Ptr target_features,
                                           pcl::PointCloud<pcl::PointXYZ>::Ptr source_keypoints,
                                           pcl::PointCloud<pcl::PointXYZ>::Ptr target_keypoints)
{
    pcl::CorrespondencesPtr correspondences(new pcl::Correspondences);
    pcl::registration::CorrespondenceEstimation<pcl::FPFHSignature33, pcl::FPFHSignature33> cest;
    cest.setInputSource(source_features);
    cest.setInputTarget(target_features);
    cest.determineCorrespondences(*correspondences);

    pcl::CorrespondencesPtr corr_filtered(new pcl::Correspondences);
    pcl::registration::CorrespondenceRejectorSampleConsensus<pcl::PointXYZ> rejector;
    rejector.setInputSource(source_keypoints);
    rejector.setInputTarget(target_keypoints);
    rejector.setInlierThreshold(2.5);
    rejector.setMaximumIterations(1000000);
    rejector.setRefineModel(false);
    rejector.setInputCorrespondences(correspondences);;
    rejector.getCorrespondences(*corr_filtered);
    
    Eigen::Matrix4f transform;
    pcl::registration::TransformationEstimationSVD<pcl::PointXYZ, pcl::PointXYZ> trans_est;
    trans_est.estimateRigidTransformation(*source_keypoints,*target_keypoints, 
                                          *corr_filtered, transform);
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "PointCloudAlignment");

    PointCloudAlignment point_cloud_alignment;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cad_pc(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr scan_pc(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cad_pc_downsampled(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr scan_pc_downsampled(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::Normal>::Ptr cad_normals(new pcl::PointCloud<pcl::Normal>);
    pcl::PointCloud<pcl::Normal>::Ptr scan_normals(new pcl::PointCloud<pcl::Normal>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cad_keypoints(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr scan_keypoints(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::FPFHSignature33>::Ptr cad_features (new pcl::PointCloud<pcl::FPFHSignature33> ());
    pcl::PointCloud<pcl::FPFHSignature33>::Ptr scan_features (new pcl::PointCloud<pcl::FPFHSignature33> ());

    CADToPointCloud cad_to_pointcloud = CADToPointCloud("untitled.obj", cad_pc, true);

    ROS_INFO("Downsampling...");
    point_cloud_alignment.downsampleCloud(cad_pc,cad_pc_downsampled);
    point_cloud_alignment.downsampleCloud(scan_pc,scan_pc_downsampled);
    ROS_INFO("Computing normals...");
    point_cloud_alignment.getNormals(cad_pc_downsampled,cad_normals);
    point_cloud_alignment.getNormals(scan_pc_downsampled,scan_normals);
    ROS_INFO("Computing keypoints...");
    point_cloud_alignment.getKeypointsAndFeatures(cad_pc_downsampled,cad_normals,cad_keypoints,cad_features);
    point_cloud_alignment.getKeypointsAndFeatures(scan_pc_downsampled,scan_normals,scan_keypoints,scan_features);
    ROS_INFO("Getting transform...");
    point_cloud_alignment.initialAlingment(scan_features,cad_features,scan_keypoints,cad_keypoints);

    ROS_INFO("end");

    return 0;
}