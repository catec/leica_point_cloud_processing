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
#include <cad_to_pointcloud.h>

/**
* Large scans key issue:

* To find and appropiate descriptor but also an appropiate scale, for computing proccess

* One option is to look for descriptors that proves to be distintive at the scale 
* and persistent over multiple scales 
**/


class PointCloudAlignment 
{
    public:
        PointCloudAlignment() {};
        ~PointCloudAlignment() {};

        pcl::PointCloud<pcl::PointXYZ> pc;

        void printKeypoints(std::vector<int> const &input);
        bool getNormals(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
                        pcl::PointCloud<pcl::Normal>::Ptr &normals);
        boost::shared_ptr<std::vector<int> > getKeypoints(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, 
                                                          pcl::PointCloud<pcl::Normal>::Ptr normals);

};

boost::shared_ptr<std::vector<int>> PointCloudAlignment::getKeypoints(  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, 
                                                                        pcl::PointCloud<pcl::Normal>::Ptr normals)
{
    // SELECTED DESCRIPTOR: FPFH
    ROS_INFO("1. Set descriptor FPFH");
    pcl::FPFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33>::Ptr  
    fest (new pcl::FPFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33>);
    fest->setInputCloud(cloud);
    fest->setInputNormals(normals);
    // Use all neighbors in a sphere of radius 5cm
    // IMPORTANT: the radius used here has to be larger than the radius used to estimate the surface normals!!!
    fest->setRadiusSearch (0.05);

    ROS_INFO("2. Computing features");
    pcl::PointCloud<pcl::FPFHSignature33>::Ptr features (new pcl::PointCloud<pcl::FPFHSignature33> ());
    // fest->compute(*features);

    ROS_INFO("3. Define feature persistence");
    pcl::MultiscaleFeaturePersistence<pcl::PointXYZ, pcl::FPFHSignature33> fper;
    boost::shared_ptr<std::vector<int> > keypoints (new std::vector<int>); // Interest points
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
    pcl::PointCloud<pcl::PointXYZ>::Ptr persistent_features_locations(new pcl::PointCloud<pcl::PointXYZ>());
    extract_indices_filter.filter(*persistent_features_locations);

    pcl::visualization::CloudViewer viewer("Viewer for keypoints");
    viewer.showCloud(persistent_features_locations, "persistent features");
    PCL_INFO("Persistent features have been computed. Waiting for the user to quit the "
           "visualization window.\n");

    while (!viewer.wasStopped(50)) {
    }

    return keypoints;
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


int main(int argc, char** argv)
{
    ros::init(argc, argv, "PointCloudAlignment");

    PointCloudAlignment point_cloud_alignment;
    pcl::PointCloud<pcl::PointXYZ>::Ptr pc(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);

    CADToPointCloud cad_to_pointcloud = CADToPointCloud("untitled.obj", pc, true);

    ROS_INFO("Computing normals...");
    point_cloud_alignment.getNormals(pc,normals);
    ROS_INFO("Computing keypoints...");
    point_cloud_alignment.getKeypoints(pc,normals);

    ROS_INFO("end");

    return 0;
}