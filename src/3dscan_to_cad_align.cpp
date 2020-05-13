#include "ros/ros.h"
#include "ros/package.h"
#include "pcl_conversions/pcl_conversions.h"
#include <pcl_ros/point_cloud.h> 
#include <pcl/io/vtk_lib_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/features/fpfh.h>
#include <pcl/features/multiscale_feature_persistence.h>
#include <pcl/features/normal_3d.h>

/**
* Large scans key issue:

* To find and appropiate descriptor but also an appropiate scale, for computing proccess

* One option is to look for descriptors that proves to be distintive at the scale 
* and persistent over multiple scales 
**/


class PointCloudAlignment {
    public:
        PointCloudAlignment() {};
        ~PointCloudAlignment() {};

        pcl::PointCloud<pcl::PointXYZ> pc;

    void getNormals(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
    boost::shared_ptr<std::vector<int> > getKeypoints(   pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, 
                                                        pcl::PointCloud<pcl::Normal>::Ptr normals);


};

boost::shared_ptr<std::vector<int>> PointCloudAlignment::getKeypoints(  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, 
                                                                        pcl::PointCloud<pcl::Normal>::Ptr normals)
{
    // SELECTED DESCRIPTOR: FPFH
    pcl::FPFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33>::Ptr 
    fest (new pcl::FPFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33>);
    fest->setInputCloud(cloud);
    fest->setInputNormals(normals);
    // Use all neighbors in a sphere of radius 5cm
    // IMPORTANT: the radius used here has to be larger than the radius used to estimate the surface normals!!!
    fest->setRadiusSearch (0.05);

    pcl::PointCloud<pcl::FPFHSignature33>::Ptr features (new pcl::PointCloud<pcl::FPFHSignature33> ());
    fest->compute(*features);

    pcl::MultiscaleFeaturePersistence<pcl::PointXYZ, pcl::FPFHSignature33> fper;
    boost::shared_ptr<std::vector<int> > keypoints; // Interest points
    std::vector<float> scale_values = { 0.5f, 1.0f, 1.5f }; //pre-selected 
    fper.setScalesVector(scale_values);
    fper.setAlpha(1.3f);
    fper.setFeatureEstimator(fest);
    fper.setDistanceMetric(pcl::CS);
    fper.determinePersistentFeatures(*features, keypoints);

    return keypoints;
}

void PointCloudAlignment::getNormals(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    ne.setInputCloud (cloud);

    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
    
    ne.setSearchMethod (tree);

    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
    // Use all neighbors in a sphere of radius 3cm
    ne.setRadiusSearch (0.03);
    ne.compute (*cloud_normals);

    ROS_INFO("normal size: %zu",cloud_normals->points.size ());
    ROS_INFO("pc size: %zu",cloud->points.size ());
    // cloud_normals->points.size () should have the same size as the input cloud->points.size ()*
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "PointCloudAlignment");

    PointCloudAlignment point_cloud_alignment;

    ros::spin();

    return 0;
}