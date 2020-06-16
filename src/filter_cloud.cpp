#include "ros/ros.h"
#include "ros/package.h"
#include <pcl_ros/point_cloud.h> 
#include "pcl_conversions/pcl_conversions.h"
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/crop_box.h>
#include <pcl/search/kdtree.h>
#include "pcl/common/angles.h"
#include "pcl/segmentation/sac_segmentation.h"
#include <utils.h>

/**
 * POINTCLOUDS:
    - target pointcloud: directly obtain from downsampling a part's cad
    - source pointcloud: result from scanning the same cad part in Gazebo with leica c5 simulator
**/

std::string FRAME_ID = "world";
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudRGB;

double leaf_size, noise_filter_threshold, floor_filter_threshold;
ros::Publisher g_pub;

void downsampleCloud(PointCloudRGB::Ptr cloud, PointCloudRGB::Ptr cloud_downsampled, const Eigen::Vector4f leaf_size);
void filter_noise(double threshold, PointCloudRGB::Ptr cloud, PointCloudRGB::Ptr cloud_filtered);
void filter_floor(double threshold, PointCloudRGB::Ptr cloud, PointCloudRGB::Ptr cloud_filtered);


void cloudCb(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
    PointCloudRGB::Ptr cloud(new PointCloudRGB);
    pcl::fromROSMsg(*msg, *cloud);

    ROS_INFO("Downsampling cloud");
    PointCloudRGB::Ptr cloud_downsampled(new PointCloudRGB);
    const Eigen::Vector4f downsampling_leaf_size(leaf_size, leaf_size, leaf_size, 0.0f);
    downsampleCloud(cloud, cloud_downsampled, downsampling_leaf_size);

    PointCloudRGB::Ptr cloud_filtered(new PointCloudRGB);
    pcl::copyPointCloud(*cloud_downsampled,*cloud_filtered);
    
    if (noise_filter_threshold > 0)
    {
        ROS_INFO("Applying noise filter to cloud");
        filter_noise(noise_filter_threshold, cloud_filtered, cloud_filtered);
    }
    if (floor_filter_threshold > 0)
    {
        ROS_INFO("Filtering cloud floor");
        filter_floor(floor_filter_threshold, cloud_filtered, cloud_filtered);
    }

    sensor_msgs::PointCloud2 cloud_msg;
    pcl::toROSMsg(*cloud_filtered,cloud_msg);
    cloud_msg.header.frame_id = FRAME_ID;
    cloud_msg.header.stamp = ros::Time::now();

    ROS_INFO("publishing");
    g_pub.publish(cloud_msg);
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "filter");
    ros::NodeHandle nh;

    if (!nh.getParam("filter/leaf_size", leaf_size)) 
    {
        ROS_ERROR("Leaf size not found");
        return 0;
    }
    if (!nh.getParam("filter/noise_filter_threshold", noise_filter_threshold))   
    {
        ROS_ERROR("Noise filter threshold not found");
        return 0;
    }
    if (!nh.getParam("filter/floor_filter_threshold", floor_filter_threshold))   
    {
        ROS_ERROR("Floor filter threshold not found");
        return 0;
    }

    ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2>("cloud", 1, cloudCb);
    g_pub = nh.advertise<sensor_msgs::PointCloud2>("cloud_filtered", 1);

    ros::spin();

    return 0;
}

void downsampleCloud(PointCloudRGB::Ptr cloud,
                     PointCloudRGB::Ptr cloud_downsampled,
                     const Eigen::Vector4f leaf_size)
{
    double res = Utils::computeCloudResolution(cloud);
    ROS_INFO("Pointcloud size before downsampling: %zu",cloud->points.size());
    ROS_INFO("Pointcloud resolution before downsampling: %f",res);

    pcl::VoxelGrid<pcl::PointXYZRGB> downsampling_filter;
    downsampling_filter.setInputCloud(cloud);
    downsampling_filter.setLeafSize(leaf_size);
    downsampling_filter.filter(*cloud_downsampled);

    res = Utils::computeCloudResolution(cloud_downsampled);
    ROS_INFO("Pointcloud size after downsampling: %zu",cloud_downsampled->points.size());
    ROS_INFO("Pointcloud resolution after downsampling: %f",res);
}

void filter_noise(double threshold, PointCloudRGB::Ptr cloud, PointCloudRGB::Ptr cloud_filtered)
{
    ROS_INFO("create box with threshold: %f",threshold);
    pcl::CropBox<pcl::PointXYZRGB> boxFilter;
    
    //get box center
    Eigen::Vector3f boxTranslatation;
    Eigen::Vector4f centroid;
    pcl::compute3DCentroid(*cloud, centroid);
    boxTranslatation[0]=centroid[0];  
    boxTranslatation[1]=centroid[1];  
    boxTranslatation[2]=centroid[2];
    ROS_INFO("Point cloud center: (%f,%f,%f)",centroid[0],centroid[1],centroid[2]);

    // apply filter
    boxFilter.setTranslation(boxTranslatation);
    boxFilter.setMin(Eigen::Vector4f(-threshold, -threshold, -threshold, 1.0));
    boxFilter.setMax(Eigen::Vector4f(threshold, threshold, threshold, 1.0));
    boxFilter.setInputCloud(cloud);
    boxFilter.filter(*cloud_filtered);
}

void filter_floor(double threshold, PointCloudRGB::Ptr cloud, PointCloudRGB::Ptr cloud_filtered)
{
    ROS_INFO("create segmenter with threshold: %f",threshold);
    pcl::SACSegmentation<pcl::PointXYZRGB> seg;

    // Search for a plane perpendicular to z axis, 1 degree tolerance
    Eigen::Vector3f axis;
    axis << 0, 0, 1;
    seg.setAxis(axis);
    seg.setEpsAngle(pcl::deg2rad(1.0));seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(threshold);
    seg.setInputCloud(cloud);
    
    ROS_INFO("apply");
    pcl::ModelCoefficients coeff;
    pcl::PointIndices indices_internal;
    seg.segment(indices_internal, coeff);

    if (indices_internal.indices.size() == 0) 
    {
        ROS_ERROR("Unable to find floor.");
    }
    else
    {
        pcl::ExtractIndices<pcl::PointXYZRGB> extract_indices_filter;
        extract_indices_filter.setInputCloud(cloud);
        extract_indices_filter.setIndices(boost::make_shared<std::vector<int>>(indices_internal.indices));
        extract_indices_filter.setNegative(true);
        extract_indices_filter.filter(*cloud_filtered);
    }
}