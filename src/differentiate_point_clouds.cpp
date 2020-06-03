#include <stdlib.h> 
#include <string>
#include <pcl_ros/point_cloud.h> 
// #include "pcl/common/angles.h"
// #include "pcl/sample_consensus/method_types.h"
// #include "pcl/sample_consensus/model_types.h"
// #include <pcl/filters/crop_box.h>
#include <pcl/surface/organized_fast_mesh.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/mls.h>
#include <pcl/io/pcd_io.h>
#include <pcl/surface/gp3.h>
#include <pcl_ros/point_cloud.h> 
#include <pcl/filters/extract_indices.h>
#include <pcl/octree/octree_pointcloud_changedetector.h>
#include <pcl/segmentation/extract_clusters.h>
#include <std_msgs/Int16.h>
#include <cad_to_pointcloud.h>

#define RESOLUTION 0.5
#define PUBLISH_TIME 10

int main(int argc, char** argv)
{
    ros::init(argc, argv, "DiffOnPointclouds");
    ros::NodeHandle nh;

    double voxel_resolution; // Octree resolution - side length of octree voxels
    if (argc<2)
    {
        voxel_resolution = RESOLUTION;
        ROS_WARN("Resolution not specified. Set to default: %f",voxel_resolution);
    }
    else
    {
        voxel_resolution = atof(argv[1]);
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr cad_pc(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr scan_pc(new pcl::PointCloud<pcl::PointXYZ>);

    ROS_INFO("open file");
    CADToPointCloud cad_to_pointcloud;
    std::string f = cad_to_pointcloud._pc_path + "conjunto_estranio_cad.pcd";
    pcl::io::loadPCDFile<pcl::PointXYZ> (f, *cad_pc);
    f = cad_to_pointcloud._pc_path + "conjunto_estranio_fod_aligned.pcd";
    pcl::io::loadPCDFile<pcl::PointXYZ> (f, *scan_pc);

    cad_to_pointcloud.visualizePointCloud(scan_pc,cad_to_pointcloud.PINK);
    // cad_to_pointcloud.addPCToVisualizer(cad_pc,cad_to_pointcloud.WHITE,"scan");

    pcl::octree::OctreePointCloudChangeDetector<pcl::PointXYZ> octree(voxel_resolution);
    octree.setInputCloud(cad_pc);
    octree.addPointsFromInputCloud();

    // Switch octree buffers: This resets octree but keeps previous tree structure in memory.
    octree.switchBuffers();

    octree.setInputCloud(scan_pc);
    octree.addPointsFromInputCloud();

    ROS_INFO("Extracting differences");
    boost::shared_ptr<std::vector<int> > indices(new std::vector<int>); // Diferentiated points
    octree.getPointIndicesFromNewVoxels(*indices);

    pcl::PointCloud<pcl::PointXYZ>::Ptr diff_pc(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::ExtractIndices<pcl::PointXYZ> extract_indices_filter;
    extract_indices_filter.setInputCloud(scan_pc);
    extract_indices_filter.setIndices(indices);
    extract_indices_filter.filter(*diff_pc);

    if (diff_pc->size()<=0) 
    {
        ROS_ERROR("No differences found. Try a small resolution");
        return 0;
    }

    cad_to_pointcloud._point_size = 3;
    // cad_to_pointcloud.visualizePointCloud(diff_pc,cad_to_pointcloud.BLUE);
    cad_to_pointcloud.addPCToVisualizer(diff_pc,cad_to_pointcloud.BLUE,"diff");

    ROS_INFO("Getting possible fods");
    // get possible fods
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(diff_pc);

    std::vector<pcl::PointIndices>cluster_indices; //This is a vector of cluster
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance(voxel_resolution);
    ec.setMinClusterSize(2);
    ec.setMaxClusterSize(25000);
    ec.setSearchMethod(tree);
    ec.setInputCloud(diff_pc);
    ec.extract(cluster_indices);

    // iterate through cluster_indices to create a pc for each cluster
    int R,G,B,j=0;
    std::string cluster_name,topic_name;
    std::vector<sensor_msgs::PointCloud2> cluster_msg_array;
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZ>);
        for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit)
            cloud_cluster->points.push_back(diff_pc->points[*pit]); //*
        cloud_cluster->width = cloud_cluster->points.size();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;

        cluster_name = "/fod"+std::to_string(j);
        cad_to_pointcloud.addPCToVisualizer(cloud_cluster,cad_to_pointcloud.GREEN,cluster_name);
        j++;

        // apply color to publish
        R = 255;
        G = 0;
        B = 128;
        sensor_msgs::PointCloud2 cluster_msg;

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_rgb(new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::copyPointCloud(*cloud_cluster,*cloud_rgb);
        for (size_t i = 0; i < cloud_rgb->points.size(); i++)
        {
            cloud_rgb->points[i].r = R;
            cloud_rgb->points[i].g = G;
            cloud_rgb->points[i].b = B;
        }
        pcl::toROSMsg(*cloud_rgb,cluster_msg);
        cluster_msg.header.frame_id = "world";
        cluster_msg.header.stamp = ros::Time::now();
        cluster_msg_array.push_back(cluster_msg);
    }
    ROS_INFO("Detected %d objects",j);
    // publish total of clusters
    ros::Publisher npub = nh.advertise<std_msgs::Int16>("/num_of_fods_detected", 1);
    std_msgs::Int16 msg;
    msg.data = j;
    npub.publish(msg);
    ROS_INFO("Publishing on topics /fodX");
    // publish each fod in a separated topic
    while(ros::ok())
    {
        for(int i=0; i<j; i++)
        {
            topic_name = "/fod"+std::to_string(i);
            ros::Publisher pub = nh.advertise<sensor_msgs::PointCloud2>(topic_name, 1);

            pub.publish(cluster_msg_array[i]);
        }
    }

    return 0;
}