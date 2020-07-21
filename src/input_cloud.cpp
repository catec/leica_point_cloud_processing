#include <Utils.h>

#include "ros/ros.h"
#include "ros/package.h"
#include "pcl_conversions/pcl_conversions.h"
#include <pcl_ros/point_cloud.h>
#include "leica_scanstation_utils/LeicaUtils.h"
#include <CADToPointCloud.h>

typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudRGB;

/**
 * POINTCLOUDS:
    - target pointcloud: directly obtain from downsampling a part's cad
    - source pointcloud: result from scanning the same cad part in Gazebo with leica c5 simulator
**/

int FREQ = 1; // Hz

int main(int argc, char** argv)
{
    ros::init(argc, argv, "input_cloud");
    ros::NodeHandle nh;

    pcl::PointCloud<pcl::PointXYZ>::Ptr cad_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr scan_cloud(new pcl::PointCloud<pcl::PointXYZ>);

    // PARAMETERS
    std::string pc_path;
    if (!nh.getParam("/pointcloud_folder", pc_path))   
    {
        ROS_ERROR("input_cloud: No pointcloud folder path on Param Server");
        return 0;
    }

    // POINTCLOUDS
    // cad_cloud is the target pointcloud directly obtain from a part's cad
    // scan_cloud is the source pointcloud created on gazebo with leica c5 simulator
    CADToPointCloud cad2pc = CADToPointCloud(pc_path,"cajon_tumbado1.obj",cad_cloud);
    std::string f = pc_path + "cajon_tumbado1.pcd";
    pcl::io::loadPCDFile<pcl::PointXYZ> (f, *scan_cloud);
    

    // Colorize clouds
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cad_cloud_rgb(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr scan_cloud_rgb(new pcl::PointCloud<pcl::PointXYZRGB>);
    Utils::cloudToXYZRGB(cad_cloud, cad_cloud_rgb, 0, 0, 255);
    Utils::cloudToXYZRGB(scan_cloud, scan_cloud_rgb, 255, 0, 128);

    // Convert to ROS data type
    ros::Publisher cad_pub = nh.advertise<sensor_msgs::PointCloud2>("/cad/cloud", 1);
    ros::Publisher scan_pub = nh.advertise<sensor_msgs::PointCloud2>("scan/cloud", 1);

    sensor_msgs::PointCloud2 cad_cloud_msg, scan_cloud_msg;
    
    pcl::toROSMsg(*cad_cloud_rgb,cad_cloud_msg);
    cad_cloud_msg.header.frame_id = Utils::_frame_id;
    cad_cloud_msg.header.stamp = ros::Time::now();

    pcl::toROSMsg(*scan_cloud_rgb,scan_cloud_msg);
    scan_cloud_msg.header.frame_id = Utils::_frame_id;
    scan_cloud_msg.header.stamp = ros::Time::now();

    ROS_INFO("input_cloud: Publishing clouds on topics: \n\t/cad/cloud \n\t/scan/cloud");
    ros::Rate r(FREQ);
    while(ros::ok())
    {
        // Publish the data
        cad_pub.publish(cad_cloud_msg);
        scan_pub.publish(scan_cloud_msg);
        ros::spinOnce();
        r.sleep();
    }

    return 0;
}