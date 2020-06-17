#include "ros/ros.h"
#include "ros/package.h"
#include "std_srvs/Trigger.h"
#include "pcl_conversions/pcl_conversions.h"
#include <pcl_ros/point_cloud.h>
#include <cad_to_pointcloud.h>
// #include <utils.h>
#include <filter.h>
// #include <initial_alignment.h>
#include <gicp_alignment.h>


/**
 * POINTCLOUDS:
    - target pointcloud: directly obtain from downsampling a part's cad
    - source pointcloud: result from scanning the same cad part in Gazebo with leica c5 simulator
**/

std::string FRAME_ID = "world";
int FREQ = 5; // Hz

bool iterate = false;
bool undo_last_iteration = false;

bool gicpCb(std_srvs::Trigger::Request  &req,
            std_srvs::Trigger::Response &res)
{
    iterate = true; 

    res.success = true;
    res.message = "iteration";
}

bool undoCb(std_srvs::Trigger::Request  &req,
            std_srvs::Trigger::Response &res)
{
    undo_last_iteration = true; 

    res.success = true;
    res.message = "undo";
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "point_cloud_processing");
    ros::NodeHandle nh;

    // PARAMETERS
    std::string pointcloud_folder_path;
    if (!nh.getParam("/pointcloud_folder", pointcloud_folder_path))   
    {
        ROS_ERROR("input_cloud: No pointcloud folder path on Param Server");
        return 0;
    }

    // POINTCLOUDS
    // cad_pc is the target pointcloud directly obtain from a part's cad
    // scan_pc is the source pointcloud created on gazebo with leica c5 simulator
    pcl::PointCloud<pcl::PointXYZ>::Ptr cad_pc(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr scan_pc(new pcl::PointCloud<pcl::PointXYZ>);

    CADToPointCloud cad_to_pointcloud = CADToPointCloud(pointcloud_folder_path,"conjunto_estranio_cad.obj",cad_pc, false);
    std::string f = pointcloud_folder_path + "conjunto_estranio_scan.pcd";
    pcl::io::loadPCDFile<pcl::PointXYZ> (f, *scan_pc);

    // Colorize clouds
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cad_pc_rgb(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr scan_pc_rgb(new pcl::PointCloud<pcl::PointXYZRGB>);
    Utils::cloudToXYZRGB(cad_pc, cad_pc_rgb, 0, 0, 255);
    Utils::cloudToXYZRGB(scan_pc, scan_pc_rgb, 255, 0, 128);

    // Filter clouds
    Filter cad_cloud_filter(0.05);
    Filter scan_cloud_filter(0.05, 10, 0.8);
    PointCloudRGB::Ptr cad_cloud_downsampled(new PointCloudRGB);
    PointCloudRGB::Ptr scan_cloud_filtered(new PointCloudRGB);
    cad_cloud_filter.run(cad_pc_rgb, cad_cloud_downsampled);
    scan_cloud_filter.run(scan_pc_rgb, scan_cloud_filtered);
    
    // Get initial alignment
    InitialAlignment initial_alignment(cad_cloud_downsampled, scan_cloud_filtered);
    initial_alignment.run();
    // Eigen::Matrix4f rigid_transform = initial_alignment.getRigidTransform();
    Utils::printTransform(initial_alignment.getRigidTransform());
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr scan_pc_aligned(new pcl::PointCloud<pcl::PointXYZRGB>);
    initial_alignment.getAlignedCloud(scan_pc_aligned);

    // Get fine alignment
    GICPAlignment gicp_alignment(cad_cloud_downsampled, scan_pc_aligned);
    gicp_alignment.run();
    Utils::printTransform(gicp_alignment.getFineTransform());
    gicp_alignment.getAlignedCloud(scan_pc_aligned);
    ROS_INFO("final transform:");
    Eigen::Matrix4f final_transform = gicp_alignment.getFineTransform() * initial_alignment.getRigidTransform();
    Utils::printTransform(final_transform);

    // TODO aqui poner service para iterar gicp
    ros::ServiceServer gicp_service = nh.advertiseService("iterate_gicp", gicpCb);
    ros::ServiceServer undo_service = nh.advertiseService("undo_iteration", undoCb);

    // Convert to ROS data type
    ros::Publisher cad_pub = nh.advertise<sensor_msgs::PointCloud2>("/cad/cloud_filtered", 1);
    ros::Publisher scan_pub = nh.advertise<sensor_msgs::PointCloud2>("scan/cloud_aligned", 1);

    sensor_msgs::PointCloud2 cad_cloud_msg, scan_cloud_msg;

    pcl::toROSMsg(*cad_cloud_downsampled,cad_cloud_msg);
    cad_cloud_msg.header.frame_id = FRAME_ID;
    cad_cloud_msg.header.stamp = ros::Time::now();

    pcl::toROSMsg(*scan_pc_aligned,scan_cloud_msg);
    scan_cloud_msg.header.frame_id = FRAME_ID;
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

        if(iterate)
        {
            gicp_alignment.iterate();
            gicp_alignment.getAlignedCloudROSMsg(scan_cloud_msg);
            iterate = false;
        }

        if (undo_last_iteration)
        {
            gicp_alignment.undo();
            gicp_alignment.getAlignedCloudROSMsg(scan_cloud_msg);
            undo_last_iteration = false;
        }

    }

    return 0;
}