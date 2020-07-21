#include <Utils.h>

#include "leica_scanstation_msgs/PointCloudFile.h"
#include "leica_scanstation_utils/LeicaUtils.h"

#include <CADToPointCloud.h>

typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudRGB;

int FREQ = 1; // Hz

sensor_msgs::PointCloud2 g_cloud_msg, g_cadcloud_msg;


void getCADCloud(std::string file_name)
{
    PointCloudRGB::Ptr cloud(new PointCloudRGB);
    CADToPointCloud cad2pc = CADToPointCloud(LeicaUtils::getFilePath(file_name, ""), cloud);

    if(Utils::isValidCloud(cloud))
    {
        Utils::colorizeCloud(cloud, 0, 0, 255); // cad cloud is blue

        Utils::cloudToROSMsg(cloud, g_cadcloud_msg);

        ROS_INFO("Received cad cloud");
    }
    else
        ROS_ERROR("Couldn't read file %s", file_name.c_str());    
}


bool serviceCb(leica_scanstation_msgs::PointCloudFile::Request &req, 
               leica_scanstation_msgs::PointCloudFile::Response &res)
{
    ROS_INFO("request to publish clouds");
    PointCloudRGB::Ptr cloud(new PointCloudRGB);
    
    std::string f = LeicaUtils::getFilePath(req.file_name, ".pcd");
    int r = pcl::io::loadPCDFile<pcl::PointXYZRGB>(f, *cloud);

    if (r==-1) 
        ROS_ERROR("Couldn't read file %s", f.c_str());
    else 
    {
        Utils::colorizeCloud(cloud, 255, 0, 128); // scanned cloud is pink

        Utils::cloudToROSMsg(cloud, g_cloud_msg);

        ROS_INFO("Received scanned cloud");

        getCADCloud(req.file_name + ".obj");
    }
    
    res.message = "Service to receive cloud is correct";
	res.success =  r==-1 ? false : true;

    return true; 
}



int main(int argc, char** argv)
{
    ros::init(argc, argv, "publish_cloud");
    ros::NodeHandle nh;

    std::string pointcloud_folder_path;
    if (!nh.getParam("/pointcloud_folder", pointcloud_folder_path))   
    {
        ROS_ERROR("input_cloud: No pointcloud folder path on Param Server");
        return 0;
    }
    LeicaUtils::setPointCloudFolder(pointcloud_folder_path);

    // TODO: resolver la forma de coger el path a pointclouds
    ros::ServiceServer service = nh.advertiseService("publish_scan_file", serviceCb);
    ros::Publisher pub = nh.advertise<sensor_msgs::PointCloud2>("scan/cloud", 1);
    ros::Publisher cad_pub = nh.advertise<sensor_msgs::PointCloud2>("cad/cloud", 1);

    ROS_INFO("Service client waiting for call to /publish_scan_file");

    ros::Rate r(FREQ);
    while(ros::ok())
    {
        if (Utils::isValidCloudMsg(g_cloud_msg) && Utils::isValidCloudMsg(g_cadcloud_msg))
        {
            pub.publish(g_cloud_msg);
            cad_pub.publish(g_cadcloud_msg);
        }
        ros::spinOnce();
        r.sleep();
    }

    return 0;
}
