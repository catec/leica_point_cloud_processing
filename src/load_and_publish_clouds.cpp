#include <Utils.h>

#include "leica_scanstation_msgs/PointCloudFile.h"
#include "leica_scanstation_utils/LeicaUtils.h"

#include <CADToPointCloud.h>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloudXYZ;
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudRGB;

std::string TARGET_CLOUD_TOPIC = "/cad/cloud";
std::string SOURCE_CLOUD_TOPIC = "/scan/cloud";
int FREQ = 1;  // Hz

sensor_msgs::PointCloud2 g_cloud_msg, g_cadcloud_msg;

int getCADCloud(std::string file_name)
{
  PointCloudRGB::Ptr cloud(new PointCloudRGB);
  CADToPointCloud cad2pc = CADToPointCloud(LeicaUtils::getFilePath(file_name, ""), cloud);

  if (Utils::isValidCloud(cloud))
  {
    Utils::colorizeCloud(cloud, 0, 0, 255);  // cad cloud is blue
    Utils::cloudToROSMsg(cloud, g_cadcloud_msg);
    ROS_INFO("Received cad cloud");
  }
  else
  {
    ROS_ERROR("Couldn't read file %s", file_name.c_str());
    return -1;
  }

  return 0;
}

int getScanCloud(std::string file_name)
{
  PointCloudRGB::Ptr cloud(new PointCloudRGB);

  std::string f = LeicaUtils::getFilePath(file_name, ".pcd");
  ROS_INFO("Converting file: %s", f.c_str());
  int r = pcl::io::loadPCDFile<pcl::PointXYZRGB>(f, *cloud);

  if (r != -1 && Utils::isValidCloud(cloud))
  {
    Utils::colorizeCloud(cloud, 255, 0, 128);  // scanned cloud is pink
    Utils::cloudToROSMsg(cloud, g_cloud_msg);
    ROS_INFO("Received scanned cloud");
  }
  else
  {
    ROS_ERROR("Couldn't read file %s", f.c_str());
    return -1;
  }

  return 0;
}

bool serviceCb(leica_scanstation_msgs::PointCloudFile::Request& req,
               leica_scanstation_msgs::PointCloudFile::Response& res)
{
  ROS_INFO("Request to publish clouds");

  int r = getScanCloud(req.file_name);
  if (r == 0)
  {
    r = getCADCloud(req.file_name + ".obj");
    ROS_INFO("Publishing clouds on topics: \n\t%s \n\t%s", TARGET_CLOUD_TOPIC.c_str(), SOURCE_CLOUD_TOPIC.c_str());
  }

  res.message = "Finished service to receive cloud";
  res.success = r == -1 ? false : true;

  return true;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "cloud_publisher");
  ros::NodeHandle nh;

  std::string pointcloud_folder_path;
  if (!nh.getParam("/pointcloud_folder", pointcloud_folder_path))
  {
    ROS_ERROR("No pointcloud folder path on Param Server");
    return 0;
  }
  LeicaUtils::setPointCloudPath(pointcloud_folder_path);
  ROS_INFO("Search for pointcloud in %s", pointcloud_folder_path.c_str());

  ros::ServiceServer service = nh.advertiseService("publish_clouds", serviceCb);
  ros::Publisher pub = nh.advertise<sensor_msgs::PointCloud2>("scan/cloud", 1);
  ros::Publisher cad_pub = nh.advertise<sensor_msgs::PointCloud2>("cad/cloud", 1);

  ROS_INFO("Service waiting for call to /publish_clouds");

  ros::Rate r(FREQ);
  while (ros::ok())
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
