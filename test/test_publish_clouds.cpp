#include <ros/ros.h>
#include <gtest/gtest.h>
#include "Utils.h"
#include "leica_scanstation_utils/LeicaUtils.h"
#include "leica_scanstation_msgs/PointCloudFile.h"
#include "sensor_msgs/PointCloud2.h"

typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudRGB;

// Test call PointCloudFile service in load_and_publish_clouds node, it should return success
TEST(TestNode, testService)
{
    ros::NodeHandle nh;
    ros::ServiceClient client;
    client = nh.serviceClient<leica_scanstation_msgs::PointCloudFile>("publish_clouds");
    bool exists = client.waitForExistence(ros::Duration(1));
    ASSERT_TRUE(exists);

    leica_scanstation_msgs::PointCloudFile srv;
    srv.request.source_cloud_file = "scan.pcd";
    srv.request.target_cloud_file = "cad.ply";
    
    if(client.call(srv)){
        ASSERT_TRUE(srv.response.success);
    }
    else ADD_FAILURE()<< "Failed to call service";
}

// Test call PointCloudFile service in load_and_publish_clouds node, it should return fail
TEST(TestNode, testServiceFail)
{
    ros::NodeHandle nh;
    ros::ServiceClient client;
    client = nh.serviceClient<leica_scanstation_msgs::PointCloudFile>("publish_clouds");
    bool exists = client.waitForExistence(ros::Duration(1));
    ASSERT_TRUE(exists);

    leica_scanstation_msgs::PointCloudFile srv;
    srv.request.source_cloud_file = "scan"; // file names without extension
    srv.request.target_cloud_file = "cad";  // service should fail
    
    if(client.call(srv)){
        ADD_FAILURE() << "Service doesn't work as expected";
    }
    else SUCCEED(); 
}

// Test load_and_publish_clouds node is publishing point_clouds
TEST(TestNode, testTopics)
{
    sensor_msgs::PointCloud2ConstPtr cloud_msg; 
    cloud_msg = ros::topic::waitForMessage<sensor_msgs::PointCloud2>(SOURCE_CLOUD_TOPIC);
    EXPECT_TRUE(Utils::isValidCloudMsg(*cloud_msg));

    cloud_msg = ros::topic::waitForMessage<sensor_msgs::PointCloud2>(TARGET_CLOUD_TOPIC);
    EXPECT_TRUE(Utils::isValidCloudMsg(*cloud_msg));
}

int main(int argc,char **argv)
{
  ros::init(argc, argv, "test_publish_clouds");
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}