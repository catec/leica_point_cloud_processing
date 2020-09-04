// Bring in my package's API, which is what I'm testing
#include "Utils.h"
// Bring in gtest
#include <gtest/gtest.h>
#include <exception>

class TestUtils : public ::testing::Test { 
};

TEST_F(TestUtils, testCloudValidness)
{
    // create empty XYZ pointcloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudXYZ {new pcl::PointCloud<pcl::PointXYZ>};

    // create empty RGB pointcloud
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudRGB {new pcl::PointCloud<pcl::PointXYZRGB>};

    // create empty pointcloud msg
    sensor_msgs::PointCloud2 cloudMsg;

    // empty cloud are not valid
    EXPECT_FALSE(Utils::isValidCloud(cloudXYZ));
    EXPECT_FALSE(Utils::isValidCloud(cloudRGB));
    EXPECT_FALSE(Utils::isValidCloudMsg(cloudMsg));

    // fill in clouds with five points to be valid
    pcl::PointXYZ p;
    pcl::PointXYZRGB p_rgb;
    for (int i=0; i<5; i++)
    {
        p.x = p_rgb.x = i;
        p.y = p_rgb.y = i;
        p.z = p_rgb.z = i;
        p_rgb.r = i;
        p_rgb.g = i;
        p_rgb.b = i;
        cloudXYZ->push_back(p);
        cloudRGB->push_back(p_rgb);
    }
    EXPECT_TRUE(Utils::isValidCloud(cloudXYZ));
    EXPECT_TRUE(Utils::isValidCloud(cloudRGB));
}

TEST_F(TestUtils, testCloudConversions)
{
    // create XYZ pointcloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudXYZ {new pcl::PointCloud<pcl::PointXYZ>};
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudRGB {new pcl::PointCloud<pcl::PointXYZRGB>};
    sensor_msgs::PointCloud2 cloudMsg;

    // fill in cloud with five points to be valid
    pcl::PointXYZ p;
    for (int i=0; i<5; i++)
    {
        p.x = i;
        p.y = i;
        p.z = i;
        cloudXYZ->push_back(p);
    }

    // test conversion
    // XYZ to XYZRGB and XYZRGB to PointCloud2
    try
    {
        Utils::cloudToXYZRGB(cloudXYZ, cloudRGB, 255, 255, 255);
        ros::Time::init(); // because message stamp need ros::Time::now()
        Utils::cloudToROSMsg(cloudRGB, cloudMsg);
        SUCCEED();
    }
    catch(std::exception& e)
    {
        ADD_FAILURE()<< e.what();
    }        
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}