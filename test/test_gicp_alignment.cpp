// Bring in my package's API, which is what I'm testing
#include "GICPAlignment.h"
// Bring in gtest
#include <gtest/gtest.h>

class TestGICPAlignment : public ::testing::Test { 
};

TEST_F(TestGICPAlignment, testAlignment)
{
    // create pointcloud
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudRGB {new pcl::PointCloud<pcl::PointXYZRGB>};
    // fill in cloud with five points to be valid
    pcl::PointXYZRGB p_rgb;
    for (int i=0; i<200; i++)
    {
        p_rgb.x = std::rand();
        p_rgb.y = std::rand();
        p_rgb.z = std::rand();
        p_rgb.r = 255;
        p_rgb.g = 255;
        p_rgb.b = 255;
        cloudRGB->push_back(p_rgb);
    }

    // instantiate class to test
    // target and source cloud are the same to ensure alignment
    GICPAlignment gicp_alignment(cloudRGB, cloudRGB);

    try
    {
        ros::Time::init(); // because need of ros::Time::now()
        gicp_alignment.run();
        EXPECT_TRUE(gicp_alignment.transform_exists_);
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