// Bring in my package's API, which is what I'm testing
#include "InitialAlignment.h"
// Bring in gtest
#include <gtest/gtest.h>

class TestInitialAlignment : public ::testing::Test { 
};

TEST_F(TestInitialAlignment, testAlignment)
{
    // create pointcloud
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudRGB {new pcl::PointCloud<pcl::PointXYZRGB>};
    // fill in cloud with 200 points to be valid
    pcl::PointXYZRGB p_rgb;
    for (int i=0; i<200; i++)
    {
        p_rgb.x = i;
        p_rgb.y = i;
        p_rgb.z = i;
        p_rgb.r = i;
        p_rgb.g = i;
        p_rgb.b = i;
        cloudRGB->push_back(p_rgb);
    }

    // instantiate class to test
    // target and source cloud are the same to ensure alignment
    InitialAlignment initial_alignment(cloudRGB, cloudRGB);

    try
    {
        ros::Time::init(); // because need of ros::Time::now()
        initial_alignment.run();
        EXPECT_TRUE(initial_alignment.transform_exists_);
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