#include "GICPAlignment.h"
#include "CADToPointCloud.h"
#include "leica_scanstation_utils/LeicaUtils.h"
#include "Utils.h"
#include <Viewer.h>
#include <gtest/gtest.h>

class TestGICPAlignment : public ::testing::Test
{ 
protected:

    PointCloudRGB::Ptr sourceRGB {new PointCloudRGB};
    PointCloudRGB::Ptr targetRGB {new PointCloudRGB};
    
    TestGICPAlignment()
    {
        cubePointCloud(sourceRGB);
        Utils::rotateCloud(sourceRGB, targetRGB, 0, 0, 0.175); // radians rotation in yaw
    }

    void cubePointCloud(PointCloudRGB::Ptr cloud)
    {
        std::string pkg_path = ros::package::getPath("leica_point_cloud_processing");
        std::string f = pkg_path + "/test/cube.ply";

        int sample_points = 5000;
        
        CADToPointCloud cad2pc(f, sample_points);
        cad2pc.convertCloud(cloud);
    }
};

TEST_F(TestGICPAlignment, testApplyTF)
{
    GICPAlignment gicp_alignment(targetRGB, sourceRGB, false);

    Eigen::Matrix4f tf = gicp_alignment.getFineTransform();
    ASSERT_EQ(tf, Eigen::Matrix4f::Identity());  // constructor works well
    
    try
    {
        ros::Time::init(); // because need of ros::Time::now()
        
        gicp_alignment.run();

        gicp_alignment.applyTFtoCloud(sourceRGB);

        double tolerance = 1e-3;
        // check first point for translation result is conincident with first point in target
        EXPECT_TRUE(abs(sourceRGB->points[0].x-targetRGB->points[0].x) <= tolerance);
        EXPECT_TRUE(abs(sourceRGB->points[0].x-targetRGB->points[0].x) <= tolerance);
        EXPECT_TRUE(abs(sourceRGB->points[0].x-targetRGB->points[0].x) <= tolerance);
    }
    catch(std::exception& e)
    {
        ADD_FAILURE()<< e.what();
    }
}

TEST_F(TestGICPAlignment, testRun)
{
    GICPAlignment gicp_alignment(targetRGB, sourceRGB, false);
    
    Eigen::Matrix4f tf = gicp_alignment.getFineTransform();
    ASSERT_EQ(tf, Eigen::Matrix4f::Identity());  // constructor works well

    try
    {
        ros::Time::init(); // because need of ros::Time::now()
        
        gicp_alignment.run();

        PointCloudRGB::Ptr aligned_cloud {new PointCloudRGB};
        gicp_alignment.getAlignedCloud(aligned_cloud);

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