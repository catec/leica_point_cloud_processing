// Bring in my package's API, which is what I'm testing
#include "InitialAlignment.h"
#include "CADToPointCloud.h"
#include "leica_scanstation_utils/LeicaUtils.h"
#include <Viewer.h>
// Bring in gtest
#include <gtest/gtest.h>

typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudRGB;

class TestInitialAlignment : public ::testing::Test
{ 
protected:

    PointCloudRGB::Ptr cloudRGB {new PointCloudRGB};
    PointCloudRGB::Ptr sourceRGB {new PointCloudRGB};
    PointCloudRGB::Ptr targetRGB {new PointCloudRGB};

    TestInitialAlignment()
    {
        cubePointCloud(sourceRGB);
        Utils::translateCloud(sourceRGB, targetRGB, 3, 3, 3);
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

TEST_F(TestInitialAlignment, testApplyTF)
{
    InitialAlignment initial_alignment(targetRGB, sourceRGB);

    Eigen::Matrix4f tf = initial_alignment.getRigidTransform();
    ASSERT_EQ(tf, Eigen::Matrix4f::Identity());  // constructor works well
    
    initial_alignment.run(); // run method works well
    
    initial_alignment.applyTFtoCloud(sourceRGB);

    double tolerance = 1e-3;
    // check first point for translation result is conincident with first point in target
    EXPECT_TRUE(abs(sourceRGB->points[0].x-targetRGB->points[0].x) <= tolerance);
    EXPECT_TRUE(abs(sourceRGB->points[0].x-targetRGB->points[0].x) <= tolerance);
    EXPECT_TRUE(abs(sourceRGB->points[0].x-targetRGB->points[0].x) <= tolerance);
}

TEST_F(TestInitialAlignment, testTF)
{
    InitialAlignment initial_alignment(targetRGB, sourceRGB);

    Eigen::Matrix4f tf = initial_alignment.getRigidTransform();
    ASSERT_EQ(tf, Eigen::Matrix4f::Identity());  // constructor works well
    
    initial_alignment.run(); // run method works well
    
    tf = initial_alignment.getRigidTransform();

    double translation = 3;
    double tolerance = 1e-3;
    // check translation result is translation value applied to target
    EXPECT_TRUE(abs(tf.col(3)[0]-translation) <= tolerance);
    EXPECT_TRUE(abs(tf.col(3)[1]-translation) <= tolerance);
    EXPECT_TRUE(abs(tf.col(3)[2]-translation) <= tolerance);
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}