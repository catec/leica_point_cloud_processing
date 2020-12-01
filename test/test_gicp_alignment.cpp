// Bring in my package's API, which is what I'm testing
#include "GICPAlignment.h"
#include "CADToPointCloud.h"
#include "leica_scanstation_utils/LeicaUtils.h"
#include "Utils.h"
#include <Viewer.h>
// Bring in gtest
#include <gtest/gtest.h>


/** COMMENT - Estaria bien si se pudieran cambiar desde fuera los parametros del proceso iterativo:
 corr dist, rejection thr, euclideanFS, TfEpsilon ? **/ 

class TestGICPAlignment : public ::testing::Test
{ 
protected:

    PointCloudRGB::Ptr cubeRGB {new PointCloudRGB};
    PointCloudRGB::Ptr sourceRGB {new PointCloudRGB};
    PointCloudRGB::Ptr targetRGB {new PointCloudRGB};
    
    TestGICPAlignment()
    {
        cubePointCloud(cubeRGB);
        Utils::translateCloud(cubeRGB, sourceRGB, 3, 3, 3);
        Utils::colorizeCloud(sourceRGB, 255, 0, 0); // source red
        Utils::rotateCloud(sourceRGB, targetRGB, 0, 0, 0.03); // radians rotation in yaw
        Utils::colorizeCloud(targetRGB, 0, 0, 255); // target blue
    }

    void cubePointCloud(PointCloudRGB::Ptr cloud)
    {
        std::string cad_folder_path = LeicaUtils::findPointcloudFolderPath();
        std::string file_name = "cad.ply";
        std::string f = LeicaUtils::getFilePath(file_name);

        int sample_points = 10000;
        
        CADToPointCloud cad2pc(f, sample_points);
        cad2pc.convertCloud(cubeRGB);
    }
};


// Test GICPAlignment different RGB clouds should return success
// TEST_F(TestGICPAlignment, testCorrectAlignment)
// {
//     GICPAlignment gicp_alignment(targetRGB, sourceRGB, false);

//     Viewer v;
//     v.addPCToViewer<pcl::PointXYZRGB>(sourceRGB, "source");
//     v.addPCToViewer<pcl::PointXYZRGB>(targetRGB, "target");

//     try
//     {
//         ros::Time::init(); // because need of ros::Time::now()
//         gicp_alignment.setMaxIterations(200);
//         gicp_alignment.run();

//         PointCloudRGB::Ptr aligned_cloud {new PointCloudRGB};
//         gicp_alignment.getAlignedCloud(aligned_cloud);
//         Utils::colorizeCloud(aligned_cloud, 0, 255, 0);

//         Eigen::Matrix4f tf = gicp_alignment.getFineTransform();
//         Utils::printMatrix(tf, 4);

//         v.addPCToViewer<pcl::PointXYZRGB>(aligned_cloud, "aligned");

//         EXPECT_TRUE(gicp_alignment.transform_exists_);
//     }
//     catch(std::exception& e)
//     {
//         ADD_FAILURE()<< e.what();
//     }
// }


// Test GICPAlignment equal point clouds should return success automatically
// TEST_F(TestGICPAlignment, testAlignmentEqual)
// {
//     try
//     {
//         // instantiate class to test
//         // target and source cloud are the same to ensure alignment
//         GICPAlignment gicp_alignment(sourceRGB, sourceRGB, false);

//         ros::Time::init(); // because need of ros::Time::now()
//         gicp_alignment.run();

//         Eigen::Matrix4f tf = gicp_alignment.getFineTransform();
//         // Utils::printMatrix(tf, 4);
//         ASSERT_EQ(tf, Eigen::Matrix4f::Identity());
//     }
//     catch(std::exception& e)
//     {
//         ADD_FAILURE()<< e.what();
//     } 
// }

// Test GICPAlignment empty cloud should return error for success
TEST_F(TestGICPAlignment, testAlignmentEmpty)
{
    // create empty point cloud
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr empty_cloud {new pcl::PointCloud<pcl::PointXYZRGB>};

    try
    {
        GICPAlignment gicp_alignment(empty_cloud, sourceRGB, false);

        ros::Time::init(); // because need of ros::Time::now()
        gicp_alignment.run();

        ASSERT_FALSE(gicp_alignment.transform_exists_);
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