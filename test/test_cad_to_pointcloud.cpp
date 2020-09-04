// Bring in my package's API, which is what I'm testing
#include "CADToPointCloud.h"
// Bring in gtest
#include <gtest/gtest.h>

class TestCADToPointCloud : public ::testing::Test { 
};

TEST_F(TestCADToPointCloud, testConversions)
{
    // create pointcloud
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudRGB {new pcl::PointCloud<pcl::PointXYZRGB>};

    // instantiate class to test
    CADToPointCloud cad_to_pointcloud;

    // test conversion
    try
    {
        cad_to_pointcloud.CADToMesh("file", cloudRGB);
        cad_to_pointcloud.MeshToPointCloud(cad_to_pointcloud.CAD_Mesh_);
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