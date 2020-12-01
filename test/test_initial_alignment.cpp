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

    PointCloudRGB::Ptr cubeRGB {new PointCloudRGB};
    PointCloudRGB::Ptr sourceRGB {new PointCloudRGB};
    PointCloudRGB::Ptr targetRGB {new PointCloudRGB};

    TestInitialAlignment()
    {
        cubePointCloud(cubeRGB);
        Utils::translateCloud(cubeRGB, sourceRGB, 3, 3, 3);
        Utils::colorizeCloud(sourceRGB, 255,0,0);
        Utils::translateCloud(cubeRGB, targetRGB, 6, 6, 6);
        Utils::colorizeCloud(targetRGB, 0,0,255);
    }

    void cubePointCloud(PointCloudRGB::Ptr cloud)
    {
        std::string cad_folder_path = LeicaUtils::findPointcloudFolderPath();
        std::string file_name = "cube.ply";
        std::string f = LeicaUtils::getFilePath(file_name);

        int sample_points = 50000;
        
        CADToPointCloud cad2pc(f, sample_points);
        cad2pc.convertCloud(cloud); // cubeRGB ??
    }
};

/*TODO: HAY QUE QUITAR EL VIEWER EN LOS TESTS*/

    
// TEST_F(TestInitialAlignment, testEmptyCloud)
// {
//     PointCloudRGB::Ptr empty_cloud {new PointCloudRGB};
//     InitialAlignment initial_alignment(empty_cloud, sourceRGB);

//     Eigen::Matrix4f tf = initial_alignment.getRigidTransform();
//     ASSERT_EQ(tf, Eigen::Matrix4f::Identity());

//     initial_alignment.run();
    
//     tf = initial_alignment.getRigidTransform();
//     Eigen::Vector3f translation(5, 5, 5); // applied to target on TestInitialAlignment

//     ASSERT_EQ(translation[0], tf.col(3)[0]); // translation x on matrix
//     ASSERT_EQ(translation[1], tf.col(3)[1]); // translation y on matrix
//     ASSERT_EQ(translation[2], tf.col(3)[2]); // translation z on matrix
// }


// TEST_F(TestInitialAlignment, testDefaultMethod)
// {
//     // Viewer v;
//     // v.addPCToViewer<pcl::PointXYZRGB>(cloud, "source");

//     InitialAlignment initial_alignment(targetRGB, sourceRGB);

//     Eigen::Matrix4f tf = initial_alignment.getRigidTransform();
//     ASSERT_EQ(tf, Eigen::Matrix4f::Identity());  // constructor works well

//     initial_alignment.run(); // run method works well
    
//     tf = initial_alignment.getRigidTransform(); // getRigidTransform method works well
//     Eigen::Vector3f translation(3, 3, 3); // applied to target on TestInitialAlignment

//     // ASSERT_EQ(translation[0], tf.col(3)[0]); // translation x on matrix
//     // ASSERT_EQ(translation[1], tf.col(3)[1]); // translation y on matrix
//     // ASSERT_EQ(translation[2], tf.col(3)[2]); // translation z on matrix

//     std::cout << tf.col(3)[0] << std::endl;
//     EXPECT_TRUE((tf.col(3)[0] >= translation[0]-0.05) && (tf.col(3)[0] <= translation[0]+0.05));
//     EXPECT_TRUE((tf.col(3)[1] >= translation[1]-0.05) && (tf.col(3)[1] <= translation[1]+0.05));
//     EXPECT_TRUE((tf.col(3)[2] >= translation[2]-0.05) && (tf.col(3)[2] <= translation[2]+0.05));
// }


// TEST_F(TestInitialAlignment, testSetMethod)
// {
//     InitialAlignment initial_alignment(targetRGB, sourceRGB);

//     Eigen::Matrix4f tf = initial_alignment.getRigidTransform();
//     ASSERT_EQ(tf, Eigen::Matrix4f::Identity());

//     initial_alignment.setMethod(AlignmentMethod::HARRIS);

//     initial_alignment.run();
    
//     tf = initial_alignment.getRigidTransform();
//     Eigen::Vector3f translation(3, 3, 3); // applied to target on TestInitialAlignment

//     std::cout << tf.col(3)[0] << std::endl;
//     EXPECT_TRUE((tf.col(3)[0] >= translation[0]-0.05) && (tf.col(3)[0] <= translation[0]+0.05));
//     EXPECT_TRUE((tf.col(3)[1] >= translation[1]-0.05) && (tf.col(3)[1] <= translation[1]+0.05));
//     EXPECT_TRUE((tf.col(3)[2] >= translation[2]-0.05) && (tf.col(3)[2] <= translation[2]+0.05));
//     EXPECT_NE(tf, Eigen::Matrix4f::Identity());
// }


// TEST_F(TestInitialAlignment, testNormalsMethod)
// {
//     InitialAlignment initial_alignment(targetRGB, sourceRGB);

//     Eigen::Matrix4f tf = initial_alignment.getRigidTransform();
//     ASSERT_EQ(tf, Eigen::Matrix4f::Identity());

//     initial_alignment.runNormalsBasedAlgorithm();
    
//     tf = initial_alignment.getRigidTransform();
//     Eigen::Vector3f translation(3, 3, 3); // applied to target on TestInitialAlignment

//     std::cout << tf.col(3)[0] << std::endl;
//     EXPECT_TRUE((tf.col(3)[0] >= translation[0]-0.05) && (tf.col(3)[0] <= translation[0]+0.05));
//     EXPECT_TRUE((tf.col(3)[1] >= translation[1]-0.05) && (tf.col(3)[1] <= translation[1]+0.05));
//     EXPECT_TRUE((tf.col(3)[2] >= translation[2]-0.05) && (tf.col(3)[2] <= translation[2]+0.05));
//     EXPECT_NE(tf, Eigen::Matrix4f::Identity());
// }

// TEST_F(TestInitialAlignment, testHarrisMethod)
// {
//     InitialAlignment initial_alignment(targetRGB, sourceRGB);

//     Eigen::Matrix4f tf = initial_alignment.getRigidTransform();
//     ASSERT_EQ(tf, Eigen::Matrix4f::Identity());

//     initial_alignment.setNormals();
//     initial_alignment.runKeypointsBasedAlgorithm();
    
//     tf = initial_alignment.getRigidTransform();
//     Eigen::Vector3f translation(3, 3, 3); // applied to target on TestInitialAlignment

//     std::cout << tf.col(3)[0] << std::endl;
//     EXPECT_TRUE((tf.col(3)[0] >= translation[0]-0.05) && (tf.col(3)[0] <= translation[0]+0.05));
//     EXPECT_TRUE((tf.col(3)[1] >= translation[1]-0.05) && (tf.col(3)[1] <= translation[1]+0.05));
//     EXPECT_TRUE((tf.col(3)[2] >= translation[2]-0.05) && (tf.col(3)[2] <= translation[2]+0.05));
//     EXPECT_NE(tf, Eigen::Matrix4f::Identity());
// }


// Test  InitialAlignment constructor with empty cloud, should return error
// Deberia fallar al hacer computeCloudResolution de una nube vacia
// TEST_F(TestInitialAlignment, testConstructor)
// {
//     PointCloudRGB::Ptr empty_cloud {new PointCloudRGB};

//     try
//     {
//         InitialAlignment initial_alignment(empty_cloud, cloudRGB);
//         ADD_FAILURE();
//     }
//     catch(std::exception& e)
//     {
//         ADD_FAILURE()<< e.what();
//     }     
// }


TEST_F(TestInitialAlignment, testApplyTF)
{

    InitialAlignment initial_alignment(targetRGB, sourceRGB);

    Eigen::Matrix4f tf = initial_alignment.getRigidTransform();
    ASSERT_EQ(tf, Eigen::Matrix4f::Identity());  // constructor works well

    tf.col(3)[0] = tf.col(3)[1] = tf.col(3)[2] = 3;
    initial_alignment.applyTFtoCloud(sourceRGB, tf);
    
    initial_alignment.run(); // run method works well
    
    tf = initial_alignment.getRigidTransform();

    std::cout << tf.col(3)[0] << std::endl;
    std::cout << tf.col(3)[1] << std::endl;
    std::cout << tf.col(3)[2] << std::endl;
    Eigen::Vector3f translation(0, 0, 0); // no traslation
    EXPECT_TRUE((tf.col(3)[0] >= translation[0]-0.05) && (tf.col(3)[0] <= translation[0]+0.05));
    EXPECT_TRUE((tf.col(3)[1] >= translation[1]-0.05) && (tf.col(3)[1] <= translation[1]+0.05));
    EXPECT_TRUE((tf.col(3)[2] >= translation[2]-0.05) && (tf.col(3)[2] <= translation[2]+0.05));
}


// Run all the tests that were declared with TEST()
int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}