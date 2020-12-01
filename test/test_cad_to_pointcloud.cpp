// Bring in my package's API, which is what I'm testing
#include "CADToPointCloud.h"
#include "leica_scanstation_utils/LeicaUtils.h"
// Bring in gtest
#include <gtest/gtest.h>


// Test  CADToPointCloud constructor should return success
TEST(TestCADToPointCloud, testCADtoPC)
{
    std::string cad_folder_path = LeicaUtils::findPointcloudFolderPath();
    std::string file_name = "cube.ply";
    std::string f = LeicaUtils::getFilePath(file_name);

    int sample_points = 10000;
    
    PointCloudRGB::Ptr cubeRGB {new PointCloudRGB};

    try
    {
        CADToPointCloud cad2pc(f, sample_points);
        cad2pc.convertCloud(cubeRGB);

        ASSERT_TRUE(Utils::isValidCloud(cubeRGB));
        ASSERT_GT(cubeRGB->size(),1);
    }
    catch(std::exception& e)
    {
        ADD_FAILURE()<< e.what();
    }
}


/* COMMENT - en los siguientes tests creo que hay que hacer un
CONTROL DE EXTENSION en la clase, ya que salta error del VTK y se para por completo la ejecución */


// Test CADToPointCloud constructor should return error: file extension is lost
// TEST(TestCADToPointCloud, testNoExtension)
// {
//     std::string cad_folder_path = LeicaUtils::findPointcloudFolderPath();
//     std::string file_name = "cube";
//     std::string f = LeicaUtils::getFilePath(file_name);

//     int sample_points = 10000;
    
//     PointCloudRGB::Ptr cubeRGB {new PointCloudRGB};

//     CADToPointCloud cad2pc(f, sample_points);
//     cad2pc.convertCloud(cubeRGB);

//     ASSERT_TRUE(Utils::isValidCloud(cubeRGB));
//     ASSERT_GT(cubeRGB->size(),1);


//     // try
//     // {
//     //     CADToPointCloud cad2pc(f, sample_points);
//     //     cad2pc.convertCloud(cubeRGB);

//     //     ASSERT_TRUE(Utils::isValidCloud(cubeRGB));
//     //     ASSERT_GT(cubeRGB->size(),1);
//     // }
//     // catch(std::exception& e)
//     // {
//     //     ADD_FAILURE()<< e.what();
//     // }    
// }

// Test CADToPointCloud constructor should return error: it is not a cad file
// TEST(TestCADToPointCloud, testCADnoExists)
// {
//     std::string cad_folder_path = LeicaUtils::findPointcloudFolderPath();
//     std::string file_name = "no_exists.ply";
//     std::string f = LeicaUtils::getFilePath(file_name);

//     int sample_points = 10000;
    
//     PointCloudRGB::Ptr cubeRGB {new PointCloudRGB};

//     try
//     {
//         CADToPointCloud cad2pc(f, sample_points);
//         cad2pc.convertCloud(cubeRGB);

//         ASSERT_TRUE(Utils::isValidCloud(cubeRGB));
//         ASSERT_GT(cubeRGB->size(),1);
//     }
//     catch(std::exception& e)
//     {
//         ADD_FAILURE()<< e.what();
//     }       
// }


// Run all the tests that were declared with TEST()
int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}